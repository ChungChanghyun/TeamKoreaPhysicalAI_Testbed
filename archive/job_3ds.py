"""
job_3ds.py — 3D Shuttle 전용 작업 관제 시스템.

동일 층 작업은 셔틀만으로 직행 처리.
타 층 작업은 셔틀 → 엘리베이터 → 셔틀 3단계 자동 orchestration.

작업 흐름
─────────
  동일 층:
    PENDING → SHUTTLE_SRC → DONE

  타 층:
    PENDING → SHUTTLE_SRC → WAIT_LIFT → LIFT_TRANSIT
            → SHUTTLE_DST → DONE

  SHUTTLE_SRC : 출발 층 셔틀이 화물을 픽업 → lift gate(타층) 또는 목적지(동일층)로 이송
  WAIT_LIFT   : 엘리베이터 호출 대기 + 적재
  LIFT_TRANSIT: 엘리베이터 층간 이동 + 하역
  SHUTTLE_DST : 도착 층 셔틀이 lift gate에서 화물 픽업 → 최종 목적지로 이송

DES 이벤트
──────────
  S3D_SHUTTLE_DONE   셔틀 구간 완료 (출발/도착 측)

  엘리베이터 이벤트는 elevator.py의 LIFT_MOVE_DONE / LIFT_XFER_DONE을
  그대로 사용 (콜백으로 연동).

구성 요소
─────────
  Job3DS               작업 데이터
  ShuttleFloor          한 층의 셔틀 풀 + 배차 (task_search / load_available)
  Job3DSController      전체 orchestrator

Usage
─────
  ctrl = Job3DSController(heap=env._heap)

  # 층 등록
  ctrl.add_floor('1', shuttle_ids=[0,1,2],
                 path_finder=floor1_bfs,
                 reassign_fn=lambda vid, path, t: env.reassign(...))

  # 엘리베이터 연결
  ctrl.set_elevator_ctrl(lift_ctrl)

  # 작업 생성
  ctrl.create_job(from_node='00000365', to_node='00000031', t=0.0)
  #   from_node은 F1, to_node은 F2 → 자동으로 타 층 작업 인식

  # env.step() 루프
  if ctrl.handle_event(ev):
      continue
"""
from __future__ import annotations
import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Tuple, Set
from collections import deque


# ── DES event type ───────────────────────────────────────────────────────────
S3D_SHUTTLE_DONE = 'S3D_SHUTTLE_DONE'

_S3D_EVENT_TYPES = frozenset({S3D_SHUTTLE_DONE})

_S3D_AGENT_ID = -2000   # job 이벤트용 agent_id


# ── Job states ───────────────────────────────────────────────────────────────

PENDING      = 'PENDING'
SHUTTLE_SRC  = 'SHUTTLE_SRC'    # 출발 층 셔틀 이송 중
WAIT_LIFT    = 'WAIT_LIFT'      # 엘리베이터 대기 + 적재 중
LIFT_TRANSIT = 'LIFT_TRANSIT'   # 엘리베이터 층간 이동 + 하역 중
SHUTTLE_DST  = 'SHUTTLE_DST'    # 도착 층 셔틀 이송 중
DONE         = 'DONE'
CANCELLED    = 'CANCELLED'

_TRANSITIONS = {
    PENDING:      (SHUTTLE_SRC, CANCELLED),
    SHUTTLE_SRC:  (WAIT_LIFT, DONE, CANCELLED),     # DONE: 동일 층
    WAIT_LIFT:    (LIFT_TRANSIT, CANCELLED),
    LIFT_TRANSIT: (SHUTTLE_DST, CANCELLED),
    SHUTTLE_DST:  (DONE, CANCELLED),
}


# ── Event (env_oht_des.Event 호환) ───────────────────────────────────────────

class Event:
    __slots__ = ('t', 'kind', 'agent_id', 'token', 'data')

    def __init__(self, t: float, kind: str, agent_id: int,
                 token: int, data=None):
        self.t        = t
        self.kind     = kind
        self.agent_id = agent_id
        self.token    = token
        self.data     = data

    def __lt__(self, other):
        return self.t < other.t


# ── Data objects ─────────────────────────────────────────────────────────────

@dataclass
class Job3DS:
    id:           int
    from_node:    str            # 화물 출발 노드
    to_node:      str            # 최종 목적 노드
    from_floor:   str            # 출발 층 ID
    to_floor:     str            # 도착 층 ID
    cross_floor:  bool           # 타 층 작업 여부
    state:        str = PENDING
    priority:     int = 0
    token:        int = 0

    # 셔틀 할당 정보
    src_shuttle:  Optional[int] = None   # 출발 측 셔틀 ID
    dst_shuttle:  Optional[int] = None   # 도착 측 셔틀 ID
    lift_id:      Optional[str] = None   # 사용 엘리베이터 ID
    lift_req_id:  Optional[int] = None   # 엘리베이터 요청 ID

    # 타 층 작업 시 중간 노드
    src_gate:     Optional[str] = None   # 출발 층 lift gate 노드
    dst_gate:     Optional[str] = None   # 도착 층 lift gate 노드

    # 타임스탬프
    t_created:      float = 0.0
    t_shuttle_src:  float = 0.0   # 출발 셔틀 배차
    t_at_gate:      float = 0.0   # gate 도착 (타 층)
    t_lift_start:   float = 0.0   # 엘리베이터 이동 시작
    t_lift_done:    float = 0.0   # 엘리베이터 하역 완료
    t_shuttle_dst:  float = 0.0   # 도착 셔틀 배차
    t_done:         float = 0.0


@dataclass
class Job3DSLogEntry:
    time:       float
    job_id:     int
    prev_state: str
    new_state:  str
    detail:     str = ''


@dataclass
class Shuttle:
    """한 층에서 운행하는 셔틀 메타데이터."""
    id:        int
    floor_id:  str
    cur_node:  Optional[str] = None
    status:    str = 'IDLE'      # IDLE | BUSY

    # KPI
    t_registered: float = 0.0
    t_last_busy:  float = 0.0
    total_busy:   float = 0.0
    jobs_done:    int   = 0


# ── ShuttleFloor ─────────────────────────────────────────────────────────────

class ShuttleFloor:
    """
    한 층의 셔틀 풀 관리.

    Parameters
    ----------
    floor_id     : str — 층 식별자
    path_finder  : callable(from_node, to_node) -> List[str]
    reassign_fn  : callable(shuttle_id, path, t) — 셔틀 주행 지시
    dist_func    : callable(node_a, node_b) -> float — 노드 간 거리
    """

    def __init__(self, floor_id: str, *,
                 path_finder: Callable = None,
                 reassign_fn: Callable = None,
                 dist_func: Callable = None):
        self.floor_id    = floor_id
        self.path_finder = path_finder
        self.reassign_fn = reassign_fn
        self.dist_func   = dist_func
        self._shuttles: Dict[int, Shuttle] = {}

    def add_shuttle(self, sid: int, cur_node: str = None, t: float = 0.0):
        self._shuttles[sid] = Shuttle(
            id=sid, floor_id=self.floor_id,
            cur_node=cur_node, t_registered=t,
        )

    def idle_shuttles(self) -> List[Shuttle]:
        return [s for s in self._shuttles.values() if s.status == 'IDLE']

    def get_shuttle(self, sid: int) -> Optional[Shuttle]:
        return self._shuttles.get(sid)

    def update_node(self, sid: int, node_id: str):
        s = self._shuttles.get(sid)
        if s:
            s.cur_node = node_id

    # ── Automod-style dispatch ───────────────────────────────────────────

    def task_search(self, shuttle: Shuttle,
                    pending_jobs: List[Job3DS],
                    is_src: bool = True
                    ) -> Optional[Tuple[int, Optional[list]]]:
        """
        셔틀이 idle → 수행 가능한 작업 탐색.

        Parameters
        ----------
        is_src : True면 출발 측 (from_node 기준), False면 도착 측 (dst_gate 기준)
        """
        if not pending_jobs:
            return None

        target_fn = (lambda j: j.from_node) if is_src else (lambda j: j.dst_gate)

        if self.dist_func and shuttle.cur_node:
            best_job, best_d = None, float('inf')
            for job in pending_jobs:
                target = target_fn(job)
                if target is None:
                    continue
                d = self.dist_func(shuttle.cur_node, target)
                if d < best_d:
                    best_d = d
                    best_job = job
        else:
            best_job = pending_jobs[0] if pending_jobs else None

        if best_job is None:
            return None

        target = target_fn(best_job)
        path = None
        if self.path_finder and shuttle.cur_node and target:
            path = self.path_finder(shuttle.cur_node, target)
        return (best_job.id, path)

    def load_available(self, job: Job3DS,
                       is_src: bool = True
                       ) -> Optional[Tuple[int, Optional[list]]]:
        """
        작업 생성 → idle 셔틀 탐색.
        """
        idle = self.idle_shuttles()
        if not idle:
            return None

        target = job.from_node if is_src else job.dst_gate

        if self.dist_func and target:
            best_s, best_d = None, float('inf')
            for s in idle:
                if s.cur_node is None:
                    continue
                d = self.dist_func(s.cur_node, target)
                if d < best_d:
                    best_d = d
                    best_s = s
        else:
            best_s = idle[0]

        if best_s is None:
            return None

        path = None
        if self.path_finder and best_s.cur_node and target:
            path = self.path_finder(best_s.cur_node, target)
        return (best_s.id, path)

    def dispatch(self, sid: int, path: Optional[list], t: float):
        """셔틀을 BUSY로 전환하고 주행 지시."""
        s = self._shuttles[sid]
        s.status     = 'BUSY'
        s.t_last_busy = t
        if path and self.reassign_fn:
            self.reassign_fn(sid, path, t)

    def release(self, sid: int, node_id: str, t: float):
        """셔틀을 IDLE로 복귀."""
        s = self._shuttles.get(sid)
        if s and s.status == 'BUSY':
            s.total_busy += (t - s.t_last_busy)
            s.jobs_done  += 1
            s.status      = 'IDLE'
            s.cur_node    = node_id

    @property
    def shuttles(self) -> Dict[int, Shuttle]:
        return self._shuttles


# ── Job3DSController ─────────────────────────────────────────────────────────

class Job3DSController:
    """
    3D Shuttle 전용 작업 orchestrator.

    Parameters
    ----------
    heap : list — 공유 이벤트 힙
    node_floor_map : dict — {node_id: floor_id}  노드→층 매핑
    """

    def __init__(self, heap: list,
                 node_floor_map: Dict[str, str] = None):
        self._heap = heap
        self._node_floor = node_floor_map or {}

        self._jobs:     Dict[int, Job3DS] = {}
        self._next_id:  int = 0
        self._token:    int = 0
        self._log:      List[Job3DSLogEntry] = []

        self._floors:   Dict[str, ShuttleFloor] = {}
        self._lift_ctrl = None   # ElevatorController

        # shuttle_id → 현재 수행 중인 job_id
        self._shuttle_job: Dict[int, int] = {}

    # ═════════════════════════════════════════════════════════════════════════
    #  설정
    # ═════════════════════════════════════════════════════════════════════════

    def set_node_floor_map(self, mapping: Dict[str, str]):
        """노드→층 매핑 설정. 맵 로드 후 호출."""
        self._node_floor = mapping

    def add_floor(self, floor: ShuttleFloor):
        """층 등록."""
        self._floors[floor.floor_id] = floor

    def set_elevator_ctrl(self, lift_ctrl):
        """ElevatorController 연결."""
        self._lift_ctrl = lift_ctrl

    def get_floor(self, floor_id: str) -> Optional[ShuttleFloor]:
        return self._floors.get(floor_id)

    def node_to_floor(self, node_id: str) -> Optional[str]:
        return self._node_floor.get(node_id)

    # ═════════════════════════════════════════════════════════════════════════
    #  작업 생성
    # ═════════════════════════════════════════════════════════════════════════

    def create_job(self, from_node: str, to_node: str,
                   t: float = 0.0, priority: int = 0) -> Job3DS:
        """
        작업 생성. 노드의 층을 자동 판별하여 동일/타 층 결정.
        """
        from_floor = self._node_floor.get(from_node)
        to_floor   = self._node_floor.get(to_node)
        if from_floor is None or to_floor is None:
            raise ValueError(
                f"Node floor unknown: {from_node}={from_floor}, "
                f"{to_node}={to_floor}")

        cross = (from_floor != to_floor)

        # 타 층이면 gate 노드 결정
        src_gate = None
        dst_gate = None
        if cross and self._lift_ctrl:
            lid = self._lift_ctrl.select_lift(from_floor, to_floor, t)
            if lid:
                lift = self._lift_ctrl.get_lift(lid)
                src_gate = lift.gate_nodes.get(from_floor)
                dst_gate = lift.gate_nodes.get(to_floor)

        job = Job3DS(
            id=self._next_id,
            from_node=from_node, to_node=to_node,
            from_floor=from_floor, to_floor=to_floor,
            cross_floor=cross,
            priority=priority, t_created=t,
            src_gate=src_gate, dst_gate=dst_gate,
        )
        self._jobs[job.id] = job
        self._next_id += 1
        self._record(t, job, '', PENDING)

        # load_available: 출발 층 셔틀 탐색
        self._try_dispatch_src(job, t)
        return job

    # ═════════════════════════════════════════════════════════════════════════
    #  DES 연동
    # ═════════════════════════════════════════════════════════════════════════

    def handle_event(self, ev) -> bool:
        """3DS 작업 이벤트 처리."""
        if ev.kind not in _S3D_EVENT_TYPES:
            return False
        if ev.agent_id != _S3D_AGENT_ID:
            return False
        if ev.token != self._token:
            return True   # stale
        # data = (job_id, phase)
        if ev.data is None:
            return True
        job_id, phase = ev.data
        job = self._jobs.get(job_id)
        if job is None or ev.token != job.token:
            return True

        if phase == 'src_done':
            self._on_src_shuttle_done(ev.t, job)
        elif phase == 'dst_done':
            self._on_dst_shuttle_done(ev.t, job)
        return True

    def on_shuttle_path_complete(self, shuttle_id: int,
                                 node_id: str, t: float) -> bool:
        """
        셔틀이 경로 끝에 도착했을 때 호출.
        env._on_try_advance()에서 호출.

        Returns True면 셔틀 정지 유지.
        """
        jid = self._shuttle_job.get(shuttle_id)
        if jid is None:
            return False
        job = self._jobs[jid]

        if job.state == SHUTTLE_SRC:
            if job.cross_floor and node_id == job.src_gate:
                # 출발 측: gate 도착 → 엘리베이터 호출
                self._on_src_at_gate(t, job, shuttle_id, node_id)
                return True
            elif not job.cross_floor and node_id == job.to_node:
                # 동일 층: 목적지 도착 → 완료
                self._on_same_floor_done(t, job, shuttle_id, node_id)
                return True

        if job.state == SHUTTLE_DST:
            if node_id == job.to_node:
                # 도착 측: 최종 목적지 도착 → 완료
                self._on_dst_at_dest(t, job, shuttle_id, node_id)
                return True

        return False

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — 출발 측 배차
    # ═════════════════════════════════════════════════════════════════════════

    def _try_dispatch_src(self, job: Job3DS, t: float):
        """출발 층에서 idle 셔틀을 찾아 배차."""
        floor = self._floors.get(job.from_floor)
        if floor is None:
            return

        result = floor.load_available(job, is_src=True)
        if result is None:
            return  # 빈 셔틀 없음 → pending 유지

        sid, path = result
        self._do_dispatch_src(job, sid, path, t, floor)

    def _do_dispatch_src(self, job: Job3DS, sid: int,
                         path: Optional[list], t: float,
                         floor: ShuttleFloor):
        """출발 측 셔틀 배차 실행."""
        self._transition(job, SHUTTLE_SRC, t)
        job.src_shuttle   = sid
        job.t_shuttle_src = t
        self._shuttle_job[sid] = job.id

        # 셔틀의 목적지: 타 층이면 gate, 동일 층이면 최종 목적지
        dest = job.src_gate if job.cross_floor else job.to_node

        # 경로가 화물 위치까지인 경우: 화물 위치 → 목적지 2-leg
        # 여기서는 단순화: 셔틀이 이미 화물 위치에 있다고 가정하거나
        # path_finder가 전체 경로를 제공
        if path is None and floor.path_finder:
            shuttle = floor.get_shuttle(sid)
            if shuttle and shuttle.cur_node:
                # 셔틀 현재 위치 → 화물 위치 → 목적지
                path = floor.path_finder(shuttle.cur_node, job.from_node)

        floor.dispatch(sid, path, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — 경로 완료 핸들러
    # ═════════════════════════════════════════════════════════════════════════

    def _on_src_at_gate(self, t: float, job: Job3DS,
                        shuttle_id: int, node_id: str):
        """출발 측 셔틀이 gate 도착 → 셔틀 해제 + 엘리베이터 호출."""
        job.t_at_gate = t
        # 셔틀 해제
        floor = self._floors.get(job.from_floor)
        if floor:
            floor.release(shuttle_id, node_id, t)
        self._shuttle_job.pop(shuttle_id, None)

        # 엘리베이터 호출
        self._transition(job, WAIT_LIFT, t)
        self._call_elevator(job, t)

        # 해제된 셔틀로 task_search
        if floor:
            shuttle = floor.get_shuttle(shuttle_id)
            if shuttle:
                self._task_search_floor(floor, shuttle, t)

    def _on_same_floor_done(self, t: float, job: Job3DS,
                            shuttle_id: int, node_id: str):
        """동일 층 작업 완료."""
        # 셔틀 해제
        floor = self._floors.get(job.from_floor)
        if floor:
            floor.release(shuttle_id, node_id, t)
        self._shuttle_job.pop(shuttle_id, None)

        self._transition(job, DONE, t)
        job.t_done = t

        # task_search
        if floor:
            shuttle = floor.get_shuttle(shuttle_id)
            if shuttle:
                self._task_search_floor(floor, shuttle, t)

    def _on_dst_at_dest(self, t: float, job: Job3DS,
                        shuttle_id: int, node_id: str):
        """도착 측 셔틀이 최종 목적지 도착 → 완료."""
        floor = self._floors.get(job.to_floor)
        if floor:
            floor.release(shuttle_id, node_id, t)
        self._shuttle_job.pop(shuttle_id, None)

        self._transition(job, DONE, t)
        job.t_done = t

        # task_search
        if floor:
            shuttle = floor.get_shuttle(shuttle_id)
            if shuttle:
                self._task_search_floor(floor, shuttle, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — 엘리베이터 연동
    # ═════════════════════════════════════════════════════════════════════════

    def _call_elevator(self, job: Job3DS, t: float):
        """엘리베이터에 이송 요청."""
        if self._lift_ctrl is None:
            return

        def on_pickup(tp):
            """엘리베이터 적재 완료 → LIFT_TRANSIT."""
            self._transition(job, LIFT_TRANSIT, tp)
            job.t_lift_start = tp

        def on_complete(tc):
            """엘리베이터 하역 완료 → 도착 측 셔틀 배차."""
            job.t_lift_done = tc
            self._on_lift_done(tc, job)

        result = self._lift_ctrl.request_auto(
            from_floor=job.from_floor,
            to_floor=job.to_floor,
            t=t,
            on_pickup=on_pickup,
            on_complete=on_complete,
            priority=job.priority,
        )
        if result:
            job.lift_id, job.lift_req_id = result

    def _on_lift_done(self, t: float, job: Job3DS):
        """엘리베이터 하역 완료 → 도착 층 셔틀 배차."""
        floor = self._floors.get(job.to_floor)
        if floor is None:
            return

        self._transition(job, SHUTTLE_DST, t)
        job.t_shuttle_dst = t

        # load_available: 도착 층에서 idle 셔틀 탐색
        result = floor.load_available(job, is_src=False)
        if result is None:
            # 빈 셔틀 없음 — pending에 등록하여 task_search에서 처리
            return

        sid, path = result
        self._do_dispatch_dst(job, sid, path, t, floor)

    def _do_dispatch_dst(self, job: Job3DS, sid: int,
                         path: Optional[list], t: float,
                         floor: ShuttleFloor):
        """도착 측 셔틀 배차 실행."""
        job.dst_shuttle = sid
        self._shuttle_job[sid] = job.id

        # gate → 최종 목적지
        if path is None and floor.path_finder:
            shuttle = floor.get_shuttle(sid)
            if shuttle and shuttle.cur_node and job.dst_gate:
                path = floor.path_finder(shuttle.cur_node, job.dst_gate)

        floor.dispatch(sid, path, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — task_search (셔틀 idle 시)
    # ═════════════════════════════════════════════════════════════════════════

    def _task_search_floor(self, floor: ShuttleFloor,
                           shuttle: Shuttle, t: float):
        """셔틀이 idle → 해당 층에서 수행 가능한 작업 탐색."""
        # 1) SHUTTLE_DST 대기 중인 작업 (도착 측 셔틀 필요)
        dst_waiting = [j for j in self._jobs.values()
                       if j.state == SHUTTLE_DST
                       and j.to_floor == floor.floor_id
                       and j.dst_shuttle is None]
        if dst_waiting:
            result = floor.task_search(shuttle, dst_waiting, is_src=False)
            if result:
                job_id, path = result
                job = self._jobs[job_id]
                self._do_dispatch_dst(job, shuttle.id, path, t, floor)
                return

        # 2) PENDING 작업 (출발 측 셔틀 필요)
        src_pending = [j for j in self._jobs.values()
                       if j.state == PENDING
                       and j.from_floor == floor.floor_id]
        if src_pending:
            result = floor.task_search(shuttle, src_pending, is_src=True)
            if result:
                job_id, path = result
                job = self._jobs[job_id]
                self._do_dispatch_src(job, shuttle.id, path, t, floor)
                return

    # ═════════════════════════════════════════════════════════════════════════
    #  작업 취소
    # ═════════════════════════════════════════════════════════════════════════

    def cancel_job(self, job_id: int, t: float):
        job = self._get(job_id)
        old = job.state
        job.state = CANCELLED
        job.token += 1
        self._record(t, job, old, CANCELLED)

        # 셔틀 해제
        for sid in (job.src_shuttle, job.dst_shuttle):
            if sid is not None:
                self._shuttle_job.pop(sid, None)
                for floor in self._floors.values():
                    s = floor.get_shuttle(sid)
                    if s and s.status == 'BUSY':
                        floor.release(sid, s.cur_node or '', t)

    # ═════════════════════════════════════════════════════════════════════════
    #  State machine / logging
    # ═════════════════════════════════════════════════════════════════════════

    def _transition(self, job: Job3DS, new_state: str, t: float):
        allowed = _TRANSITIONS.get(job.state, ())
        if new_state not in allowed:
            raise ValueError(
                f"Job3DS {job.id}: {job.state} → {new_state}")
        old = job.state
        job.state = new_state
        self._record(t, job, old, new_state)

    def _record(self, t, job, prev, new, detail=''):
        self._log.append(Job3DSLogEntry(
            time=t, job_id=job.id,
            prev_state=prev, new_state=new, detail=detail,
        ))

    def _get(self, job_id: int) -> Job3DS:
        job = self._jobs.get(job_id)
        if job is None:
            raise KeyError(f"Job3DS {job_id} not found")
        return job

    # ═════════════════════════════════════════════════════════════════════════
    #  Queries
    # ═════════════════════════════════════════════════════════════════════════

    def pending_jobs(self, floor_id: str = None) -> List[Job3DS]:
        jobs = [j for j in self._jobs.values() if j.state == PENDING]
        if floor_id:
            jobs = [j for j in jobs if j.from_floor == floor_id]
        return sorted(jobs, key=lambda j: -j.priority)

    def active_jobs(self) -> List[Job3DS]:
        active = {SHUTTLE_SRC, WAIT_LIFT, LIFT_TRANSIT, SHUTTLE_DST}
        return [j for j in self._jobs.values() if j.state in active]

    def completed_jobs(self) -> List[Job3DS]:
        return [j for j in self._jobs.values() if j.state == DONE]

    @property
    def event_log(self) -> List[Job3DSLogEntry]:
        return list(self._log)

    # ═════════════════════════════════════════════════════════════════════════
    #  KPI
    # ═════════════════════════════════════════════════════════════════════════

    def job_breakdown(self, job_id: int) -> dict:
        j = self._get(job_id)
        r = {
            'job_id': j.id, 'state': j.state,
            'from': j.from_node, 'to': j.to_node,
            'cross_floor': j.cross_floor,
            'from_floor': j.from_floor, 'to_floor': j.to_floor,
        }
        if j.state == DONE:
            r['total'] = j.t_done - j.t_created
            if j.cross_floor:
                r['shuttle_src']  = j.t_at_gate - j.t_shuttle_src
                r['wait_lift']    = j.t_lift_start - j.t_at_gate
                r['lift_transit'] = j.t_lift_done - j.t_lift_start
                r['shuttle_dst']  = j.t_done - j.t_shuttle_dst
            else:
                r['shuttle_direct'] = j.t_done - j.t_shuttle_src
        return r

    def kpi(self, t_now: float) -> dict:
        done = self.completed_jobs()
        cancelled = [j for j in self._jobs.values() if j.state == CANCELLED]
        same = [j for j in done if not j.cross_floor]
        cross = [j for j in done if j.cross_floor]

        result = {
            'jobs_created':    len(self._jobs),
            'jobs_completed':  len(done),
            'jobs_cancelled':  len(cancelled),
            'jobs_pending':    len(self.pending_jobs()),
            'jobs_active':     len(self.active_jobs()),
            'jobs_same_floor': len(same),
            'jobs_cross_floor': len(cross),
        }

        if same:
            leads = [j.t_done - j.t_created for j in same]
            result['avg_lead_same'] = sum(leads) / len(leads)

        if cross:
            leads   = [j.t_done - j.t_created for j in cross]
            shuttles = [j.t_at_gate - j.t_shuttle_src for j in cross]
            waits   = [j.t_lift_start - j.t_at_gate for j in cross]
            lifts   = [j.t_lift_done - j.t_lift_start for j in cross]
            dsts    = [j.t_done - j.t_shuttle_dst for j in cross]
            n = len(cross)
            result['avg_lead_cross']    = sum(leads) / n
            result['avg_shuttle_src']   = sum(shuttles) / n
            result['avg_wait_lift']     = sum(waits) / n
            result['avg_lift_transit']  = sum(lifts) / n
            result['avg_shuttle_dst']   = sum(dsts) / n

        # 층별 셔틀 KPI
        floor_details = {}
        for fid, floor in sorted(self._floors.items()):
            shuttles_info = []
            for s in floor.shuttles.values():
                busy = s.total_busy
                if s.status == 'BUSY':
                    busy += (t_now - s.t_last_busy)
                elapsed = t_now - s.t_registered
                shuttles_info.append({
                    'id': s.id,
                    'status': s.status,
                    'utilization': busy / elapsed if elapsed > 0 else 0.0,
                    'jobs_done': s.jobs_done,
                })
            floor_details[fid] = shuttles_info
        result['floor_details'] = floor_details

        # 처리량
        t_first = min((j.t_created for j in self._jobs.values()),
                      default=t_now)
        elapsed = t_now - t_first
        result['throughput'] = len(done) / elapsed if elapsed > 0 else 0.0

        return result

    def print_kpi(self, t_now: float):
        k = self.kpi(t_now)
        lines = [
            '',
            '═══════════════════════════════════════════',
            f'  3DS KPI Report  (t = {t_now:.1f}s)',
            '═══════════════════════════════════════════',
            '',
            f'  Created: {k["jobs_created"]}  '
            f'Done: {k["jobs_completed"]}  '
            f'Active: {k["jobs_active"]}  '
            f'Pending: {k["jobs_pending"]}',
            f'  Same-floor: {k["jobs_same_floor"]}  '
            f'Cross-floor: {k["jobs_cross_floor"]}',
            '',
        ]
        if 'avg_lead_same' in k:
            lines.append(
                f'  Same-floor avg lead: {k["avg_lead_same"]:.1f}s')
        if 'avg_lead_cross' in k:
            lines += [
                f'  Cross-floor avg lead:    {k["avg_lead_cross"]:.1f}s',
                f'    shuttle src:  {k["avg_shuttle_src"]:.1f}s',
                f'    wait lift:    {k["avg_wait_lift"]:.1f}s',
                f'    lift transit: {k["avg_lift_transit"]:.1f}s',
                f'    shuttle dst:  {k["avg_shuttle_dst"]:.1f}s',
            ]
        lines.append('')
        for fid, shuttles in sorted(k.get('floor_details', {}).items()):
            lines.append(f'  Floor {fid}:')
            for si in shuttles:
                lines.append(
                    f'    S{si["id"]:>2d}  {si["status"]:<5s}  '
                    f'util {si["utilization"]:>5.1%}  '
                    f'jobs {si["jobs_done"]:>3d}')
        lines += [
            '',
            f'  Throughput: {k["throughput"]:.4f} jobs/sec',
            '═══════════════════════════════════════════',
            '',
        ]
        print('\n'.join(lines))
