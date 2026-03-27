"""
job_event.py — DES 기반 반송 작업 관제 시스템.

env_oht_des 주행 엔진과 **동일한 이벤트 힙**(min-heap)을 공유합니다.
주행 이벤트(TRY_ADVANCE, SEGMENT_DONE, CATCH_UP, GAP_CLEAR)와
작업 이벤트(JOB_ARRIVE, LOAD_DONE, UNLOAD_DONE)가 시간순으로
자연스럽게 interleave됩니다.

작업 DES 이벤트
───────────────
  JOB_ARRIVE    새 반송 작업 도착 (Generator가 힙에 예약)
  LOAD_DONE     적재 완료 → 목적지로 실차 이동 개시
  UNLOAD_DONE   하역 완료 → 작업 종료, 차량 idle 복귀

Job 상태 전이
─────────────
  PENDING → ASSIGNED → PICKUP → LOADING → DELIVERING → UNLOADING → DONE
  어느 단계에서든 → CANCELLED 가능

env_oht_des 연동 (env 쪽 수정 3곳)
──────────────────────────────────

  # ── 0) 컨트롤러 생성 ──
  #    env 생성 후, step() 호출 전에 설정
  job_ctrl = JobDESController(
      heap         = env._heap,
      reassign_fn  = lambda vid, path, t: env.reassign(env._agents[vid], path, t),
      path_finder  = env.map.bfs,
  )
  env.job_ctrl = job_ctrl      # env에 참조 저장

  # ── 1) env.__init__() ──
  #    self.job_ctrl = None     # 외부에서 주입

  # ── 2) env.step() — heappop 직후, agent 조회 전에 삽입 ──
  #    ev = heapq.heappop(self._heap)
  #    if self.job_ctrl and self.job_ctrl.handle_event(ev):
  #        continue             # job 이벤트 처리 완료
  #    agent = self._agents.get(ev.agent_id)
  #    if agent is None:
  #        continue
  #    ...

  # ── 3) env._on_try_advance() — 경로 완료 판정 수정 ──
  #    if nxt_idx >= len(path):
  #        if self.job_ctrl and self.job_ctrl.on_path_complete(
  #                agent.id, agent.cur_node, t):
  #            agent.state = IDLE    # 적재/하역 대기 (DONE 아님)
  #            return
  #        agent.state = DONE
  #        return
"""
from __future__ import annotations
import heapq
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Callable


# ── Job DES event types ──────────────────────────────────────────────────────
# env_oht_des의 TRY_ADVANCE / SEGMENT_DONE / CATCH_UP / GAP_CLEAR와 겹치지 않음
JOB_ARRIVE  = 'JOB_ARRIVE'
LOAD_DONE   = 'LOAD_DONE'
UNLOAD_DONE = 'UNLOAD_DONE'

_JOB_EVENT_TYPES = frozenset({JOB_ARRIVE, LOAD_DONE, UNLOAD_DONE})

# JOB_ARRIVE 이벤트의 agent_id (특정 차량에 귀속되지 않음)
_GEN_AGENT_ID = -1


# ── Job states ───────────────────────────────────────────────────────────────

PENDING    = 'PENDING'
ASSIGNED   = 'ASSIGNED'
PICKUP     = 'PICKUP'          # 공차 이동 중 (픽업 위치로)
LOADING    = 'LOADING'
DELIVERING = 'DELIVERING'      # 실차 이동 중 (목적지로)
UNLOADING  = 'UNLOADING'
DONE       = 'DONE'
CANCELLED  = 'CANCELLED'

_TRANSITIONS = {
    PENDING:    (ASSIGNED, CANCELLED),
    ASSIGNED:   (PICKUP, CANCELLED),
    PICKUP:     (LOADING, CANCELLED),
    LOADING:    (DELIVERING, CANCELLED),
    DELIVERING: (UNLOADING, CANCELLED),
    UNLOADING:  (DONE, CANCELLED),
}


# ── Event (env_oht_des.Event과 동일 인터페이스) ──────────────────────────────
# 통합 시 env_oht_des.Event를 import하여 대체 가능

class Event:
    """env_oht_des.Event과 동일한 슬롯/비교 — 같은 힙에서 공존."""
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
class Job:
    id:         int
    from_port:  str              # 픽업 노드 ID
    to_port:    str              # 배달 노드 ID
    state:      str = PENDING
    vehicle_id: Optional[int] = None
    priority:   int = 0
    token:      int = 0          # LOAD_DONE / UNLOAD_DONE 무효화용

    # 각 단계 진입 시각
    t_created:    float = 0.0
    t_assigned:   float = 0.0
    t_pickup:     float = 0.0    # 공차 이동 시작
    t_loading:    float = 0.0    # 적재 시작
    t_delivering: float = 0.0   # 실차 이동 시작
    t_unloading:  float = 0.0   # 하역 시작
    t_done:       float = 0.0


@dataclass
class JobLogEntry:
    """작업 상태 전이 기록 한 건."""
    time:       float
    job_id:     int
    prev_state: str
    new_state:  str
    vehicle_id: Optional[int] = None
    detail:     str = ''


@dataclass
class Vehicle:
    """DES agent와 1:1 대응하는 상위 레벨 차량 메타데이터."""
    id:        int
    home_node: str
    cur_node:  Optional[str] = None
    status:    str = 'IDLE'      # IDLE | BUSY | RETURNING

    # 가동 시간 추적 — KPI 산출용
    t_registered:  float = 0.0             # 등록 시각
    t_last_busy:   float = 0.0             # 최근 BUSY 전환 시각
    total_busy:    float = 0.0             # 누적 BUSY 시간
    jobs_done:     int   = 0               # 완료한 작업 수


# ── JobDESController ─────────────────────────────────────────────────────────

class JobDESController:
    """
    DES 기반 반송 작업 관제기.

    env_oht_des의 이벤트 힙을 공유하여 작업 이벤트를 삽입·처리합니다.
    tick() 폴링 없이, 이벤트 체인만으로 작업 라이프사이클이 진행됩니다.

    Parameters
    ----------
    heap            : list — env_oht_des._heap (shared min-heap reference)
    load_duration   : float — 적재 소요 시간 (sim seconds)
    unload_duration : float — 하역 소요 시간 (sim seconds)
    path_finder     : callable(from_node, to_node) -> List[str]
    reassign_fn     : callable(vehicle_id, path, t)
                      env.reassign()를 vehicle_id 기반으로 호출하는 래퍼.
    """

    def __init__(self, heap: list, *,
                 load_duration:   float = 5.0,
                 unload_duration: float = 5.0,
                 path_finder:  Callable = None,
                 reassign_fn:  Callable = None):
        self._heap           = heap
        self.load_duration   = load_duration
        self.unload_duration = unload_duration
        self.path_finder     = path_finder
        self.reassign_fn     = reassign_fn

        self._jobs:        Dict[int, Job]     = {}
        self._next_id:     int                = 0
        self._log:         List[JobLogEntry]  = []

        self._vehicles:    Dict[int, Vehicle] = {}
        self._vehicle_job: Dict[int, int]     = {}   # vehicle_id -> job_id

        # Generator / Dispatcher (set_generator / set_dispatcher로 등록)
        self._gen:        Optional[JobGenerator] = None
        self._gen_token:  int = 0
        self._dispatcher: Optional[Dispatcher]   = None

    # ── Vehicle registration ─────────────────────────────────────────────────

    def register_vehicle(self, vid: int, home_node: str,
                         cur_node: str = None, t: float = 0.0):
        self._vehicles[vid] = Vehicle(
            id=vid, home_node=home_node,
            cur_node=cur_node or home_node,
            t_registered=t,
        )

    def update_vehicle_node(self, vid: int, node_id: str):
        """DES 엔진에서 차량 위치 갱신 시 호출."""
        v = self._vehicles.get(vid)
        if v:
            v.cur_node = node_id

    # ── Generator / Dispatcher 설정 ──────────────────────────────────────────

    def set_generator(self, gen: 'JobGenerator'):
        self._gen = gen

    def set_dispatcher(self, disp: 'Dispatcher'):
        self._dispatcher = disp

    def start_generation(self, t: float = 0.0):
        """첫 JOB_ARRIVE 이벤트를 힙에 예약. 시뮬레이션 시작 시 1회 호출."""
        if self._gen:
            self._schedule_next_arrival(t)

    # ═════════════════════════════════════════════════════════════════════════
    #  env_oht_des 연동 인터페이스 (2개)
    # ═════════════════════════════════════════════════════════════════════════

    def handle_event(self, ev) -> bool:
        """
        작업 이벤트이면 처리하고 True 반환.  주행 이벤트이면 False.

        env_oht_des.step() 루프에서 heappop 직후에 호출:
            ev = heapq.heappop(self._heap)
            if job_ctrl.handle_event(ev):
                continue   # 작업 이벤트 — 주행 핸들러 스킵
        """
        if ev.kind not in _JOB_EVENT_TYPES:
            return False

        if ev.kind == JOB_ARRIVE:
            if ev.token != self._gen_token:
                return True                # stale (generator reset 등)
            self._on_job_arrive(ev.t)
            return True

        # LOAD_DONE / UNLOAD_DONE — data에 job_id 저장
        job = self._jobs.get(ev.data)
        if job is None or ev.token != job.token:
            return True                    # stale or cancelled

        if ev.kind == LOAD_DONE:
            self._on_load_done(ev.t, job)
        elif ev.kind == UNLOAD_DONE:
            self._on_unload_done(ev.t, job)
        return True

    def on_path_complete(self, vehicle_id: int, node_id: str,
                         t: float) -> bool:
        """
        차량이 경로 끝에 도착했을 때 env_oht_des에서 호출.

        env._on_try_advance()에서 nxt_idx >= len(path) 직전에:
            if job_ctrl and job_ctrl.on_path_complete(agent.id,
                                                      agent.cur_node, t):
                agent.state = IDLE   # DONE 대신 — 적재/하역 후 재출발
                return

        Returns
        -------
        True  — 적재 또는 하역 진행 중. 차량 정지 유지.
        False — 이 차량에 활성 작업 없음. 기존 로직(DONE) 수행.
        """
        jid = self._vehicle_job.get(vehicle_id)
        if jid is None:
            return False
        job = self._jobs[jid]

        if job.state == PICKUP and node_id == job.from_port:
            # 픽업 도착 → 적재 시작
            self._transition(job, LOADING, t)
            self._post_job_event(
                t + self.load_duration, LOAD_DONE, vehicle_id, job)
            return True

        if job.state == DELIVERING and node_id == job.to_port:
            # 목적지 도착 → 하역 시작
            self._transition(job, UNLOADING, t)
            self._post_job_event(
                t + self.unload_duration, UNLOAD_DONE, vehicle_id, job)
            return True

        return False

    # ═════════════════════════════════════════════════════════════════════════
    #  Job 생성 / 취소 (외부 직접 호출)
    # ═════════════════════════════════════════════════════════════════════════

    def create_job(self, from_port: str, to_port: str,
                   t: float = 0.0, priority: int = 0) -> Job:
        """작업 생성. 즉시 load_available 트리거."""
        job = Job(id=self._next_id, from_port=from_port,
                  to_port=to_port, priority=priority, t_created=t)
        self._jobs[job.id] = job
        self._next_id += 1
        self._record(t, job, '', PENDING)
        # Automod: load 생성 → load_available
        self._load_available(job, t)
        return job

    def cancel_job(self, job_id: int, t: float):
        """작업 취소. pending LOAD_DONE/UNLOAD_DONE 이벤트 무효화."""
        job = self._get(job_id)
        old = job.state
        job.state = CANCELLED
        job.token += 1              # 힙에 남은 이벤트 무효화
        self._record(t, job, old, CANCELLED)
        if job.vehicle_id is not None:
            self._vehicle_job.pop(job.vehicle_id, None)
            v = self._vehicles.get(job.vehicle_id)
            if v and v.status == 'BUSY':
                v.total_busy += (t - v.t_last_busy)
                v.status = 'IDLE'
                # 차량 idle 복귀 → task_search
                self._task_search(v, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — event handlers
    # ═════════════════════════════════════════════════════════════════════════

    def _on_job_arrive(self, t: float):
        """JOB_ARRIVE 처리: 작업 생성 → 배차 시도 → 다음 도착 예약."""
        if not self._gen:
            return
        fp, tp, pri = self._gen.pick_job()
        # max_pending 초과 검사
        if len(self.pending_jobs()) < self._gen.max_pending:
            self.create_job(fp, tp, t, pri)
        # 다음 작업 도착 예약 (항상)
        self._schedule_next_arrival(t)

    def _on_load_done(self, t: float, job: Job):
        """적재 완료 → DELIVERING 전이 → 목적지 경로로 reassign."""
        self._transition(job, DELIVERING, t)
        if self.path_finder and self.reassign_fn:
            path = self.path_finder(job.from_port, job.to_port)
            if path:
                self.reassign_fn(job.vehicle_id, path, t)

    def _on_unload_done(self, t: float, job: Job):
        """하역 완료 → DONE 전이 → 차량 idle 복귀 → task_search."""
        self._transition(job, DONE, t)
        vid = job.vehicle_id
        self._vehicle_job.pop(vid, None)
        v = self._vehicles.get(vid)
        if v:
            v.total_busy += (t - v.t_last_busy)
            v.jobs_done  += 1
            v.status      = 'IDLE'
            v.cur_node    = job.to_port
            # Automod: 작업 완료 → task_search
            self._task_search(v, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Automod-style dispatch triggers
    #
    #  task_search   : 차량이 idle이 됐을 때 호출.
    #                  "할 일 있나?" → pending load 중 최적 작업 탐색·배차.
    #
    #  load_available: load가 새로 생성됐을 때 호출.
    #                  "빈 차 있나?" → idle 차량 중 최적 차량 탐색·배차.
    #
    #  두 함수 모두 단일 (vehicle, job) 쌍을 매칭.
    #  Automod Path Mover의 이벤트 기반 배차 구조와 동일한 트리거.
    # ═════════════════════════════════════════════════════════════════════════

    def _task_search(self, vehicle: Vehicle, t: float):
        """
        차량이 idle 됐을 때 호출 — pending load를 찾아 배차.

        Automod: vehicle completes task → task search procedure.
        전략에 따라 가장 가까운 load, 우선순위 높은 load 등을 선택.
        """
        pending = self.pending_jobs()
        if not pending:
            return  # 할 일 없음 — 차량 idle 유지

        # Dispatcher가 있으면 전략 기반 선택
        if self._dispatcher:
            best = self._dispatcher.select_job_for_vehicle(
                vehicle, pending, self.path_finder)
        else:
            # fallback: priority 가장 높은 작업
            best = (pending[0].id, None)

        if best is None:
            return

        job_id, path = best
        if path is None and self.path_finder and vehicle.cur_node:
            path = self.path_finder(
                vehicle.cur_node, self._jobs[job_id].from_port)
        self._do_dispatch(vehicle.id, job_id, path, t)

    def _load_available(self, job: Job, t: float):
        """
        load가 생성됐을 때 호출 — idle 차량을 찾아 배차.

        Automod: load arrives at pickup station → load available procedure.
        전략에 따라 가장 가까운 차량 등을 선택.
        """
        idle = self.idle_vehicles()
        if not idle:
            return  # 빈 차 없음 — 작업 pending 유지

        # Dispatcher가 있으면 전략 기반 선택
        if self._dispatcher:
            best = self._dispatcher.select_vehicle_for_job(
                job, idle, self.path_finder)
        else:
            # fallback: 첫 번째 idle 차량
            v = idle[0]
            path = None
            if self.path_finder and v.cur_node:
                path = self.path_finder(v.cur_node, job.from_port)
            best = (v.id, path)

        if best is None:
            return

        vid, path = best
        self._do_dispatch(vid, job.id, path, t)

    def _do_dispatch(self, vid: int, job_id: int,
                     path: Optional[list], t: float):
        """단일 배차: 상태 전이 + env reassign."""
        job = self._jobs[job_id]
        v   = self._vehicles[vid]

        # PENDING → ASSIGNED
        self._transition(job, ASSIGNED, t)
        job.vehicle_id = vid
        self._vehicle_job[vid] = job_id
        v.status     = 'BUSY'
        v.t_last_busy = t

        # ASSIGNED → PICKUP
        self._transition(job, PICKUP, t)

        # env에 경로 전달
        if path and self.reassign_fn:
            self.reassign_fn(vid, path, t)

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — event posting
    # ═════════════════════════════════════════════════════════════════════════

    def _post_job_event(self, t: float, kind: str,
                        agent_id: int, job: Job):
        """작업 이벤트를 공유 힙에 삽입. token = job.token."""
        heapq.heappush(self._heap,
                       Event(t, kind, agent_id, job.token, job.id))

    def _schedule_next_arrival(self, t: float):
        """다음 JOB_ARRIVE 이벤트 예약."""
        if not self._gen:
            return
        dt = self._gen.next_interval()
        self._gen_token += 1
        heapq.heappush(self._heap,
                       Event(t + dt, JOB_ARRIVE, _GEN_AGENT_ID,
                             self._gen_token, None))

    # ═════════════════════════════════════════════════════════════════════════
    #  Internal — state machine
    # ═════════════════════════════════════════════════════════════════════════

    def _transition(self, job: Job, new_state: str, t: float):
        allowed = _TRANSITIONS.get(job.state, ())
        if new_state not in allowed:
            raise ValueError(
                f"Job {job.id}: invalid transition {job.state} → {new_state}")
        old = job.state
        job.state = new_state
        # 타임스탬프 자동 기록
        ts_attr = f't_{new_state.lower()}'
        if hasattr(job, ts_attr):
            setattr(job, ts_attr, t)
        self._record(t, job, old, new_state)

    def _record(self, t: float, job: Job, prev: str, new: str,
                detail: str = ''):
        self._log.append(JobLogEntry(
            time=t, job_id=job.id, prev_state=prev,
            new_state=new, vehicle_id=job.vehicle_id, detail=detail,
        ))

    def _get(self, job_id: int) -> Job:
        job = self._jobs.get(job_id)
        if job is None:
            raise KeyError(f"Job {job_id} not found")
        return job

    # ═════════════════════════════════════════════════════════════════════════
    #  Queries
    # ═════════════════════════════════════════════════════════════════════════

    def pending_jobs(self) -> List[Job]:
        """미배정 작업 (priority 내림차순)."""
        return sorted(
            [j for j in self._jobs.values() if j.state == PENDING],
            key=lambda j: -j.priority,
        )

    def active_jobs(self) -> List[Job]:
        """현재 수행 중인 작업 (ASSIGNED ~ UNLOADING)."""
        active = {ASSIGNED, PICKUP, LOADING, DELIVERING, UNLOADING}
        return [j for j in self._jobs.values() if j.state in active]

    def completed_jobs(self) -> List[Job]:
        return [j for j in self._jobs.values() if j.state == DONE]

    def vehicle_job(self, vid: int) -> Optional[Job]:
        jid = self._vehicle_job.get(vid)
        return self._jobs.get(jid) if jid is not None else None

    def idle_vehicles(self) -> List[Vehicle]:
        return [v for v in self._vehicles.values() if v.status == 'IDLE']

    @property
    def event_log(self) -> List[JobLogEntry]:
        return list(self._log)

    # ═════════════════════════════════════════════════════════════════════════
    #  Stats / KPI
    # ═════════════════════════════════════════════════════════════════════════

    def job_breakdown(self, job_id: int) -> dict:
        """개별 작업 소요 시간 분석."""
        j = self._get(job_id)
        r = {'job_id': j.id, 'state': j.state,
             'from': j.from_port, 'to': j.to_port,
             'vehicle_id': j.vehicle_id}
        if j.state == DONE:
            r['total']         = j.t_done - j.t_created
            r['wait']          = j.t_assigned - j.t_created
            r['empty_travel']  = j.t_loading - j.t_pickup
            r['loading']       = j.t_delivering - j.t_loading
            r['loaded_travel'] = j.t_unloading - j.t_delivering
            r['unloading']     = j.t_done - j.t_unloading
        return r

    def vehicle_kpi(self, vid: int, t_now: float) -> dict:
        """개별 차량 KPI."""
        v = self._vehicles.get(vid)
        if v is None:
            raise KeyError(f"Vehicle {vid} not found")
        elapsed = t_now - v.t_registered
        busy = v.total_busy
        # 현재 BUSY 상태면 진행 중인 시간 포함
        if v.status == 'BUSY':
            busy += (t_now - v.t_last_busy)
        return {
            'vehicle_id':    v.id,
            'status':        v.status,
            'elapsed':       elapsed,
            'busy_time':     busy,
            'idle_time':     elapsed - busy,
            'utilization':   busy / elapsed if elapsed > 0 else 0.0,
            'jobs_done':     v.jobs_done,
        }

    def kpi(self, t_now: float) -> dict:
        """
        전체 시스템 KPI 리포트.

        Returns
        -------
        dict with keys:
          ── 작업 현황 ──
          jobs_created      : 총 생성 작업 수
          jobs_completed    : 완료 작업 수
          jobs_cancelled    : 취소 작업 수
          jobs_pending      : 미배정 대기 중
          jobs_active       : 수행 중 (ASSIGNED ~ UNLOADING)

          ── 시간 분석 (완료 작업 기준, 초) ──
          avg_lead_time     : 평균 리드타임 (생성 → 완료)
          max_lead_time     : 최대 리드타임
          avg_wait          : 평균 대기시간 (생성 → 배정)
          avg_empty_travel  : 평균 공차 이동시간
          avg_loading       : 평균 적재시간
          avg_loaded_travel : 평균 실차 이동시간
          avg_unloading     : 평균 하역시간
          empty_ratio       : 공차 이동 비율 (공차 / (공차 + 실차))

          ── 처리량 ──
          throughput        : 완료 작업 수 / 경과 시간 (jobs/sec)

          ── 차량 ──
          vehicles_total    : 등록 차량 수
          vehicles_idle     : 현재 idle
          avg_utilization   : 평균 가동률
          vehicle_details   : 차량별 상세 KPI 리스트
        """
        done = self.completed_jobs()
        cancelled = [j for j in self._jobs.values() if j.state == CANCELLED]

        result: dict = {
            # ── 작업 현황
            'jobs_created':   len(self._jobs),
            'jobs_completed': len(done),
            'jobs_cancelled': len(cancelled),
            'jobs_pending':   len(self.pending_jobs()),
            'jobs_active':    len(self.active_jobs()),
        }

        # ── 시간 분석 (완료 작업)
        if done:
            leads   = [j.t_done - j.t_created for j in done]
            waits   = [j.t_assigned - j.t_created for j in done]
            empties = [j.t_loading - j.t_pickup for j in done]
            loads   = [j.t_delivering - j.t_loading for j in done]
            loadeds = [j.t_unloading - j.t_delivering for j in done]
            unloads = [j.t_done - j.t_unloading for j in done]

            n = len(done)
            result['avg_lead_time']     = sum(leads) / n
            result['max_lead_time']     = max(leads)
            result['avg_wait']          = sum(waits) / n
            result['avg_empty_travel']  = sum(empties) / n
            result['avg_loading']       = sum(loads) / n
            result['avg_loaded_travel'] = sum(loadeds) / n
            result['avg_unloading']     = sum(unloads) / n

            total_empty  = sum(empties)
            total_loaded = sum(loadeds)
            total_travel = total_empty + total_loaded
            result['empty_ratio'] = (total_empty / total_travel
                                     if total_travel > 0 else 0.0)

        # ── 처리량
        t_first = min((j.t_created for j in self._jobs.values()),
                      default=t_now)
        elapsed = t_now - t_first
        result['throughput'] = len(done) / elapsed if elapsed > 0 else 0.0

        # ── 차량 KPI
        v_details = []
        utils = []
        for vid in sorted(self._vehicles):
            vk = self.vehicle_kpi(vid, t_now)
            v_details.append(vk)
            utils.append(vk['utilization'])

        result['vehicles_total']   = len(self._vehicles)
        result['vehicles_idle']    = len(self.idle_vehicles())
        result['avg_utilization']  = (sum(utils) / len(utils)
                                      if utils else 0.0)
        result['vehicle_details']  = v_details

        return result

    def print_kpi(self, t_now: float):
        """KPI를 포맷팅하여 출력."""
        k = self.kpi(t_now)
        lines = [
            '',
            '═══════════════════════════════════════════',
            f'  KPI Report  (t = {t_now:.1f}s)',
            '═══════════════════════════════════════════',
            '',
            '── Jobs ──',
            f'  Created:   {k["jobs_created"]:>5d}',
            f'  Completed: {k["jobs_completed"]:>5d}',
            f'  Cancelled: {k["jobs_cancelled"]:>5d}',
            f'  Pending:   {k["jobs_pending"]:>5d}',
            f'  Active:    {k["jobs_active"]:>5d}',
            '',
        ]
        if k['jobs_completed'] > 0:
            lines += [
                '── Time Analysis (completed jobs) ──',
                f'  Avg lead time:     {k["avg_lead_time"]:>8.1f}s',
                f'  Max lead time:     {k["max_lead_time"]:>8.1f}s',
                f'  Avg wait:          {k["avg_wait"]:>8.1f}s',
                f'  Avg empty travel:  {k["avg_empty_travel"]:>8.1f}s',
                f'  Avg loading:       {k["avg_loading"]:>8.1f}s',
                f'  Avg loaded travel: {k["avg_loaded_travel"]:>8.1f}s',
                f'  Avg unloading:     {k["avg_unloading"]:>8.1f}s',
                f'  Empty ratio:       {k["empty_ratio"]:>7.1%}',
                '',
                '── Throughput ──',
                f'  {k["throughput"]:.4f} jobs/sec',
                '',
            ]
        lines += [
            '── Vehicles ──',
            f'  Total: {k["vehicles_total"]}   '
            f'Idle: {k["vehicles_idle"]}   '
            f'Avg util: {k["avg_utilization"]:.1%}',
        ]
        for vd in k['vehicle_details']:
            lines.append(
                f'    V{vd["vehicle_id"]:>2d}  '
                f'{vd["status"]:<6s}  '
                f'util {vd["utilization"]:>5.1%}  '
                f'busy {vd["busy_time"]:>7.1f}s  '
                f'jobs {vd["jobs_done"]:>3d}'
            )
        lines += ['═══════════════════════════════════════════', '']
        print('\n'.join(lines))


# ── JobGenerator ─────────────────────────────────────────────────────────────

class JobGenerator:
    """
    작업 생성 패턴 정의.  DES 이벤트 기반 — tick() 불필요.

    JobDESController가 JOB_ARRIVE 이벤트를 처리할 때
    pick_job()으로 (from, to) 쌍을 얻고,
    next_interval()로 다음 도착까지의 시간차를 얻습니다.

    Parameters
    ----------
    ports       : 포트 노드 ID 목록
    mode        : 'periodic' | 'poisson'
    interval    : 생성 간격 (periodic: 고정, poisson: 평균)
    max_pending : 미배정 작업 상한 (초과 시 생성 보류, 다음 JOB_ARRIVE에서 재시도)
    pairs       : 지정 (from, to) 쌍. None이면 ports에서 랜덤.
    seed        : 난수 시드
    """

    def __init__(self, ports: List[str], *,
                 mode: str = 'periodic',
                 interval: float = 30.0,
                 max_pending: int = 10,
                 pairs: Optional[List[Tuple[str, str]]] = None,
                 seed: int = None):
        self.ports       = list(ports)
        self.mode        = mode
        self.interval    = interval
        self.max_pending = max_pending
        self.pairs       = pairs
        self._rng        = random.Random(seed)

    def next_interval(self) -> float:
        """다음 JOB_ARRIVE까지 시간 간격."""
        if self.mode == 'poisson':
            return self._rng.expovariate(1.0 / self.interval)
        return self.interval

    def pick_job(self) -> Tuple[str, str, int]:
        """(from_port, to_port, priority) 반환."""
        if self.pairs:
            fp, tp = self._rng.choice(self.pairs)
        else:
            fp = self._rng.choice(self.ports)
            tp = self._rng.choice([p for p in self.ports if p != fp])
        return fp, tp, 0


# ── Dispatcher ───────────────────────────────────────────────────────────────

class Dispatcher:
    """
    배차 전략 — Automod의 task_search / load_available 양방향 매칭.

    select_job_for_vehicle : 차량 기준 → 최적 작업 선택  (task_search)
    select_vehicle_for_job : 작업 기준 → 최적 차량 선택  (load_available)

    Parameters
    ----------
    strategy  : 'nearest' | 'priority' | 'round_robin'
    dist_func : callable(node_a, node_b) -> float — nearest 전략용
    """

    def __init__(self, *,
                 strategy: str = 'nearest',
                 dist_func: Callable = None):
        self.strategy  = strategy
        self.dist_func = dist_func
        self._rr_idx   = 0

    def select_job_for_vehicle(
            self, vehicle: Vehicle, pending: List[Job],
            path_finder: Callable = None
    ) -> Optional[Tuple[int, Optional[list]]]:
        """
        task_search: idle 차량에게 최적의 pending 작업을 골라줌.

        Returns (job_id, path) or None.
        """
        if not pending:
            return None

        if self.strategy == 'nearest' and self.dist_func:
            # 차량에서 가장 가까운 pickup
            best_job, best_d = None, float('inf')
            for job in pending:
                if vehicle.cur_node is None:
                    continue
                d = self.dist_func(vehicle.cur_node, job.from_port)
                if d < best_d:
                    best_d   = d
                    best_job = job
            if best_job is None:
                return None
            path = None
            if path_finder and vehicle.cur_node:
                path = path_finder(vehicle.cur_node, best_job.from_port)
            return (best_job.id, path)

        if self.strategy == 'priority':
            # priority 내림차순 정렬돼 있으므로 첫 번째
            job = pending[0]
            path = None
            if path_finder and vehicle.cur_node:
                path = path_finder(vehicle.cur_node, job.from_port)
            return (job.id, path)

        # round_robin / fallback
        idx = self._rr_idx % len(pending)
        self._rr_idx += 1
        job = pending[idx]
        path = None
        if path_finder and vehicle.cur_node:
            path = path_finder(vehicle.cur_node, job.from_port)
        return (job.id, path)

    def select_vehicle_for_job(
            self, job: Job, idle: List[Vehicle],
            path_finder: Callable = None
    ) -> Optional[Tuple[int, Optional[list]]]:
        """
        load_available: 새 작업에 최적의 idle 차량을 골라줌.

        Returns (vehicle_id, path) or None.
        """
        if not idle:
            return None

        if self.strategy == 'nearest' and self.dist_func:
            # 작업 pickup에서 가장 가까운 차량
            best_v, best_d = None, float('inf')
            for v in idle:
                if v.cur_node is None:
                    continue
                d = self.dist_func(v.cur_node, job.from_port)
                if d < best_d:
                    best_d = d
                    best_v = v
            if best_v is None:
                return None
            path = None
            if path_finder and best_v.cur_node:
                path = path_finder(best_v.cur_node, job.from_port)
            return (best_v.id, path)

        # round_robin / priority / fallback
        idx = self._rr_idx % len(idle)
        self._rr_idx += 1
        v = idle[idx]
        path = None
        if path_finder and v.cur_node:
            path = path_finder(v.cur_node, job.from_port)
        return (v.id, path)
