"""
planner.py — CBS+SIPP planner adapter for pkl-based maps.

ACS_focal 솔버를 그대로 사용하되, 격자 .map 파일 초기화를 우회하고
pkl의 Stop/Move state를 직접 연결합니다.

Usage
─────
    planner = PklPlanner('c10_map.pkl')
    node_paths = planner.plan_random(n_agents=3)
    # [['16', '31', '55'], ['42', '66', ...], ...]
"""
from __future__ import annotations
import sys, os, pickle, random
from typing import List, Optional

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'solvers'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map_gen'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'utils'))


def _load_pkl_raw(pkl_path: str) -> dict:
    """Load pkl with State class injected into __main__ for unpickling."""
    from Generalized_251012_Affect_state_no_rot import State
    import __main__
    __main__.State = State
    with open(pkl_path, 'rb') as f:
        return pickle.load(f)


class PklPlanner:
    """
    Offline CBS+SIPP planner wrapper around ACS_focal.

    ACS_focal.__init__ 는 격자 텍스트 파일을 필요로 하므로,
    object.__new__ 로 인스턴스를 만들고 필요한 속성만 직접 설정합니다.
    이후 ACS_focal 의 bound method(find_solution, compute_plan 등)를
    그대로 사용합니다.
    """

    def __init__(self, pkl_path: str, focal_bound: int = 6):
        from ACS_graph_grid_focal_crisscross_heapcost import ACS_focal, AGV

        data = _load_pkl_raw(pkl_path)

        # ── ACS_focal 인스턴스를 __init__ 없이 생성 ──────────────────────────
        env = object.__new__(ACS_focal)
        env.Stop_state        = data['Stop_state']
        env.Move_state        = data['Move_state']
        env.collision_profile = data['collision_profile']
        env.focal_bound       = focal_bound
        env.AGVs              = {}
        env.lam               = {}

        # ── 역방향 state 그래프 + heuristic 테이블 사전 계산 ─────────────────
        env.build_reverse_state_graph_weighted()          # revW, revW_divisor
        all_goal_ids = list(env.Stop_state.keys())
        env.compute_all_highway_heuristic(all_goal_ids)  # highway_heuristic_table

        self._env     = env
        self._AGV_cls = AGV
        self.od_pairs = data.get('od_pairs', [])         # [(src_node, dst_node), ...]

        # node_id → stop_state_id 목록
        self._node_to_stops: dict = {}
        for sid in env.Stop_state:
            node = sid.split(',')[1]
            self._node_to_stops.setdefault(node, []).append(sid)

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────

    def _reset_intervals(self):
        """예약 정보를 초기화해 플래닝을 새로 시작할 수 있게 합니다."""
        for s in self._env.Stop_state.values():
            s.interval_list = [(0, float('inf'))]
        for m in self._env.Move_state.values():
            m.interval_list = [(0, float('inf'))]

    def _setup_agents(self, start_ids: List[str], goal_ids: List[str]):
        env = self._env
        env.AGVs = {}
        for i, (s_id, g_id) in enumerate(zip(start_ids, goal_ids)):
            v_id = i + 1
            agv  = self._AGV_cls(v_id)
            agv.cur_state      = env.Stop_state[s_id]
            agv.dst_state_list = [(g_id, 0)]
            env.AGVs[v_id]     = agv

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def plan(self,
             start_stop_ids: List[str],
             goal_stop_ids:  List[str],
             max_time: float = 30.0
             ) -> Optional[List[list]]:
        """
        지정한 start/goal stop_state ID 로 CBS 플래닝을 실행합니다.

        Parameters
        ----------
        start_stop_ids : ['S,16,0', 'S,42,270', ...]
        goal_stop_ids  : ['S,55,0', 'S,66,270', ...]
        max_time       : CBS 제한 시간(초)

        Returns
        -------
        paths : agent별 solver path 리스트, 또는 None(실패/타임아웃)
        """
        self._reset_intervals()
        self._setup_agents(start_stop_ids, goal_stop_ids)
        paths, _ = self._env.find_solution(max_time=max_time)
        return paths

    def plan_random(self,
                    n_agents: int,
                    max_time: float = 30.0
                    ) -> Optional[List[List[str]]]:
        """
        od_pairs에서 랜덤으로 n_agents 쌍을 골라 CBS 플래닝 후
        node_id 경로 리스트를 반환합니다.

        시작/목적지 stop_state가 에이전트 간에 겹치지 않도록 보장합니다.

        Returns
        -------
        list of node_id lists, 또는 None(실패)
        """
        usable = [(s, g) for s, g in self.od_pairs if str(s) != str(g)]
        if not usable:
            usable = self.od_pairs
        if not usable:
            return None

        # 시작/목적지 노드가 겹치지 않도록 노드 번호 기준으로 중복 체크
        random.shuffle(usable)
        used_start_nodes: set = set()
        used_goal_nodes:  set = set()
        starts, goals = [], []

        for src, dst in usable:
            if len(starts) >= n_agents:
                break
            src_node = str(src)
            dst_node = str(dst)
            if src_node in used_start_nodes or dst_node in used_goal_nodes:
                continue
            src_stops = self._node_to_stops.get(src_node, [])
            dst_stops = self._node_to_stops.get(dst_node, [])
            if not src_stops or not dst_stops:
                continue
            s_id = random.choice(src_stops)
            g_id = random.choice(dst_stops)
            starts.append(s_id)
            goals.append(g_id)
            used_start_nodes.add(src_node)
            used_goal_nodes.add(dst_node)

        if not starts:
            return None

        self._last_starts    = starts          # 실패 시에도 참조 가능하도록 미리 저장
        self._last_goals     = goals
        paths = self.plan(starts, goals, max_time=max_time)
        if paths is None:
            return None

        self._last_raw_paths = paths          # TAPG 등 후처리에서 접근 가능
        self._last_starts    = starts
        self._last_goals     = goals
        node_paths = [self.path_to_nodes(p) for p in paths]
        self._print_plan(starts, goals, paths, node_paths)
        return node_paths

    def plan_fixed(self,
                   start_stop_ids: List[str],
                   goal_stop_ids:  List[str],
                   max_time: float = 30.0
                   ):
        """
        저장된 시나리오(start/goal stop-state ID)를 그대로 재현합니다.

        Parameters
        ----------
        start_stop_ids : ['S,32,90', 'S,29,180', ...]
        goal_stop_ids  : ['S,28,180', 'S,26,270', ...]

        Returns
        -------
        node_paths 또는 None
        """
        paths = self.plan(start_stop_ids, goal_stop_ids, max_time=max_time)
        if paths is None:
            return None
        self._last_raw_paths = paths
        self._last_starts    = list(start_stop_ids)
        self._last_goals     = list(goal_stop_ids)
        node_paths = [self.path_to_nodes(p) for p in paths]
        self._print_plan(start_stop_ids, goal_stop_ids, paths, node_paths)
        return node_paths

    def _print_plan(self, starts, goals, raw_paths, node_paths):
        """CBS 플래닝 결과를 콘솔에 출력합니다."""
        sep = '-' * 60
        print(f'\n{sep}')
        print(f'CBS Plan  ({len(starts)} agents)')
        print(sep)
        for i, (s_id, g_id, path, nodes) in enumerate(
                zip(starts, goals, raw_paths, node_paths)):
            t_arrive = path[-1][1] if path else 0
            print(f'\nAgent {i+1}')
            print(f'  Origin      : {s_id}')
            print(f'  Destination : {g_id}')
            print(f'  Arrival time: {t_arrive:.1f}')
            print(f'  Node path   : {" -> ".join(nodes)}')
            # state 단위 경로 (타임스탬프 포함)
            state_parts = [f'{sid}@{t:.1f}' for sid, t in path]
            print(f'  State path  : {" -> ".join(state_parts)}')
        print(f'\n{sep}\n')

    @staticmethod
    def path_to_nodes(path: list) -> List[str]:
        """
        solver path [(state_id, cost), ...] → node_id 리스트.

        S state 의 node_id 만 추출하고 연속 중복을 제거합니다.
        예) [('S,16,0',0), ('M,16,31',1), ('S,31,0',2)] → ['16','31']
        """
        nodes: List[str] = []
        for state_id, _ in path:
            if state_id.startswith('S,'):
                node_id = state_id.split(',')[1]
                if not nodes or nodes[-1] != node_id:
                    nodes.append(node_id)
        return nodes
