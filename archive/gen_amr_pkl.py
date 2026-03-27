"""
gen_amr_pkl.py — KaistTB AMR_A 서브네트워크에서 collision profile pkl 생성.

1) KaistTB.map_latest.json에서 AMR_A 노드/세그먼트만 추출 → AMR_A 전용 JSON
2) Generalized_251012_Affect_state_no_rot.py의 LayoutManager로 pkl 생성

Usage:
    cd mapf_edu
    python gen_amr_pkl.py
"""
import sys, os, json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_GEN_DIR = os.path.join(SCRIPT_DIR, '..', 'map_gen')
MAPS_DIR    = os.path.join(SCRIPT_DIR, '..', 'Maps')

KAIST_JSON  = os.path.join(MAPS_DIR, 'KaistTB.map_latest.json')
AMR_JSON    = os.path.join(SCRIPT_DIR, 'KaistTB_AMR_A.json')
OUTPUT_PKL  = os.path.join(SCRIPT_DIR, 'KaistTB_AMR_A.pkl')


def extract_amr_submap():
    """KaistTB 맵에서 AMR_A 서브네트워크만 추출하여 JSON 저장."""
    with open(KAIST_JSON, 'r', encoding='utf-8') as f:
        d = json.load(f)

    # AMR_A 노드 + area 없는 노드 중 AMR_A와 연결된 것
    amr_ids = {n['id'] for n in d['nodes'] if n.get('area') == 'AMR_A'}
    no_area = {n['id'] for n in d['nodes'] if not n.get('area')}
    connected = set()
    for s in d['segments']:
        fn, tn = s['startNodeId'], s['endNodeId']
        if fn in no_area and tn in amr_ids:
            connected.add(fn)
        if tn in no_area and fn in amr_ids:
            connected.add(tn)
    area_ids = amr_ids | connected

    # 노드 필터링
    nodes = [n for n in d['nodes'] if n['id'] in area_ids]

    # 세그먼트 필터링
    segments = []
    for s in d['segments']:
        fn, tn = s['startNodeId'], s['endNodeId']
        if fn in area_ids and tn in area_ids:
            segments.append(s)

    # 포트 필터링
    ports = [p for p in d.get('ports', []) if p['nodeId'] in area_ids]

    # AMR_CV1 차량 모델 사용
    vehicle_models = []
    for vm in d.get('vehicleModels', []):
        if vm['id'] == 'AMR_CV1':
            vehicle_models.append(vm)
            break
    if not vehicle_models:
        # fallback: 기본값
        vehicle_models = [{
            'id': 'AMR_CV1',
            'dimension': {'width': 700, 'length': 780, 'height': 650}
        }]

    sub_map = {
        'schema': d.get('schema', 'xmsmap-v4.0'),
        'info': {
            'name': 'KaistTB_AMR_A',
            'version': d.get('info', {}).get('version', '1.0'),
        },
        'nodes': nodes,
        'segments': segments,
        'ports': ports,
        'vehicleModels': vehicle_models,
    }

    with open(AMR_JSON, 'w', encoding='utf-8') as f:
        json.dump(sub_map, f, indent=2, ensure_ascii=False)

    print(f'Extracted AMR_A sub-map:')
    print(f'  Nodes:    {len(nodes)}')
    print(f'  Segments: {len(segments)}')
    print(f'  Ports:    {len(ports)}')
    print(f'  Vehicle:  {vehicle_models[0]["dimension"]}')
    print(f'  Saved to: {AMR_JSON}')
    return AMR_JSON


def generate_pkl(json_path: str, pkl_path: str):
    """LayoutManager로 collision profile pkl 생성."""
    # map_gen을 import하기 위해 경로 추가
    sys.path.insert(0, MAP_GEN_DIR)
    sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'utils'))

    # LayoutManager가 글로벌 layout_filename을 참조하므로 설정
    import Generalized_251012_Affect_state as gen_mod
    gen_mod.layout_filename = json_path

    print(f'\nGenerating collision profile...')
    print(f'  Input:  {json_path}')
    print(f'  Output: {pkl_path}')

    layout = gen_mod.LayoutManager(json_path)

    print(f'  Stop states:  {len(layout.Stop_state)}')
    print(f'  Move states:  {len(layout.Move_state)}')
    print(f'  Stop regions: {len(layout.stop_regions)}')
    print(f'  Move regions: {len(layout.move_regions)}')

    layout.save_collision_profile(pkl_path)
    print(f'  Saved: {pkl_path}')
    return pkl_path


def verify_pkl(pkl_path: str):
    """pkl_loader로 생성된 pkl 검증."""
    sys.path.insert(0, SCRIPT_DIR)
    from pkl_loader import PklMapGraph

    print(f'\nVerifying pkl...')
    graph = PklMapGraph(pkl_path)
    print(f'  Nodes:       {len(graph.nodes)}')
    print(f'  Edges:       {len(graph.edges)}')
    print(f'  Stop states: {len(graph.stop_states_raw)}')
    print(f'  Move states: {len(graph.move_states_raw)}')
    print(f'  Vehicle:     {graph.vehicle_length:.0f}mm × {graph.vehicle_width:.0f}mm')
    print(f'  Ports:       {len(graph.ports)}')
    print(f'  BBox:        {graph.bbox}')

    # affect_state 검증
    total_aff = sum(len(s.affect_state) for s in graph.stop_states_raw.values())
    total_aff += sum(len(s.affect_state) for s in graph.move_states_raw.values())
    print(f'  Total affect_state links: {total_aff}')

    # 몇 개 샘플 출력
    for sid, s in list(graph.stop_states_raw.items())[:2]:
        print(f'    {sid}: {len(s.affect_state)} conflicts')
    for sid, s in list(graph.move_states_raw.items())[:2]:
        print(f'    {sid}: {len(s.affect_state)} conflicts')

    return graph


if __name__ == '__main__':
    json_path = extract_amr_submap()
    pkl_path  = generate_pkl(json_path, OUTPUT_PKL)
    verify_pkl(pkl_path)
    print('\nDone!')
