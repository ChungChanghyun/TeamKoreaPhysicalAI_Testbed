"""
gen_3ds_pkl.py — KaistTB 3DS 각 층별 collision profile pkl 생성.

1) KaistTB_map.json에서 3DS_F1/F2/F3 노드/세그먼트 추출 → 층별 JSON
2) LayoutManager로 collision profile pkl 생성 (no rotation)

Usage:
    cd mapf_edu
    python gen_3ds_pkl.py
"""
import sys, os, json, io

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
MAP_GEN_DIR = os.path.join(SCRIPT_DIR, '..', 'map_gen')
UTILS_DIR   = os.path.join(SCRIPT_DIR, '..', 'utils')
MAPS_DIR    = os.path.join(SCRIPT_DIR, '..', 'Maps')

KAIST_JSON  = os.path.join(MAPS_DIR, 'KaistTB.map_latest.json')

FLOOR_IDS = ['3DS_F1', '3DS_F2', '3DS_F3']

# Fix Korean console output on Windows
if sys.stdout.encoding and sys.stdout.encoding.lower() in ('cp949', 'cp1252', 'ascii'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')


def extract_floor_submap(kaist_data: dict, floor_id: str) -> str:
    """KaistTB 맵에서 특정 3DS 층만 추출하여 JSON 저장."""
    d = kaist_data

    # 해당 층 노드
    floor_ids = {n['id'] for n in d['nodes'] if n.get('area') == floor_id}
    nodes = [n for n in d['nodes'] if n['id'] in floor_ids]

    # 세그먼트
    segments = []
    for s in d['segments']:
        fn, tn = s['startNodeId'], s['endNodeId']
        if fn in floor_ids and tn in floor_ids:
            segments.append(s)

    # 포트: degree==1 노드
    adj = {}
    for s in segments:
        adj.setdefault(s['startNodeId'], []).append(s['endNodeId'])
        adj.setdefault(s['endNodeId'], [])
    ports = []
    for nid in floor_ids:
        if len(adj.get(nid, [])) <= 1:
            ports.append({'nodeId': nid})

    # 3DS 차량 모델
    vehicle_models = []
    for vm in d.get('vehicleModels', []):
        if vm['id'] == '3DS':
            vehicle_models.append(vm)
            break
    if not vehicle_models:
        vehicle_models = [{
            'id': '3DS',
            'dimension': {'width': 850, 'length': 850, 'height': 210}
        }]

    sub_map = {
        'schema': d.get('schema', 'xmsmap-v4.0'),
        'info': {
            'name': f'KaistTB_{floor_id}',
            'version': d.get('info', {}).get('version', '1.0'),
        },
        'nodes': nodes,
        'segments': segments,
        'ports': ports,
        'vehicleModels': vehicle_models,
    }

    json_path = os.path.join(SCRIPT_DIR, f'KaistTB_{floor_id}.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(sub_map, f, indent=2, ensure_ascii=False)

    print(f'{floor_id}:')
    print(f'  Nodes:    {len(nodes)}')
    print(f'  Segments: {len(segments)}')
    print(f'  Ports:    {len(ports)}')
    print(f'  Vehicle:  {vehicle_models[0]["dimension"]}')
    print(f'  JSON:     {json_path}')
    return json_path


def generate_pkl(json_path: str, pkl_path: str):
    """LayoutManager로 collision profile pkl 생성 (no rotation)."""
    sys.path.insert(0, MAP_GEN_DIR)
    sys.path.insert(0, UTILS_DIR)

    import Generalized_251012_Affect_state_no_rot as gen_mod
    gen_mod.layout_filename = json_path

    layout = gen_mod.LayoutManager(json_path)

    print(f'  Stop states:  {len(layout.Stop_state)}')
    print(f'  Move states:  {len(layout.Move_state)}')
    print(f'  Stop regions: {len(layout.stop_regions)}')
    print(f'  Move regions: {len(layout.move_regions)}')

    layout.save_collision_profile(pkl_path)
    print(f'  PKL:     {pkl_path}')
    return pkl_path


def verify_pkl(pkl_path: str):
    """pkl_loader로 생성된 pkl 검증."""
    sys.path.insert(0, SCRIPT_DIR)
    from pkl_loader import PklMapGraph

    graph = PklMapGraph(pkl_path)
    print(f'  Verify:  {len(graph.nodes)}n {len(graph.edges)}e '
          f'{len(graph.stop_states_raw)}S {len(graph.move_states_raw)}M '
          f'vehicle={graph.vehicle_length:.0f}x{graph.vehicle_width:.0f}mm '
          f'ports={len(graph.ports)}')

    total_aff = sum(len(s.affect_state) for s in graph.stop_states_raw.values())
    total_aff += sum(len(s.affect_state) for s in graph.move_states_raw.values())
    print(f'  Affects: {total_aff} total links')
    return graph


if __name__ == '__main__':
    print('Loading KaistTB map...')
    with open(KAIST_JSON, 'r', encoding='utf-8') as f:
        kaist_data = json.load(f)

    for fid in FLOOR_IDS:
        print(f'\n{"="*60}')
        json_path = extract_floor_submap(kaist_data, fid)
        pkl_path = os.path.join(SCRIPT_DIR, f'KaistTB_{fid}.pkl')
        generate_pkl(json_path, pkl_path)
        verify_pkl(pkl_path)

    print(f'\n{"="*60}')
    print('All floors done!')
