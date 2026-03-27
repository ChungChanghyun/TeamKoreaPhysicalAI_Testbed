"""
gen_pkl.py — generate a collision-profile pkl from any JSON map file.

Usage (from ACS_general/ root):
    python mapf_edu/gen_pkl.py Maps/general_diamond.json Pickles/general_diamond.pkl
    python mapf_edu/gen_pkl.py Maps/general_diamond.json Pickles/general_diamond.pkl --no-rot
"""
import sys, os, io, argparse

# ── path setup ─────────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'map_gen'))
sys.path.insert(0, os.path.join(ROOT, 'utils'))

# Fix Korean console output on Windows (cp949 can't encode some Unicode chars)
if sys.stdout.encoding and sys.stdout.encoding.lower() in ('cp949', 'cp1252', 'ascii'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('json_path', help='path to JSON map file')
    ap.add_argument('pkl_path',  help='output pkl path')
    ap.add_argument('--no-rot',  action='store_true',
                    help='use no-rotation generator (default: rotation enabled)')
    args = ap.parse_args()

    json_path = os.path.join(ROOT, args.json_path) if not os.path.isabs(args.json_path) else args.json_path
    pkl_path  = os.path.join(ROOT, args.pkl_path)  if not os.path.isabs(args.pkl_path)  else args.pkl_path

    os.makedirs(os.path.dirname(pkl_path), exist_ok=True)

    if args.no_rot:
        import Generalized_251012_Affect_state_no_rot as gen
    else:
        import Generalized_251012_Affect_state as gen

    # The module uses layout_filename as a global for compute_critical_move_frequencies
    gen.layout_filename = json_path

    layout = gen.LayoutManager(json_path)
    gen.layout = layout          # set global used by CollisionProfile helpers
    layout.save_collision_profile(pkl_path)
    print(f"Saved: {pkl_path}")

if __name__ == '__main__':
    main()
