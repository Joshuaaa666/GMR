"""逐帧打印 GMR 动作 pkl 的关节角（DoF）与根位姿。

用法：
    # 按 pkl 的 fps 实时逐帧打印（默认）
    python scripts/print_pkl_dofs.py output/baduanjin_sr1_v2.pkl --robot sr1_v2

    # 不限速、只看左臂、每 30 帧打一次
    python scripts/print_pkl_dofs.py output/baduanjin_sr1_v2.pkl --robot sr1_v2 \
        --no_realtime --stride 30 --filter L_J

    # 同时用度数
    python scripts/print_pkl_dofs.py output/baduanjin_sr1_v2.pkl --robot sr1_v2 --deg
"""
import argparse
import pickle
import runpy
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def get_dof_names(robot: str):
    import mujoco as mj
    xml = runpy.run_path(str(ROOT / "general_motion_retargeting" / "params.py"))["ROBOT_XML_DICT"][robot]
    m = mj.MjModel.from_xml_path(str(xml))
    names = []
    for i in range(m.nv):
        jid = m.dof_jntid[i]
        names.append(mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, jid))
    # 去掉根 free-joint 的前 6 个 DoF
    return names[6:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pkl")
    ap.add_argument("--robot", default="sr1_v2")
    ap.add_argument("--deg", action="store_true", help="用度数打印（默认弧度）")
    ap.add_argument("--stride", type=int, default=1, help="每隔多少帧打印一次")
    ap.add_argument("--filter", default=None, help="只打印名字包含该串的 DoF，如 L_J / R_J")
    ap.add_argument("--no_realtime", action="store_true", help="不按 fps 限速，尽快打印")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=None)
    args = ap.parse_args()

    data = pickle.load(open(args.pkl, "rb"))
    dof = np.asarray(data["dof_pos"])
    root_pos = np.asarray(data["root_pos"])
    fps = float(data.get("fps", 30))
    N, ndof = dof.shape

    try:
        names = get_dof_names(args.robot)
    except Exception as e:
        print(f"(读不到 DoF 名，用序号) {e}")
        names = [f"dof{i}" for i in range(ndof)]
    if len(names) != ndof:
        names = [f"dof{i}" for i in range(ndof)]

    sel = [i for i, n in enumerate(names) if (args.filter is None or args.filter in n)]
    unit = "deg" if args.deg else "rad"
    conv = (lambda x: np.rad2deg(x)) if args.deg else (lambda x: x)

    print(f"file={args.pkl}  fps={fps}  frames={N}  ndof={ndof}  unit={unit}")
    print("DoF order:", ", ".join(names))
    print("-" * 80)

    end = args.end if args.end is not None else N
    dt = 1.0 / fps if not args.no_realtime else 0.0
    for f in range(args.start, end, args.stride):
        t0 = time.time()
        vals = "  ".join(f"{names[i]}={conv(dof[f, i]): 7.3f}" for i in sel)
        rp = root_pos[f]
        print(f"[{f:5d} t={f/fps:6.2f}s] root=({rp[0]:+.3f},{rp[1]:+.3f},{rp[2]:+.3f}) | {vals}")
        if dt:
            slp = dt * args.stride - (time.time() - t0)
            if slp > 0:
                time.sleep(slp)


if __name__ == "__main__":
    main()
