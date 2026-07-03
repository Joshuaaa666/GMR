import argparse
import pickle
import runpy
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def quat_xyzw_to_euler_xyz(quat: np.ndarray) -> np.ndarray:
    """Convert xyzw quaternions to xyz Euler angles in radians."""
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(t2, -1.0, 1.0))

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return np.stack([roll, pitch, yaw], axis=-1)


def print_array_stats(name: str, arr: np.ndarray) -> None:
    arr = np.asarray(arr)
    print(f"{name}: shape={arr.shape}, dtype={arr.dtype}")
    if arr.size == 0:
        return
    if not np.issubdtype(arr.dtype, np.number):
        print(f"  non-numeric value type: {type(arr.item()).__name__ if arr.shape == () else 'object'}")
        return
    flat = arr.reshape(-1)
    print(
        f"  min={np.nanmin(flat): .6f}, max={np.nanmax(flat): .6f}, "
        f"mean={np.nanmean(flat): .6f}, std={np.nanstd(flat): .6f}"
    )


def get_dof_names(robot: str) -> list[str] | None:
    try:
        import mujoco as mj
    except ImportError:
        print("Warning: mujoco is not installed, so DoF names cannot be shown.")
        return None

    params_path = ROOT / "general_motion_retargeting" / "params.py"
    robot_xml_dict = runpy.run_path(str(params_path))["ROBOT_XML_DICT"]
    if robot not in robot_xml_dict:
        choices = ", ".join(sorted(robot_xml_dict.keys()))
        raise KeyError(f"Unknown robot '{robot}'. Choices: {choices}")

    model = mj.MjModel.from_xml_path(str(robot_xml_dict[robot]))
    names = []
    for i in range(model.nv):
        joint_id = model.dof_jntid[i]
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id)
        names.append(name)
    return names[6:]


def print_euler_stats(root_rot: np.ndarray) -> None:
    root_rot = np.asarray(root_rot)
    if root_rot.ndim != 2 or root_rot.shape[1] != 4:
        print("root_rot is not shaped as (T, 4); skipping Euler stats.")
        return

    euler_deg = np.rad2deg(quat_xyzw_to_euler_xyz(root_rot.astype(np.float64)))
    print("\nRoot Euler XYZ degrees:")
    for i, name in enumerate(("roll", "pitch", "yaw")):
        values = euler_deg[:, i]
        print(
            f"  {name:5s}: "
            f"mean={values.mean(): .3f}, median={np.median(values): .3f}, "
            f"std={values.std(): .3f}, min={values.min(): .3f}, max={values.max(): .3f}, "
            f"p5={np.percentile(values, 5): .3f}, p95={np.percentile(values, 95): .3f}"
        )


def print_dof_stats(dof_pos: np.ndarray, dof_names: list[str] | None, limit: int | None) -> None:
    dof_pos = np.asarray(dof_pos)
    if dof_pos.ndim != 2:
        print("dof_pos is not shaped as (T, D); skipping per-DoF stats.")
        return

    count = dof_pos.shape[1] if limit is None else min(limit, dof_pos.shape[1])
    print("\nDoF stats:")
    for i in range(count):
        name = dof_names[i] if dof_names and i < len(dof_names) else f"dof_{i}"
        values = dof_pos[:, i]
        print(
            f"  {i:02d} {name:28s} "
            f"mean={values.mean(): .5f}, std={values.std(): .5f}, "
            f"min={values.min(): .5f}, max={values.max(): .5f}"
        )
    if count < dof_pos.shape[1]:
        print(f"  ... skipped {dof_pos.shape[1] - count} DoFs; use --all_dofs to show all.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a GMR robot motion .pkl file.")
    parser.add_argument("pkl", help="Path to motion .pkl file.")
    parser.add_argument(
        "--robot",
        default=None,
        help="Optional robot name, e.g. sr1_v1_pro, for showing DoF names.",
    )
    parser.add_argument(
        "--all_dofs",
        action="store_true",
        help="Print statistics for every DoF instead of the first 12.",
    )
    args = parser.parse_args()

    pkl_path = Path(args.pkl)
    with open(pkl_path, "rb") as f:
        data = pickle.load(f)

    print("=" * 72)
    print(f"File: {pkl_path}")
    print(f"Keys: {list(data.keys())}")
    print("=" * 72)

    if "fps" in data:
        print(f"FPS: {data['fps']}")

    for key in ("root_pos", "root_rot", "dof_pos", "local_body_pos"):
        if key in data:
            print_array_stats(key, np.asarray(data[key]))

    if "root_pos" in data:
        root_pos = np.asarray(data["root_pos"])
        if root_pos.ndim == 2 and root_pos.shape[1] >= 3:
            z = root_pos[:, 2]
            print(
                f"\nRoot z: min={z.min(): .6f}, max={z.max(): .6f}, "
                f"mean={z.mean(): .6f}, std={z.std(): .6f}"
            )
            print(f"First root_pos: {root_pos[0]}")
            print(f"Mean  root_pos: {root_pos.mean(axis=0)}")

    if "root_rot" in data:
        print_euler_stats(np.asarray(data["root_rot"]))
        print(f"First root_rot xyzw: {np.asarray(data['root_rot'])[0]}")

    dof_names = get_dof_names(args.robot) if args.robot else None
    if "dof_pos" in data:
        limit = None if args.all_dofs else 12
        print_dof_stats(np.asarray(data["dof_pos"]), dof_names, limit)

    if "link_body_list" in data:
        links = data["link_body_list"]
        if links is None:
            print("\nlink_body_list: None")
        else:
            print(f"\nlink_body_list: len={len(links)}")
            print(f"First links: {links[:10]}")


if __name__ == "__main__":
    main()
