import argparse
import pickle
import runpy
import sys
from pathlib import Path

import mujoco as mj
import numpy as np


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PARAMS_PATH = ROOT / "general_motion_retargeting" / "params.py"
ROBOT_XML_DICT = runpy.run_path(str(PARAMS_PATH))["ROBOT_XML_DICT"]

# wheel_offset (MBASE→wheel center) + wheel_radius, per robot
WHEEL_GROUND_Z = {
    # wheel_center_z_from_MBASE + wheel_radius
    # SR1: MBASE -> HIPR(+0.078) -> WHEEL(-0.087) => center offset=-0.009; radius=0.055
    "sr1_v1":        0.064,
    "sr1_v1_pro":    0.064,
    "sr1_v1_promax": 0.064,
    "galaxea_r1pro": 0.118,
}


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """Centered moving average with edge padding."""
    if window <= 1:
        return x.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    x_pad = np.pad(x, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=np.float64) / float(window)
    return np.convolve(x_pad, kernel, mode="valid")


def get_dof_names(model: mj.MjModel) -> list[str]:
    names = []
    for i in range(model.nv):
        joint_id = model.dof_jntid[i]
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint_id)
        names.append(name)
    return names


def qpos0_to_dof_defaults(model: mj.MjModel, dof_names: list[str]) -> np.ndarray:
    out = np.zeros(len(dof_names), dtype=np.float64)
    for i, _ in enumerate(dof_names):
        joint_id = model.dof_jntid[i + 6]  # Skip root free-joint DoFs.
        qpos_addr = model.jnt_qposadr[joint_id]
        out[i] = model.qpos0[qpos_addr]
    return out


def quat_xyzw_to_euler_xyz(quat: np.ndarray) -> np.ndarray:
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


def euler_xyz_to_quat_xyzw(euler: np.ndarray) -> np.ndarray:
    roll = euler[:, 0]
    pitch = euler[:, 1]
    yaw = euler[:, 2]

    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    quat = np.stack([x, y, z, w], axis=-1)
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    return quat / np.clip(norm, 1e-8, None)


def select_joint_indices(
    dof_names: list[str],
    freeze_waist: bool,
    freeze_arms: bool,
    freeze_lower: bool,
) -> list[int]:
    prefixes = ["Jhd"]
    if freeze_waist:
        prefixes.append("Jwb")
    if freeze_arms:
        prefixes.extend(["Jal", "Jar"])

    lower_names = {
        "Jfl1_hipr",
        "Jfl2_wheel",
        "Jfr1_hipr",
        "Jfr2_wheel",
        "Jrl1_hipr",
        "Jrl2_wheel",
        "Jrr1_hipr",
        "Jrr2_wheel",
    }

    indices = []
    for i, name in enumerate(dof_names):
        if any(name.startswith(prefix) for prefix in prefixes):
            indices.append(i)
        elif freeze_lower and name in lower_names:
            indices.append(i)
    return indices


def print_deg_stats(label: str, radians: np.ndarray) -> None:
    deg = np.rad2deg(radians)
    print(
        f"{label}: mean={deg.mean(): .3f}deg, std={deg.std(): .3f}deg, "
        f"min={deg.min(): .3f}deg, max={deg.max(): .3f}deg"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Lock torso/head posture in a GMR motion .pkl. "
            "Also supports grounding wheeled-base robots via --wheel_ground."
        )
    )
    parser.add_argument("--input", required=True, help="Input motion .pkl.")
    parser.add_argument("--output", required=True, help="Output motion .pkl.")
    parser.add_argument(
        "--robot",
        default="sr1_v1_pro",
        help="Robot name for resolving DoF order, e.g. sr1_v1_pro.",
    )
    parser.add_argument(
        "--root_roll_deg",
        type=float,
        default=0.0,
        help="Fixed root roll in degrees. Default: 0.",
    )
    parser.add_argument(
        "--root_pitch_deg",
        type=float,
        default=0.0,
        help="Fixed root pitch in degrees. Default: 0, upright.",
    )
    parser.add_argument(
        "--root_yaw_deg",
        type=float,
        default=None,
        help="Fixed root yaw in degrees. If not set, original yaw is preserved.",
    )
    # --- z grounding ---
    parser.add_argument(
        "--fix_root_z",
        action="store_true",
        help="Fix root_pos[:, 2] to its median height (generic fallback).",
    )
    parser.add_argument(
        "--root_z",
        type=float,
        default=None,
        help=(
            "Exact z (meters) to lock the root to. "
            "Overrides --fix_root_z median. "
            "Use with --z_osc_scale to also flatten vertical bounce."
        ),
    )
    parser.add_argument(
        "--wheel_ground",
        action="store_true",
        help=(
            "Automatically set root z so wheels are flush with the ground. "
            "Uses per-robot wheel geometry from WHEEL_GROUND_Z table. "
            "Implies --z_osc_scale 0 and --attn_roll/pitch 0 unless overridden."
        ),
    )
    parser.add_argument(
        "--z_osc_scale",
        type=float,
        default=None,
        help=(
            "Keep this fraction of vertical oscillation around the z trend "
            "(0 = fully flat, 1 = unchanged). Applied when --root_z or "
            "--wheel_ground is used. Default: 0 for wheeled robots, 1 otherwise."
        ),
    )
    parser.add_argument(
        "--z_window",
        type=int,
        default=21,
        help="Moving-average window for z trend extraction (odd preferred). Default: 21.",
    )
    # --- roll / pitch attenuation ---
    parser.add_argument(
        "--attn_roll",
        type=float,
        default=None,
        help=(
            "Roll attenuation factor (0 = zero out, 1 = unchanged). "
            "Default: 0 when --wheel_ground is set, else no change."
        ),
    )
    parser.add_argument(
        "--attn_pitch",
        type=float,
        default=None,
        help=(
            "Pitch attenuation factor (0 = zero out, 1 = unchanged). "
            "Default: 0 when --wheel_ground is set, else no change."
        ),
    )
    parser.add_argument(
        "--euler_window",
        type=int,
        default=9,
        help="Moving-average window for roll/pitch smoothing. Default: 9.",
    )
    # --- xy / joints ---
    parser.add_argument(
        "--fix_root_xy",
        action="store_true",
        help="Also fix root_pos[:, :2] to the initial frame's x,y position.",
    )
    parser.add_argument(
        "--freeze_waist",
        action="store_true",
        help="Freeze waist joints (Jwb*). Default: waist motion is preserved.",
    )
    parser.add_argument(
        "--freeze_arms",
        action="store_true",
        help="Also freeze left/right arm joints. By default arm motion is preserved.",
    )
    parser.add_argument(
        "--freeze_lower",
        action="store_true",
        help="Also freeze wheel/lower-body joints to model defaults.",
    )
    parser.add_argument(
        "--smooth_dof_window",
        type=int,
        default=1,
        help=(
            "Moving-average window applied to every joint angle over time. "
            "1 = no smoothing (default). 5~9 removes IK jitter for slow motions like tai chi."
        ),
    )
    args = parser.parse_args()

    # Resolve defaults for wheeled-ground mode.
    if args.wheel_ground:
        if args.robot not in WHEEL_GROUND_Z:
            raise ValueError(
                f"--wheel_ground not supported for robot '{args.robot}'. "
                f"Supported: {list(WHEEL_GROUND_Z)}"
            )
        if args.root_z is None:
            args.root_z = WHEEL_GROUND_Z[args.robot]
        if args.z_osc_scale is None:
            args.z_osc_scale = 0.0
        if args.attn_roll is None:
            args.attn_roll = 0.0
        if args.attn_pitch is None:
            args.attn_pitch = 0.0

    in_path = Path(args.input)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT[args.robot]))
    dof_names = get_dof_names(model)[6:]
    default_values = qpos0_to_dof_defaults(model, dof_names)

    with open(in_path, "rb") as f:
        motion_data = pickle.load(f)

    required = {"fps", "root_pos", "root_rot", "dof_pos"}
    missing = required - set(motion_data.keys())
    if missing:
        raise KeyError(f"Input motion missing keys: {sorted(missing)}")

    root_pos = np.asarray(motion_data["root_pos"], dtype=np.float64).copy()
    root_rot = np.asarray(motion_data["root_rot"], dtype=np.float64).copy()
    dof_pos = np.asarray(motion_data["dof_pos"]).copy()

    if dof_pos.ndim != 2 or dof_pos.shape[1] != len(dof_names):
        raise ValueError(
            f"dof_pos shape {dof_pos.shape} does not match {len(dof_names)} DoFs for {args.robot}."
        )

    # ---- z processing ----
    z_report = None
    if args.root_z is not None:
        osc_scale = args.z_osc_scale if args.z_osc_scale is not None else 0.0
        if osc_scale == 0.0:
            # Fully constant — wheels never leave the ground.
            root_pos[:, 2] = args.root_z
        else:
            z = root_pos[:, 2]
            z_trend = moving_average(z, args.z_window)
            z_osc = z - z_trend
            z_shift = args.root_z - z_trend.mean()
            root_pos[:, 2] = z_trend + z_shift + osc_scale * z_osc
        z_report = f"root z locked to {args.root_z:.4f}m (osc_scale={osc_scale})"
    elif args.fix_root_z:
        z_fixed = float(np.median(root_pos[:, 2]))
        root_pos[:, 2] = z_fixed
        z_report = f"root z fixed to median {z_fixed:.6f}m"

    if args.fix_root_xy:
        xy_fixed = root_pos[0, :2].copy()
        root_pos[:, :2] = xy_fixed[None, :]

    # ---- root orientation ----
    euler_before = quat_xyzw_to_euler_xyz(root_rot)
    euler_after = euler_before.copy()

    # Apply roll/pitch attenuation if requested (smooth first, then scale).
    if args.attn_roll is not None or args.attn_pitch is not None:
        roll_s = moving_average(euler_before[:, 0], args.euler_window)
        pitch_s = moving_average(euler_before[:, 1], args.euler_window)
        yaw_s = moving_average(np.unwrap(euler_before[:, 2]), args.euler_window)
        yaw_s = (yaw_s + np.pi) % (2 * np.pi) - np.pi

        if args.attn_roll is not None:
            euler_after[:, 0] = args.attn_roll * roll_s
        if args.attn_pitch is not None:
            euler_after[:, 1] = args.attn_pitch * pitch_s
        euler_after[:, 2] = yaw_s

    # Override with fixed angles if explicitly set.
    euler_after[:, 0] = np.deg2rad(args.root_roll_deg)
    euler_after[:, 1] = np.deg2rad(args.root_pitch_deg)
    if args.root_yaw_deg is not None:
        euler_after[:, 2] = np.deg2rad(args.root_yaw_deg)

    root_rot_out = euler_xyz_to_quat_xyzw(euler_after).astype(np.float32)

    # ---- freeze joints ----
    frozen_indices = select_joint_indices(
        dof_names=dof_names,
        freeze_waist=args.freeze_waist,
        freeze_arms=args.freeze_arms,
        freeze_lower=args.freeze_lower,
    )
    dof_before = dof_pos.copy()
    dof_pos[:, frozen_indices] = default_values[frozen_indices][None, :]

    # Smooth all non-frozen joints over time to remove IK frame-to-frame jitter.
    if args.smooth_dof_window > 1:
        all_indices = [i for i in range(dof_pos.shape[1]) if i not in frozen_indices]
        for i in all_indices:
            dof_pos[:, i] = moving_average(dof_pos[:, i].astype(np.float64), args.smooth_dof_window)

    out_data = dict(motion_data)
    out_data["root_pos"] = root_pos.astype(np.float32)
    out_data["root_rot"] = root_rot_out
    out_data["dof_pos"] = dof_pos

    with open(out_path, "wb") as f:
        pickle.dump(out_data, f)

    # ---- report ----
    euler_check = quat_xyzw_to_euler_xyz(root_rot_out.astype(np.float64))
    print("Root orientation:")
    print_deg_stats("  roll before ", euler_before[:, 0])
    print_deg_stats("  roll after  ", euler_check[:, 0])
    print_deg_stats("  pitch before", euler_before[:, 1])
    print_deg_stats("  pitch after ", euler_check[:, 1])
    if args.root_yaw_deg is not None:
        print_deg_stats("  yaw before  ", euler_before[:, 2])
        print_deg_stats("  yaw after   ", euler_check[:, 2])

    if z_report:
        print(f"\nRoot z: {z_report}")
        z_out = root_pos[:, 2]
        print(f"  z after: min={z_out.min():.4f}, max={z_out.max():.4f}, mean={z_out.mean():.4f}")

    if args.fix_root_xy:
        print(f"Fixed root xy to first frame: x={xy_fixed[0]:.6f}, y={xy_fixed[1]:.6f}")

    if args.smooth_dof_window > 1:
        diff_before = np.degrees(np.abs(np.diff(dof_before, axis=0)))
        diff_after  = np.degrees(np.abs(np.diff(dof_pos,   axis=0)))
        print(f"\nJoint smoothing (window={args.smooth_dof_window}):")
        print(f"  max  deg/frame: {diff_before.max():.2f} -> {diff_after.max():.2f}")
        print(f"  p99  deg/frame: {np.percentile(diff_before,99):.2f} -> {np.percentile(diff_after,99):.2f}")
        print(f"  mean deg/frame: {diff_before.mean():.3f} -> {diff_after.mean():.3f}")

    if frozen_indices:
        print("\nFrozen joints:")
        for i in frozen_indices:
            before = dof_before[:, i]
            after = dof_pos[:, i]
            print(
                f"  {i:02d} {dof_names[i]:28s} "
                f"{before.mean(): .5f}->{after.mean(): .5f}, "
                f"std {before.std(): .5f}->{after.std(): .5f}"
            )

    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
