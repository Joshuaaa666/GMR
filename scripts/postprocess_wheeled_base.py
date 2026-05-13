import argparse
import pickle
from pathlib import Path

import numpy as np


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


def wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def quat_xyzw_to_euler_xyz(quat: np.ndarray) -> np.ndarray:
    """Convert quaternions (xyzw) to Euler xyz (radians)."""
    x = quat[:, 0]
    y = quat[:, 1]
    z = quat[:, 2]
    w = quat[:, 3]

    # roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    # yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return np.stack([roll, pitch, yaw], axis=-1)


def euler_xyz_to_quat_xyzw(euler: np.ndarray) -> np.ndarray:
    """Convert Euler xyz (radians) to quaternions (xyzw)."""
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
    # Normalize for numerical stability.
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    quat = quat / np.clip(norm, 1e-8, None)
    return quat


def process_motion(
    motion_data: dict,
    z_window: int,
    z_osc_scale: float,
    z_lift: float,
    attn_roll: float,
    attn_pitch: float,
    pitch_bias_deg: float,
    euler_window: int,
) -> dict:
    root_pos = np.asarray(motion_data["root_pos"], dtype=np.float64).copy()
    root_rot_xyzw = np.asarray(motion_data["root_rot"], dtype=np.float64).copy()

    # 1) Remove vertical bouncing while keeping slow trend.
    z = root_pos[:, 2]
    z_trend = moving_average(z, z_window)
    z_osc = z - z_trend
    root_pos[:, 2] = z_trend + z_osc_scale * z_osc + z_lift

    # 2) Attenuate base roll/pitch, keep yaw for navigation intent.
    euler = quat_xyzw_to_euler_xyz(root_rot_xyzw)
    roll = euler[:, 0]
    pitch = euler[:, 1]
    yaw = euler[:, 2]

    # Smooth before attenuation to suppress high-frequency wobble.
    roll_s = moving_average(roll, euler_window)
    pitch_s = moving_average(pitch, euler_window)
    yaw_s = moving_average(np.unwrap(yaw), euler_window)
    yaw_s = wrap_to_pi(yaw_s)

    roll_new = attn_roll * roll_s
    pitch_new = attn_pitch * pitch_s + np.deg2rad(pitch_bias_deg)
    yaw_new = yaw_s

    root_rot_new_xyzw = euler_xyz_to_quat_xyzw(
        np.stack([roll_new, pitch_new, yaw_new], axis=-1)
    )

    out = dict(motion_data)
    out["root_pos"] = root_pos.astype(np.float32)
    out["root_rot"] = root_rot_new_xyzw.astype(np.float32)
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Post-process GMR motion for wheeled base stability."
    )
    parser.add_argument("--input", required=True, help="Input motion .pkl path.")
    parser.add_argument("--output", required=True, help="Output motion .pkl path.")
    parser.add_argument(
        "--z_window",
        type=int,
        default=21,
        help="Window for z-trend moving average (odd preferred).",
    )
    parser.add_argument(
        "--z_osc_scale",
        type=float,
        default=0.10,
        help="Keep ratio of vertical oscillation; 0 = fully flattened, 1 = unchanged.",
    )
    parser.add_argument(
        "--z_lift",
        type=float,
        default=0.0,
        help="Add constant upward offset to root z (meters) to avoid wheel ground penetration.",
    )
    parser.add_argument(
        "--attn_roll",
        type=float,
        default=0.20,
        help="Roll attenuation factor; 0 = fully zeroed, 1 = unchanged.",
    )
    parser.add_argument(
        "--attn_pitch",
        type=float,
        default=0.20,
        help="Pitch attenuation factor; 0 = fully zeroed, 1 = unchanged.",
    )
    parser.add_argument(
        "--pitch_bias_deg",
        type=float,
        default=0.0,
        help="Constant pitch bias in degrees (+ forward if your convention matches).",
    )
    parser.add_argument(
        "--euler_window",
        type=int,
        default=9,
        help="Window for roll/pitch/yaw smoothing.",
    )
    args = parser.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(in_path, "rb") as f:
        motion_data = pickle.load(f)

    required_keys = {"fps", "root_pos", "root_rot", "dof_pos"}
    missing = required_keys - set(motion_data.keys())
    if missing:
        raise KeyError(f"Input motion file missing keys: {sorted(missing)}")

    processed = process_motion(
        motion_data=motion_data,
        z_window=args.z_window,
        z_osc_scale=args.z_osc_scale,
        z_lift=args.z_lift,
        attn_roll=args.attn_roll,
        attn_pitch=args.attn_pitch,
        pitch_bias_deg=args.pitch_bias_deg,
        euler_window=args.euler_window,
    )

    with open(out_path, "wb") as f:
        pickle.dump(processed, f)

    print(f"Saved post-processed motion to: {out_path}")


if __name__ == "__main__":
    main()
