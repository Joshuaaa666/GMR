import argparse
import json
import pickle
from pathlib import Path
import sys
import runpy

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PARAMS_PATH = ROOT / "general_motion_retargeting" / "params.py"
ROBOT_XML_DICT = runpy.run_path(str(PARAMS_PATH))["ROBOT_XML_DICT"]


def get_dof_names(model: mj.MjModel):
    names = []
    for i in range(model.nv):
        jnt_id = model.dof_jntid[i]
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, jnt_id)
        names.append(name)
    return names


def build_default_map(dof_names, model_qpos0_dof, default_scalar, default_json_path=None):
    # Start from model qpos0 defaults (more physically consistent than hard-coded zeros).
    default_map = {name: float(model_qpos0_dof[i]) for i, name in enumerate(dof_names)}

    # Optional global override.
    if default_scalar is not None:
        default_map = {name: float(default_scalar) for name in dof_names}

    if default_json_path is None:
        return default_map

    with open(default_json_path, "r", encoding="utf-8") as f:
        user_map = json.load(f)

    for k, v in user_map.items():
        if k in default_map:
            default_map[k] = float(v)
    return default_map


def select_lower_body_indices(dof_names):
    # SR1 wheeled base lower-body DoFs by name pattern.
    keys = (
        "Jfl1_hipr",
        "Jfr1_hipr",
        "Jrl1_hipr",
        "Jrr1_hipr",
        "Jfl2_wheel",
        "Jfr2_wheel",
        "Jrl2_wheel",
        "Jrr2_wheel",
    )
    indices = [i for i, n in enumerate(dof_names) if n in keys]
    return indices


def qpos0_to_dof_defaults(model: mj.MjModel, dof_names):
    """
    Convert model.qpos0 to a per-DoF default vector aligned with dof_names (qpos[7:] order).
    For hinge/slide joints, qpos0 has a single scalar at joint's qpos address.
    """
    out = np.zeros(len(dof_names), dtype=np.float64)
    for i, dof_name in enumerate(dof_names):
        jnt_id = model.dof_jntid[i + 6]  # +6 because dof_names already skipped root_free's 6 DoFs
        qpos_adr = model.jnt_qposadr[jnt_id]
        out[i] = model.qpos0[qpos_adr]
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Freeze lower-body (hip + wheel) DoFs to default positions."
    )
    parser.add_argument("--input", required=True, help="Input motion pkl.")
    parser.add_argument("--output", required=True, help="Output motion pkl.")
    parser.add_argument(
        "--robot",
        default="sr1_v1_pro",
        help="Robot name for resolving DoF order (e.g., sr1_v1_pro, sr1_v1_promax).",
    )
    parser.add_argument(
        "--default_value",
        type=float,
        default=None,
        help=(
            "Optional global default DoF value for frozen joints. "
            "If omitted, use model qpos0 defaults."
        ),
    )
    parser.add_argument(
        "--default_json",
        default=None,
        help=(
            "Optional JSON file mapping joint name to default value. "
            "Example: {\"Jfl1_hipr\": 0.05, \"Jfr1_hipr\": 0.05}"
        ),
    )
    parser.add_argument(
        "--fix_root_z",
        action="store_true",
        help="Fix root_pos[:, 2] to the median height (wheeled robots: keeps wheels on ground).",
    )
    args = parser.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model = mj.MjModel.from_xml_path(str(ROBOT_XML_DICT[args.robot]))
    dof_names_full = get_dof_names(model)
    dof_names = dof_names_full[6:]  # motion_data["dof_pos"] corresponds to qpos[7:]
    model_defaults = qpos0_to_dof_defaults(model, dof_names)

    freeze_indices = select_lower_body_indices(dof_names)
    if len(freeze_indices) == 0:
        raise RuntimeError("No lower-body DoFs matched. Check robot type and DoF names.")

    default_map = build_default_map(dof_names, model_defaults, args.default_value, args.default_json)
    freeze_values = np.array([default_map[dof_names[i]] for i in freeze_indices], dtype=np.float32)

    with open(in_path, "rb") as f:
        motion_data = pickle.load(f)

    required = {"fps", "root_pos", "root_rot", "dof_pos"}
    missing = required - set(motion_data.keys())
    if missing:
        raise KeyError(f"Input motion missing keys: {sorted(missing)}")

    dof_pos = np.asarray(motion_data["dof_pos"]).copy()
    dof_pos[:, freeze_indices] = freeze_values[None, :]

    out_data = dict(motion_data)
    out_data["dof_pos"] = dof_pos

    if args.fix_root_z:
        root_pos = np.asarray(motion_data["root_pos"]).copy()
        z_fixed = float(np.median(root_pos[:, 2]))
        root_pos[:, 2] = z_fixed
        out_data["root_pos"] = root_pos

    with open(out_path, "wb") as f:
        pickle.dump(out_data, f)

    freeze_joint_names = [dof_names[i] for i in freeze_indices]
    print("Frozen lower-body joints:")
    for n, v in zip(freeze_joint_names, freeze_values.tolist()):
        print(f"  {n}: {v}")
    if args.fix_root_z:
        print(f"Fixed root z to median: {z_fixed:.3f}m")
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
