import argparse
import pathlib
import os
import time

import numpy as np
import torch

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.kinematics_model import KinematicsModel
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast

from rich import print

if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--smplx_file",
        help="SMPLX motion file to load.",
        type=str,
        # required=True,
        default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1General_c3d/General_A1_-_Stand_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsKicks_c3d/G8_-__roundhouse_left_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/TWIST-dev/motion_data/AMASS/KIT_572_dance_chacha11_stageii.npz"
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male2MartialArtsPunches_c3d/E1_-__Jab_left_stageii.npz",
        # default="/home/yanjieze/projects/g1_wbc/GMR/motion_data/ACCAD/Male1Running_c3d/Run_C24_-_quick_side_step_left_stageii.npz",
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1", 
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "openloong", "tienkung", "fourier_gr3", "sr1_v1", "sr1_v1_pro", "sr1_v1_promax", "sr1_v2"],
        default="unitree_g1",
    )
    
    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    parser.add_argument(
        "--loop",
        default=False,
        action="store_true",
        help="Loop the motion.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )
    parser.add_argument(
        "--no_viewer",
        default=False,
        action="store_true",
        help="Retarget and save without opening the MuJoCo viewer.",
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate of the retargeted robot motion to keep the same as the human motion.",
    )
    parser.add_argument(
        "--target_fps",
        type=float,
        default=30,
        help="Target FPS for the saved/retargeted motion.",
    )
    parser.add_argument(
        "--no_pre_downsample",
        default=False,
        action="store_true",
        help="Disable pre-downsampling before the SMPL-X forward pass.",
    )
    parser.add_argument(
        "--start_frame",
        type=int,
        default=0,
        help="First source frame to process.",
    )
    parser.add_argument(
        "--end_frame",
        type=int,
        default=None,
        help="Exclusive source frame at which to stop processing.",
    )
    parser.add_argument(
        "--max_frames",
        type=int,
        default=None,
        help="Maximum number of source frames to process after applying start/end/downsampling.",
    )
    parser.add_argument(
        "--auto_ground",
        default=False,
        action="store_true",
        help="Automatically shift root z so the lowest body point touches ground before saving pkl.",
    )
    parser.add_argument(
        "--ground_offset",
        type=float,
        default=0.0,
        help="Additional z offset applied after auto grounding (meters).",
    )

    parser.add_argument(
        "--cold_start",
        default=False,
        action="store_true",
        help=(
            "Re-seed the IK to the model's neutral pose (qpos0) before every frame instead of "
            "warm-starting from the previous frame. Prevents the IK from getting stuck in a "
            "flipped branch (e.g. sr1_v2 left shoulder flipping to -170deg). Recommended for "
            "short-armed / redundant robots; combine with light DoF smoothing afterwards."
        ),
    )

    args = parser.parse_args()


    SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
    
    
    tgt_fps = args.target_fps
    smplx_meta = np.load(args.smplx_file, allow_pickle=True)
    src_fps = float(smplx_meta["mocap_frame_rate"])
    num_source_frames = smplx_meta["pose_body"].shape[0]
    start_frame = max(0, args.start_frame)
    end_frame = num_source_frames if args.end_frame is None else min(args.end_frame, num_source_frames)
    if end_frame <= start_frame:
        raise ValueError(
            f"Invalid frame range: start_frame={start_frame}, end_frame={end_frame}, "
            f"source frames={num_source_frames}"
        )

    frame_stride = 1
    if not args.no_pre_downsample and tgt_fps < src_fps:
        frame_stride = max(1, int(round(src_fps / tgt_fps)))

    selected_frames = np.arange(start_frame, end_frame, frame_stride, dtype=np.int64)
    if args.max_frames is not None:
        selected_frames = selected_frames[: args.max_frames]
    if len(selected_frames) == 0:
        raise ValueError("No SMPL-X frames selected for processing.")

    frame_indices = None
    fps_scale = 1.0
    if (
        len(selected_frames) != num_source_frames
        or selected_frames[0] != 0
        or frame_stride != 1
    ):
        frame_indices = selected_frames
        fps_scale = 1.0 / frame_stride
        print(
            f"Preprocessing SMPL-X frames: source={num_source_frames} @ {src_fps:g} FPS, "
            f"selected={len(selected_frames)} frames, stride={frame_stride}, "
            f"effective_fps={src_fps * fps_scale:g}"
        )

    # Load SMPLX trajectory
    smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
        args.smplx_file, SMPLX_FOLDER, frame_indices=frame_indices, fps_scale=fps_scale
    )
    
    # align fps
    smplx_data_frames, aligned_fps = get_smplx_data_offline_fast(smplx_data, body_model, smplx_output, tgt_fps=tgt_fps)
    
   
    # Initialize the retargeting system
    retarget = GMR(
        actual_human_height=actual_human_height,
        src_human="smplx",
        tgt_robot=args.robot,
    )

    # Neutral pose used to re-seed the IK each frame when --cold_start is set.
    neutral_qpos = retarget.configuration.model.qpos0.copy()

    robot_motion_viewer = None
    if not args.no_viewer:
        robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                                motion_fps=aligned_fps,
                                                transparent_robot=0,
                                                record_video=args.record_video,
                                                video_path=f"videos/{args.robot}_{args.smplx_file.split('/')[-1].split('.')[0]}.mp4",)
    

    curr_frame = 0
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []
    
    # Start the viewer
    i = 0

    while True:
        if args.loop:
            i = (i + 1) % len(smplx_data_frames)
        else:
            i += 1
            if i >= len(smplx_data_frames):
                break
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
        
        # Update task targets.
        smplx_data = smplx_data_frames[i]

        # retarget (optionally re-seed to neutral to avoid stuck IK branches)
        if args.cold_start:
            retarget.configuration.update(neutral_qpos)
        qpos = retarget.retarget(smplx_data)

        # visualize
        if robot_motion_viewer is not None:
            robot_motion_viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retarget.scaled_human_data,
                # human_motion_data=smplx_data,
                human_pos_offset=np.array([0.0, 0.0, 0.0]),
                show_human_body_name=False,
                rate_limit=args.rate_limit,
                follow_camera=False,
            )
        if args.save_path is not None:
            qpos_list.append(qpos)
            
    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])

        if args.auto_ground:
            device = "cuda:0" if torch.cuda.is_available() else "cpu"
            kinematics_model = KinematicsModel(retarget.xml_file, device=device)
            body_pos, _ = kinematics_model.forward_kinematics(
                torch.from_numpy(root_pos).to(device=device, dtype=torch.float),
                torch.from_numpy(root_rot).to(device=device, dtype=torch.float),
                torch.from_numpy(dof_pos).to(device=device, dtype=torch.float),
            )
            lowest_height = torch.min(body_pos[..., 2]).item()
            root_pos[:, 2] = root_pos[:, 2] - lowest_height + args.ground_offset
            print(
                f"Auto-ground enabled: lowest body z={lowest_height:.4f}, "
                f"applied ground_offset={args.ground_offset:.4f}"
            )

        local_body_pos = None
        body_names = None
        
        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")
            
      
    
    if robot_motion_viewer is not None:
        robot_motion_viewer.close()
