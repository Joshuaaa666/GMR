# json
"MFL1_HIPR": [
    "left_hip",      <-- 1. 对应人体哪个关节
    0,               <-- 2. pos_weight（位置权重）
    10,              <-- 3. rot_weight（旋转权重）
    [0.0, 0.0, 0.0], <-- 4. pos_offset（位置偏移）
    [0.4267, ...]    <-- 5. rot_offset（旋转偏移，四元数）
]
# 动作
waving
walking:outputs/sr1_v1_pro_walking02_01.pkl

cd /home/joshua/桌面/GMR-master && conda run -n gmr 
## gvmhr
python scripts/gvhmr_to_robot.py \
  --gvhmr_pred_file ../GVHMR/outputs/turn_right/001_seg0/hmr4d_results.pt \
  --robot sr1_v1_pro \
  --save_path outputs/traffic/SVR_turn_right_001_seg0.pkl \
  --record_video

python scripts/gvhmr_to_robot.py \
  --gvhmr_pred_file /home/joshua/桌面/GVHMR/outputs/demo/sr1_confu_10_20/hmr4d_results.pt \
  --robot sr1_v1_pro \
  --save_path outputs/dance/SVR_sr1_confu_10_20.pkl \
  --record_video


python scripts/smplx_to_robot.py --smplx_file /home/joshua/桌面/GMR-master/amass_data/CMU/01/01_02_stageii.npz --robot unitree_g1

python scripts/smplx_to_robot.py --smplx_file <path_to_smplx_data> --robot <path_to_robot_data> --save_path <path_to_save_robot_data.pkl> --rate_limit

cd ~/桌面/GMR-master
conda activate gmr

python scripts/smplx_to_robot.py \
  --smplx_file /home/joshua/桌面/GMR-master/amass_data/CMU/01/01_02_stageii.npz \axihhsssssffffff
  --robot unitree_g1 \
  --save_path outputs/retargeted_g1_motion.pkl


cd /home/joshua/桌面/GMR-master
conda activate gmr
# tain
python scripts/smplx_to_robot.py \
  --smplx_file amass_data/CMU/143/143_25_stageii.npz \
  --robot sr1_v1_pro \
  --save_path outputs/sr1_v1_pro_test.pkl \
  --auto_ground \
  --ground_offset 0.0

 python scripts/smplx_to_robot.py --smplx_file amass_data/CMU/02/02_01_stageii.npz --robot sr1_v1_pro --save_path outputs/sr1_v1_pro_waving02_01.pkl
# --wheelbase--
python scripts/postprocess_wheeled_base.py \
  --input /home/joshua/桌面/GMR-master/outputs/traffic/SVR_turn_right_001_seg0.pkl \
  --output /home/joshua/桌面/GMR-master/outputs/traffic/SVR_turn_right_001_seg0_wheelbase.pkl \
  --z_window 21 \
  --z_osc_scale 0.05 \
  --z_lift 0.02 \
  --attn_roll 0.20 \
  --attn_pitch 0.20 \
  --pitch_bias_deg -1.0 \
  --euler_window 9


python scripts/vis_robot_motion.py --robot sr1_v1_pro --robot_motion_path /home/joshua/桌面/GMR-master/outputs/sr1_v1_pro_walking02_01_wheelbase_v2.pkl
# --freeze_lower_body
python scripts/freeze_lower_body_dof.py \
  --input outputs/dance/xxx_wheelbase.pkl \
  --output outputs/dance/xxx_upper.pkl \
  --robot sr1_v1_pro \
  --fix_root_z

python scripts/vis_robot_motion.py \
  --robot sr1_v1_pro \
  --robot_motion_path /home/joshua/桌面/GMR-master/outputs/sr1_v1_pro_walking02_01_upper_wheelbase_body.pkl

# play
不录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1_pro \
  --robot_motion_path outputs/traffic/SVR_turn_right_001_seg0_wheelbase.pkl

python scripts/vis_robot_motion.py --robot sr1_v1_promax --robot_motion_path outputs/sr1_v1_promax_waving.pkl

录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1_pro \
  --robot_motion_path outputs/sr1_v1_pro_test.pkl \
  --record_video \
  --video_path videos/sr1_v1_pro_test.mp4

python scripts/vis_robot_motion.py \
  --robot sr1_v1_pro \
  --robot_motion_path outputs/traffic/SVR_turn_right_001_seg0_wheelbase.pkl \
  --record_video \
  --video_path videos/SVR_turn_right_001_seg0_wheelbase.mp4



# 出数据
python scripts/smplx_to_robot_dataset.py \
  --src_folder amass_data \
  --tgt_folder retargeting_data/sr1_v1 \
  --robot sr1_v1
