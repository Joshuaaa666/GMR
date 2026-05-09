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
walking:outputs/sr1_v1_pro_walking.pkl

cd /home/joshua/桌面/GMR-master && conda run -n gmr 


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
# play
不录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1 \
  --robot_motion_path outputs/sr1_v1_test.pkl
录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1_pro \
  --robot_motion_path outputs/sr1_v1_pro_test.pkl \
  --record_video \
  --video_path videos/sr1_v1_pro_test.mp4


# 出数据
python scripts/smplx_to_robot_dataset.py \
  --src_folder amass_data \
  --tgt_folder retargeting_data/sr1_v1 \
  --robot sr1_v1
