GMR-master/
├── assets/                      ← 资源目录（机器人模型、身体模型、测试数据）
├── general_motion_retargeting/  ← 核心算法库（IK求解、重定向逻辑）
├── scripts/                     ← 可执行脚本（用户入口）
└── third_party/                 ← 第三方依赖（FBX导入器等）


cd /home/joshua/桌面/GMR-master && conda run -n gmr 


python scripts/smplx_to_robot.py --smplx_file /home/joshua/桌面/GMR-master/amass_data/CMU/01/01_02_stageii.npz --robot unitree_g1

python scripts/smplx_to_robot.py --smplx_file <path_to_smplx_data> --robot <path_to_robot_data> --save_path <path_to_save_robot_data.pkl> --rate_limit

cd ~/桌面/GMR-master
conda activate gmr

python scripts/smplx_to_robot.py \
  --smplx_file /home/joshua/桌面/GMR-master/amass_data/CMU/01/01_02_stageii.npz \
  --robot unitree_g1 \
  --save_path retargeted_g1_motion.pkl


cd /home/joshua/桌面/GMR-master
conda activate gmr
# tain
python scripts/smplx_to_robot.py \
  --smplx_file amass_data/CMU/01/01_02_stageii.npz \
  --robot sr1_v1 \
  --save_path outputs/sr1_v1_test.pkl \
  --rate_limit
# play
不录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1 \
  --robot_motion_path outputs/sr1_v1_test.pkl
录制
python scripts/vis_robot_motion.py \
  --robot sr1_v1 \
  --robot_motion_path outputs/sr1_v1_test.pkl \
  --record_video \
  --video_path videos/sr1_v1_test.mp4


# 出数据
python scripts/smplx_to_robot_dataset.py \
  --src_folder amass_data \
  --tgt_folder retargeting_data/sr1_v1 \
  --robot sr1_v1
