# SR1 V2 重定向命令手册（GVHMR 视频 / AMASS 全流程）

> 重定向到 SR1 V2（固定躯干 + 双 7-DOF 臂）→ 可视化/后处理。两种输入：
> - **路线 A（视频）**：视频 → GVHMR 提取 SMPL-X → 重定向。见步骤 0→1→2→3…
> - **路线 B（已有 SMPL-X）**：AMASS/CMU 等 .npz 直接重定向。见步骤 0→2'→3…
> 两个 conda 环境：`gvhmr`（提取，仅路线 A）、`gmr`（重定向/可视化）。

---

## 0. 一次性：生成 V2 整机 MJCF
URDF 变动后才需重跑；已保留 visual mesh 并删掉 collision box（否则头/手渲染成方块）。
```bash
conda activate gmr
cd /home/joshua/桌面/GMR-master/assets/SR1_V2.0.0
python gen_mocap_xml.py          # 输出 sr1_v2_mocap.xml
```

## 1. GVHMR 提取 SMPL-X
固定机位加 `-s` 跳过 SLAM。⚠ 视频名不能含中文/特殊字符（hydra 会报错），先复制成 ASCII 名。
```bash
conda activate gvhmr
cd /home/joshua/桌面/GVHMR
cp "inputs/data/BV1gT4y1m7ec_健身气功八段锦完整版-带呼吸法口令版_15s_3min.mp4" inputs/data/baduanjin.mp4
python tools/demo/demo.py --video inputs/data/baduanjin.mp4 --output_root outputs/baduanjin -s
# 输出：outputs/baduanjin/baduanjin/hmr4d_results.pt
```

## 2. 重定向到 sr1_v2
左臂"翻转解"（`L_J1_shp` 甩到 -170° = 手臂突然往后伸/抽帧）已通过**在模型里夹紧 J1 肩pitch下限到 -114.6°**（步骤 0 的 `gen_mocap_xml.py` 里）根治，warm start 就平滑不翻，**无需 `--cold_start`**。
```bash
conda activate gmr
cd /home/joshua/桌面/GMR-master
python scripts/gvhmr_to_robot.py \
  --gvhmr_pred_file /home/joshua/桌面/GVHMR/outputs/baduanjin/baduanjin/hmr4d_results.pt \
  --robot sr1_v2 \
  --save_path output/baduanjin_sr1_v2.pkl
# ⚠ 别同时加 --record_video：交互 viewer 用 GLX，与录像 EGL 离屏渲染冲突(GLXBadDrawable)
# --cold_start 仍保留为可选兜底（每帧冷启动，能消翻转但会引入跳帧，一般不用）
```

## 2'. 另一条输入路线：AMASS SMPL-X（.npz，不走 GVHMR）
输入已经是 SMPL-X 的（如 AMASS/CMU），跳过步骤 1，直接用 `smplx_to_robot.py`。
```bash
conda activate gmr
cd /home/joshua/桌面/GMR-master
python scripts/smplx_to_robot.py \
  --smplx_file amass_data/CMU/12/12_04_stageii.npz \
  --robot sr1_v2 \
  --save_path output/12_04_sr1_v2.pkl \
  --no_viewer      # 无头保存；去掉 --no_viewer 则开交互窗（需显示器）
```
之后接步骤 3/4/5（后处理/可视化/调试）与 GVHMR 路线完全一样。

> 注：机器人左肩 roll 是**单向 ROM**，做不了"交叉过身体中线举高"的动作（如某些云手），这类帧左臂会举高/略举后逼近极限——是硬件 ROM 固有限制，非 bug（放宽 ROM 实测反而更糟）。

## 3. 后处理（原地站桩：定高 + 不漂移 + 不转身 + 去 IK 抖动）
sr1_v2 是纯双臂，`--freeze_lower/waist/arms` 无效（关节名前缀是 V1 的），只对根位姿有意义。
```bash
python scripts/lock_upper_body_pkl.py \
  --input output/12_04_sr1_v2.pkl \
  --output output/12_04_sr1_v2_locked.pkl \
  --robot sr1_v2 \
  --fix_root_z \
  --fix_root_xy \
  --root_yaw_deg 0 \
  --smooth_dof_window 5
```

## 4. 可视化
```bash
# 交互回放（需显示器）
python scripts/vis_robot_motion.py --robot sr1_v2 --robot_motion_path output/12_04_sr1_v2.pkl

# 无头离屏录 mp4
MUJOCO_GL=egl python scripts/vis_robot_motion.py \
  --robot sr1_v2 --robot_motion_path output/baduanjin_sr1_v2_locked.pkl \
  --record_video --video_path videos/baduanjin_sr1_v2.mp4
```

## 5. 调试工具
```bash
# 逐帧/实时打印关节角（--filter L_J 只看左臂；--deg 度数；去掉 --no_realtime 则按 fps 实时播放式打印）
python scripts/print_pkl_dofs.py output/baduanjin_sr1_v2.pkl --robot sr1_v2 --deg --no_realtime --stride 30
python scripts/print_pkl_dofs.py output/baduanjin_sr1_v2.pkl --robot sr1_v2 --deg --filter L_J

# DoF 概览（每关节均值/幅度/限位）
python scripts/inspect_motion_pkl.py output/baduanjin_sr1_v2.pkl --robot sr1_v2 --all_dofs
```

---

## 换一个新视频怎么跑
把步骤 1 的 `baduanjin` 换成你的 ASCII 名、`output_root` 换个目录；步骤 2/3 的 `--gvhmr_pred_file` 和 `--save_path` 改成对应路径即可。步骤 0 不用重跑（除非改了 URDF）。

## 关键坑速查
| 坑 | 现象 | 解 |
|---|---|---|
| 中文文件名 | hydra `OverrideParseException` | 复制成 ASCII 名再跑 |
| 头/手是方块 | URDF visual=mesh/collision=box，`discardvisual` 默认丢 mesh | `gen_mocap_xml.py` 已注入 `discardvisual=false` + 删 box |
| 整机悬空 | `MBASE←pelvis` 把底盘抬到腰高 | 配置用 `MBASE←left_foot`（贴地） |
| 左臂翻转/往后伸抽帧 | `L_J1_shp` 甩到 -170°（IK 冗余+短臂+warm迟滞掉进翻转解） | 模型里已夹紧 J1 下限到 -114.6°（`gen_mocap_xml.py`），warm 即平滑不翻；`--cold_start` 为可选兜底 |
| 左臂交叉动作举后 | 云手类过中线举高，左肩单向 ROM 够不到 | 硬件固有限制，放宽 ROM 反而更糟；只能选无交叉动作或接受 |
| 双手/手穿躯干碰撞 | 纯 IK 无碰撞约束，两手末端穿模 | 已给手加碰撞球 + IK 配 `self_collision`（`smplx_to_sr1_v2.json`）；依赖 `solve_ik` limits 修复 |
| IK 约束从不生效 | GMR 把 `ik_limits` 传成了 `solve_ik` 的 `safety_break`，`limits` 恒 None | 已修：`motion_retarget.py` 改为 `limits=self.ik_limits`（关节/速度/碰撞约束现在才真正生效）|
| viewer+录像冲突 | `GLXBadDrawable` | 交互与 `--record_video` 分开跑 |

## 关键文件
- 模型：`assets/SR1_V2.0.0/sr1_v2_mocap.xml`（生成脚本 `gen_mocap_xml.py`）
- IK 配置：`general_motion_retargeting/ik_configs/smplx_to_sr1_v2.json`
- 注册：`general_motion_retargeting/params.py`（`sr1_v2` 已加入 4 个字典）
- 完整排查记录：`docs/process_boared.md`
