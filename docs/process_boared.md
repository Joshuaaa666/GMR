# 项目进度板 · SR1 V2 重定向 + 八段锦动作

> 记录 SR1_V2.0.0 新机型接入重定向，以及用 GVHMR 从八段锦视频提取动作并重定向到 SR1 的全过程。

---

## 一、SR1 V1.0.0 vs V2.0.0 差异

| 部位 | V1.0.0 | V2.0.0 (Kirin-Body) |
|---|---|---|
| 髋转向 Jfl/fr/rl/rr1_hipr | revolute 可动 | **fixed 锁死** |
| 腰 Jwb1/2/3 (roll/yaw/pitch) | revolute 3-DOF | **fixed 锁死** |
| 头 Jhd1/2 (yaw/pitch) | revolute 2-DOF | **fixed 锁死** |
| 轮子 J*2_wheel | continuous 可动 | **fixed 锁死** |
| 手臂 | 左右各 6-DOF | 左右各 **7-DOF**（新增腕 roll `*_wrr`，3-DOF 球腕）|
| 命名 | `MAL*/MAR*`，关节 `Jal*/Jar*` | `L_*/R_*`，关节 `L_J*/R_J*` |

**结论**：V2 是"固定躯干 + 双 7-DOF 臂"平台。MuJoCo 编译 URDF 时所有 fixed 链被合并进 `MBASE`，模型只剩 14 个可动关节（7+7 臂）。

---

## 二、已完成：V2 接入重定向（方案：保持双臂 + 固定躯干）

1. **MJCF**：`assets/SR1_V2.0.0/sr1_v2_mocap.xml`
   - 由 `urdf/Kirin-Body.urdf` 用 MuJoCo 3.7 编译生成，补了浮动基座 freejoint + 地面/光照。
   - 生成脚本：`assets/SR1_V2.0.0/gen_mocap_xml.py`（URDF 更新后可重跑）。
   - ⚠ 无 actuator（URDF 未带）；重定向只输出 qpos 不受影响，若需部署可参照 V1 补 `<actuator>`。

2. **IK 配置**：`general_motion_retargeting/ik_configs/smplx_to_sr1_v2.json`
   - 基于 `smplx_to_sr1_v1_pro.json` 改。
   - 臂目标：`L_L2_shr/L_L4_elp/L_L6_wrp` + `R_*`（腕瞄准 `L_L6_wrp` 单位帧，避开被转 90° 的末端 `L_L7_wrr`）。
   - 关键：V2 臂 body 静止朝向与 V1 一致（单位四元数），故 V1_pro 的旋转偏移四元数可直接复用作为可靠初值。
   - 删掉了已被合并、不再存在的 `MWB*` 腰部项。

3. **params.py**：`sr1_v2` 已加入 `ROBOT_XML_DICT` / `IK_CONFIG_DICT[smplx]` / `ROBOT_BASE_DICT`(=MBASE) / `VIEWER_CAM_DISTANCE_DICT`(=5.0)。

4. **验证**：`GeneralMotionRetargeting("smplx","sr1_v2")` 构建成功，14 DoF + 7 IK 目标全部解析正常。

5. **gvhmr_to_robot.py**：`--robot` choices 已补 `sr1_v2`（原来只到 `sr1_v1_pro`，即"1.0 版本"）。

---

## 三、已完成：八段锦视频 → GVHMR → 重定向到 sr1_v2

- **输入视频**：`GVHMR/inputs/data/BV1gT4y1m7ec_健身气功八段锦完整版-带呼吸法口令版_15s_3min.mp4`（165s，4125 帧，852×480）。

### 踩过的三个坑（均已解决）
1. **中文文件名**：hydra 无法解析含中文/特殊字符的 `video_name`。→ 复制成 ASCII 名 `inputs/data/baduanjin.mp4` 再跑。
2. **`KeyError: 'pelvis'`**：`offset_human_data()` 会遍历所有存活 human body（= 根 + `human_scale_table`），每个都要有偏移条目；条目只来自 `ik_match_table`。V1_pro 靠腰部 `MWB1/2/3←left_knee/left_hip/pelvis` 提供，V2 删了腰部映射导致 pelvis 无条目。→ 改 `motion_retarget.py:offset_human_data`，未映射 body 用单位/零偏移透传（`.get(...)`，向后兼容其它机型）。
3. **机器人整体悬空**：曾把 `MBASE←pelvis`，导致底盘被抬到人盆高度(~0.9m)、肩升到 ~2m、手臂失真。→ 改回 V1 的 `MBASE←left_foot`（底盘贴地，高躯干使肩自然到 ~1.26m≈人肩）。修坑2后 pelvis 作为根可直接透传，此方案成立。

### 提取命令（gvhmr 环境，固定机位用 `-s` 跳过 SLAM）
```bash
conda activate gvhmr && cd /home/joshua/桌面/GVHMR
python tools/demo/demo.py --video inputs/data/baduanjin.mp4 --output_root outputs/baduanjin -s
# 输出：outputs/baduanjin/baduanjin/hmr4d_results.pt (17.9MB)
```

### 重定向（gmr 环境）
- 标准脚本（带交互 viewer，需显示器）：
  ```bash
  conda activate gmr && cd /home/joshua/桌面/GMR-master
  python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file /home/joshua/桌面/GVHMR/outputs/baduanjin/baduanjin/hmr4d_results.pt \
    --robot sr1_v2 --save_path output/baduanjin_sr1_v2.pkl
  ```
  ⚠ 该脚本的 `RobotMotionViewer` 用 GLX 开交互窗，与 `--record_video` 的 EGL 离屏渲染冲突（GLXBadDrawable）。要无头录像就分开：先纯 IK 存 pkl，再单独用 `MUJOCO_GL=egl` 离屏渲染。

### 产物
- `output/baduanjin_sr1_v2.pkl` — 4125 帧，qpos=21（7 freejoint + 14 臂 DoF），root_pos z∈[0,0.24]（贴地）。
- `output/baduanjin_sr1_v2_preview.mp4` — 预览视频。
- `output/baduanjin_sr1_v2_frames.png` — 9 宫格关键帧。

### 结果评估
双臂清晰复现八段锦动作（如"两手托天理三焦"双臂上举、两臂平展等），底盘贴地、躯干直立、手臂自然。臂角度即为可用输出。

### 坑4（已解决）：头/手末端渲染成方块
- 头 `MHD2_HEADP`、左右手末端 `L_L7_wrr/R_L7_wrr` 在 URDF 里 visual=mesh、collision=box。
- MuJoCo 编译 URDF 时 `discardvisual` **默认 true**，丢弃 visual mesh 只留 collision box → 渲染成方块。
- 修法（`gen_mocap_xml.py`）：向 URDF 注入 `<mujoco><compiler discardvisual="false"/></mujoco>` 保留 visual mesh，再在后处理删掉所有 `type="box"` 碰撞盒（运动学重定向不需碰撞）。
- 只改外观、不动运动学，pkl 无需重跑，重渲染即可。现头是头部 mesh、手末端是带手指的手 mesh。

### 坑5（已解决）：左臂翻转 / 左右不对称
- 现象：`L_J1_shp`（左肩 pitch）在部分帧甩到 -160°~-170°（整条左臂往后翻），右臂正常；约 28% 帧受影响。IK 逐帧热启动的迟滞：中途翻进"翻转解"后，每帧都从上一帧(已翻)热启动，卡住出不来。
- 排查过程：
  - 左右臂关节 axis/range 是**正确镜像**，模型没坏。
  - 扫 10 个候选左 rot_offset：当前值已近最优；随机搜 SO(3) 也只微改 → **不是朝向偏移问题**。
  - 左臂朝向权重 10→0：翻转不变 → 不是朝向权重问题。
  - 平均可达性 OK（人臂展 0.385m < 机器人臂长 0.48m），肩位差仅 0.15m → 不是尺度问题。
  - **单独重解翻转帧不翻**（L_J1≈-20°），但顺序播放就翻 → **IK 逐帧热启动的迟滞**：中途翻进"翻转解"后每帧从上一帧(已翻)热启动，卡住出不来。
  - PostureTask 低权重拉不回（1174→1133）。
- 修法：**每帧把 IK 冷启动到中性姿态 `qpos0` 再解**（`gvhmr_to_robot.py --cold_start`）→ 翻转帧 1174 → **0**，`L_J1_shp` 回到 -64°~+46°。冷启动带来极少数大跳，用 `--smooth_dof_window 5` 平滑。
- 工具：`scripts/print_pkl_dofs.py`（逐帧/实时打印关节角）、`scripts/inspect_motion_pkl.py --all_dofs`（DoF 概览）。
- 产物已用冷启动重生成：`output/baduanjin_sr1_v2.pkl` / `_preview.mp4` / `_leftarm_fixed.png`。

### 坑6（已解决）：双手/手末端穿模 + 一个影响全局的 IK bug
- 现象：纯运动学 IK 无碰撞约束，太极里两手末端最近球心距 0.112m（穿模 ~4cm）。
- 加 `CollisionAvoidanceLimit` 后**完全无效**——排查发现根因是 **`mink.solve_ik` 参数错位**：
  其签名为 `solve_ik(config, tasks, dt, solver, damping, safety_break, limits, ...)`，
  而 GMR 一直按 `(..., self.damping, self.ik_limits)` 调用，把 `ik_limits` 塞进了第 6 个
  位置参数 `safety_break`，`limits` 恒为 None → **所有 IK 约束（关节限位/速度/碰撞）从未生效**。
- 修法：
  1. `motion_retarget.py` 4 处 `solve_ik` 改为 `limits=self.ik_limits`（关键修复）。
  2. `gen_mocap_xml.py` 给两手末端 `L_L7_wrr/R_L7_wrr` 各加碰撞球 `*_hand_col`（原 box 碰撞体删了）。
  3. `motion_retarget.py` 支持 IK 配置里的 `self_collision.body_pairs`，自动建 `CollisionAvoidanceLimit`。
  4. `smplx_to_sr1_v2.json` 加 `self_collision`：左臂↔右臂、左手↔躯干、右手↔躯干。
- 效果：双手球心最近 0.112→0.161m（**穿模帧 15→0**），仍不翻、平滑。
- ⚠ 副作用：修复后关节限位/速度也真正成为硬约束（之前靠碰巧不越界）——对所有机型都更正确。

### 若需进一步精修
微调 `smplx_to_sr1_v2.json` 各臂目标的旋转偏移四元数/权重（手腕朝向、手部抓握等）。
