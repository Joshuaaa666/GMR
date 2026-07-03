"""Generate a MuJoCo mocap XML for SR1 V2.0.0 (Kirin-Body) from its URDF.

V2 只有两条 7-DOF 手臂可动，躯干/腰/头/轮子在 URDF 里是 fixed。
本脚本让 MuJoCo 直接编译 URDF 得到正确的运动学树，再补上：
  - MBASE 上的 freejoint（浮动基座，供重定向根位姿用）
  - 地面 / 光照 / 视觉设置

用法：
    cd assets/SR1_V2.0.0 && python gen_mocap_xml.py
输出：assets/SR1_V2.0.0/sr1_v2_mocap.xml
"""
import re
from pathlib import Path
import mujoco

HERE = Path(__file__).resolve().parent
URDF = HERE / "urdf" / "Kirin-Body.urdf"
PROCESSED = HERE / "_kirin_processed.urdf"
RAW = HERE / "_sr1_v2_raw.xml"
OUT = HERE / "sr1_v2_mocap.xml"

# 1) 预处理 URDF：把 package:// 路径改成相对本文件目录，MuJoCo 才能找到 mesh
text = URDF.read_text()
text = text.replace("package://kirin_body-description/", "")
# 头/左右手末端 link 的 visual 是 mesh、collision 是 box；MuJoCo 编译 URDF 时
# discardvisual 默认 true 会丢掉 visual mesh 只留 box（渲染成方块）。注入
# mujoco 扩展块把它设为 false，保留 visual mesh（box 之后再删）。
text = text.replace(
    '<robot name="SR1_V1_0">',
    '<robot name="SR1_V1_0">\n  <mujoco><compiler discardvisual="false"/></mujoco>',
    1,
)
PROCESSED.write_text(text)

# 2) 编译 URDF -> 载入 -> 保存为 MJCF
model = mujoco.MjModel.from_xml_path(str(PROCESSED))
mujoco.mj_saveLastXML(str(RAW), model)
print(f"compiled URDF ok: {model.nbody} bodies, {model.njnt} joints")

raw = RAW.read_text()

# 3) URDF 根链 MBASE 是 fixed 焊在 world 上，MuJoCo 编译时把它并进了 worldbody
#    （torso geom 变静态、两条臂链直接挂 world，没有 MBASE body 也没 freejoint）。
#    这里把整个 worldbody 内容（torso 静态 geom + 两条臂链）重新包进一个带
#    freejoint 的 MBASE 浮动基座，坐标保持世界绝对值不变（MBASE 置于原点）。
mbase_open = (
    '<worldbody>\n'
    '    <body name="MBASE" pos="0 0 0">\n'
    '      <freejoint name="root_free"/>\n'
    '      <inertial pos="0 0 0.4" mass="20" diaginertia="0.5 0.5 0.5"/>'
)
raw = raw.replace('<worldbody>', mbase_open, 1)
raw = raw.replace('</worldbody>', '    </body>\n  </worldbody>', 1)

# 4) meshdir 改成相对路径，便于移植
raw = re.sub(r'meshdir="[^"]*"', 'meshdir="meshes/2"', raw)

# 4b) 删掉 collision box 几何（头/左右手的包围盒方块）；运动学重定向不需要碰撞，
#     visual mesh 已由 discardvisual=false 保留，删 box 后只剩漂亮 mesh。
raw = re.sub(r'\n\s*<geom[^>]*type="box"[^>]*/>', '', raw)

# 4c) 收紧肩 pitch (J1_shp) 限位，排除"翻转"IK 分支。
#     原始 URDF: L_J1_shp[-2.96,1.39] / R_J1_shp[-1.39,2.96]，能到 ±170°。
#     真人动作里肩 pitch 只用到约 -107°/+107°，但 IK(位置匹配+冗余+短臂)会掉进
#     "肩往后翻到 -170° + 肘折回"的虚假局部解，warm-start 又把它黏住 → 手臂突然往后伸/抽帧。
#     把翻转唯一会用到的 |angle|>115° 那段砍掉（真人动作用不到），warm-start 就翻不过去。
#     注意仅为"重定向用模型"的限位，不改真机 ROM。
J1_CLAMP = 2.0  # rad ≈ 114.6°，留足自然动作(≤107°)余量
raw = re.sub(r'(<joint name="L_J1_shp"[^>]*?)(?<![a-z])range="[^"]*"',
             rf'\g<1>range="-{J1_CLAMP} 1.39"', raw)
raw = re.sub(r'(<joint name="R_J1_shp"[^>]*?)(?<![a-z])range="[^"]*"',
             rf'\g<1>range="-1.39 {J1_CLAMP}"', raw)

# 4d) 给两只手各加一个碰撞球（原 box 碰撞体已删），仅供 IK 自碰撞规避用：
#     contype/conaffinity=1、group=3(默认 viewer 不显示)、半透明红、半径覆盖手掌核心。
#     ★ 放在 L_L6_wrp(腕 roll J7 之前的连杆)、而非 L_L7_wrr：否则球心偏离 roll 轴，
#     IK 会靠转动未被跟踪的腕 roll(J7) 来"免费"分开双手，把手拧歪。放 L6 后 J7 无从作弊。
HAND_COL = ('\\g<0>\n'
            '        <geom name="{side}_hand_col" type="sphere" pos="0.009 0 -0.126" size="0.10" '
            'contype="1" conaffinity="1" group="3" rgba="1 0 0 0.3"/>')
raw = re.sub(r'<body name="L_L6_wrp"[^>]*>',
             HAND_COL.format(side="L"), raw, count=1)
raw = re.sub(r'<body name="R_L6_wrp"[^>]*>',
             HAND_COL.format(side="R"), raw, count=1)

# 5) 在 </mujoco> 前补地面/光照/视觉
extra = """
  <statistic center="0 0 1.0" extent="1.2"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-140" elevation="-20" offwidth="2080" offheight="1170"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".4 .5 .6" rgb2="0 0 0" width="100" height="100"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1"
      markrgb="0 0 0" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.01" type="plane" material="groundplane" contype="1" conaffinity="0"
      priority="1" friction="0.6" condim="3"/>
    <light diffuse=".5 .5 .5" pos="-3 -3 5" dir="3 3 -5" castshadow="true"/>
  </worldbody>
"""
raw = raw.replace("</mujoco>", extra + "</mujoco>")

OUT.write_text(raw)
print(f"wrote {OUT}")

# 清理中间文件
PROCESSED.unlink(missing_ok=True)
RAW.unlink(missing_ok=True)

# 6) 复核：能否加载，并打印手臂 body 名（供写 IK 配置）
m2 = mujoco.MjModel.from_xml_path(str(OUT))
names = [mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m2.nbody)]
arm = [n for n in names if n and (n.startswith("L_L") or n.startswith("R_L"))]
print("arm bodies:", arm)
