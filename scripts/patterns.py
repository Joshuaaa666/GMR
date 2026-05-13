import pickle

pkl_path = "/home/joshua/桌面/GMR-master/outputs/sr1_v1_pro_walking02_01.pkl"

with open(pkl_path, "rb") as f:
    data = pickle.load(f)

print("=" * 50)
print("文件:", pkl_path)
print("Keys:", list(data.keys()))
print("=" * 50)
print("FPS:", data["fps"])
print("root_pos shape:", data["root_pos"].shape)
print("root_rot shape:", data["root_rot"].shape)
print("dof_pos shape:", data["dof_pos"].shape)
print("=" * 50)
print("第一帧 root_pos:", data["root_pos"][0])
print("第一帧 root_rot:", data["root_rot"][0])
print("第一帧 dof_pos (前10个):", data["dof_pos"][0][:10])
print("总帧数:", data["root_pos"].shape[0])
print("关节数:", data["dof_pos"].shape[1])