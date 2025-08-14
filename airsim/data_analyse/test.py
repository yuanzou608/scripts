import open3d as o3d
import numpy as np
from sklearn.neighbors import KDTree

# 加载 photo-slam 点云
pcd = o3d.io.read_point_cloud("point_cloud.ply")
gaussian_xyz = np.asarray(pcd.points)

# 构建 KD-Tree
tree = KDTree(gaussian_xyz)

# 假设你已有反投影得到的语义点和标签
projected_xyz = np.load("projected_xyz.npy")      # shape (M, 3)
projected_label = np.load("projected_label.npy")  # shape (M,)

# 初始化语义标签数组（-1 表示未赋值）
semantic_labels = -1 * np.ones(len(gaussian_xyz), dtype=np.int32)

# 进行最近邻匹配（可以按语义投票策略改进）
dist, idx = tree.query(projected_xyz, k=1)
idx = idx.flatten()

for i, gi in enumerate(idx):
    semantic_labels[gi] = projected_label[i]

# 可视化：将 label 映射为颜色
def label_to_color(label):
    colormap = {
        1: [255, 0, 0],
        2: [0, 255, 0],
        3: [0, 0, 255],
        # 添加你自己的类别颜色
    }
    return np.array([colormap.get(l, [100, 100, 100]) for l in label]) / 255.0

pcd.colors = o3d.utility.Vector3dVector(label_to_color(semantic_labels))
o3d.io.write_point_cloud("point_cloud_with_semantics.ply", pcd)
