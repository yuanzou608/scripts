import open3d as o3d

pcd = o3d.io.read_point_cloud("semantic_pointcloud.ply")
o3d.visualization.draw_geometries([pcd])
pcd.paint_uniform_color([0.5, 0.5, 0.5])
import numpy as np
Z = np.asarray(pcd.points)[:, 2]
print(f"Z range: {Z.min():.2f} → {Z.max():.2f}")
