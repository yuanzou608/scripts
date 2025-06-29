import open3d as o3d
import numpy as np
import cv2
import os
import time
from pathlib import Path
from tqdm import tqdm

# --- CONFIG ---
point_cloud_path = "point_cloud.ply"
depth_dir = Path("/home/yuan/airsim/data/depth")
semantic_dir = Path("/home/yuan/airsim/data/semantic")
pose_file = "/home/yuan/airsim/data/KeyFrameTrajectory_TUM.txt"
fx, fy = 336.0, 336.0
cx, cy = 336.0, 188.0

# --- LOAD POINT CLOUD ---
pcd_global = o3d.io.read_point_cloud(point_cloud_path)
if not pcd_global.has_colors():
    pcd_global.colors = o3d.utility.Vector3dVector(np.tile(np.array([[0.7, 0.7, 0.7]]), (np.asarray(pcd_global.points).shape[0], 1)))  # gray

# --- LOAD TRAJECTORY ---
trajectory = {}
with open(pose_file, 'r') as f:
    for line in f:
        if line.startswith("#") or len(line.strip()) == 0:
            continue
        parts = line.strip().split()
        ts = parts[0]
        pose = np.eye(4)
        pose[:3, 3] = np.array(parts[1:4], dtype=np.float32)
        qx, qy, qz, qw = map(float, parts[4:])
        # Convert quaternion to rotation matrix
        R = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])
        pose[:3, :3] = R
        trajectory[ts] = pose

# --- DYNAMIC VIEWER ---
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd_global)

depth_files = sorted(depth_dir.glob("*.png"))
semantic_files = sorted(semantic_dir.glob("*.png"))

for depth_path, semantic_path in tqdm(zip(depth_files, semantic_files), total=len(depth_files)):
    ts = depth_path.stem
    if ts not in trajectory:
        continue
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0  # mm to meters
    semantic = cv2.imread(str(semantic_path), cv2.IMREAD_COLOR)

    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    pts = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)

    valid = (Z > 0).reshape(-1)
    pts = pts[valid]
    colors = semantic.reshape(-1, 3)[valid] / 255.0

    # Transform to world frame
    Tcw = trajectory[ts]
    pts_world = (Tcw[:3, :3] @ pts.T + Tcw[:3, 3:4]).T

    # Create semantic point cloud
    semantic_pcd = o3d.geometry.PointCloud()
    semantic_pcd.points = o3d.utility.Vector3dVector(pts_world)
    semantic_pcd.colors = o3d.utility.Vector3dVector(colors)

    vis.add_geometry(semantic_pcd)
    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.1)
    vis.remove_geometry(semantic_pcd)

vis.run()
vis.destroy_window()
