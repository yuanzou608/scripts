import numpy as np
from scipy.spatial.transform import Rotation as R

input_file = "groundtruth.txt"     # TUM format
output_file = "traj.txt"           # Hier-SLAM format

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    for line in f_in:
        if line.startswith("#") or len(line.strip()) == 0:
            continue  # skip comments and empty lines

        parts = list(map(float, line.strip().split()))
        if len(parts) != 8:
            continue  # invalid line

        tx, ty, tz = parts[1:4]
        qx, qy, qz, qw = parts[4:]

        # Convert quaternion to rotation matrix
        rot = R.from_quat([qx, qy, qz, qw]).as_matrix()

        # Form 4x4 matrix
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = [tx, ty, tz]

        # Flatten to row-major
        pose_flat = pose.flatten()
        pose_str = " ".join(f"{v:.18e}" for v in pose_flat)
        f_out.write(pose_str + "\n")
