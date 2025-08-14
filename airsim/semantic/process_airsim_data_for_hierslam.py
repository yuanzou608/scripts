import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from tqdm import tqdm
import shutil
from scipy.spatial.transform import Rotation as R

'''
This file process airsim data for hier-slam, the output data is under /home/yuan/airsim/data/replica_format, you should copy
and paste these files like replica dataset in your destination directory
'''

def convert_airsim_to_replica_format():
    rgb_dir = "/home/yuan/airsim/data/rgb"
    depth_dir = "/home/yuan/airsim/data/depth"
    semantic_dir = "/home/yuan/airsim/data/semantic"

    out_rgb = "/home/yuan/airsim/data/replica_format/rgb"
    out_depth = "/home/yuan/airsim/data/replica_format/depth"
    out_sem = "/home/yuan/airsim/data/replica_format/vis_sem_class"

    os.makedirs(out_rgb, exist_ok=True)
    os.makedirs(out_depth, exist_ok=True)
    os.makedirs(out_sem, exist_ok=True)

    timestamps = sorted([f for f in os.listdir(rgb_dir) if f.endswith(".png")])

    for i, fname in enumerate(tqdm(timestamps, desc="Converting timestamp to replica format rgb, depth and semantic")):
        base = f"{i:06d}"
        rgb_path = os.path.join(rgb_dir, fname)
        depth_path = os.path.join(depth_dir, fname)
        sem_path = os.path.join(semantic_dir, fname)

        # Convert PNG to JPG for RGB
        rgb_img = Image.open(rgb_path).convert("RGB")  # Ensure 3 channels
        rgb_img.save(os.path.join(out_rgb, f"frame{base}.jpg"), "JPEG", quality=95)

        # Copy depth and semantic without format change
        shutil.copy(depth_path, os.path.join(out_depth, f"depth{base}.png"))
        shutil.copy(sem_path, os.path.join(out_sem, f"vis_sem_class_{i}.png"))

    print("✅ RGB converted to JPG and files renamed.")
def find_all_unique_color():
    semantic_dir = "/home/yuan/airsim/data/semantic"
    unique_colors = set()

    # Go through all .png files in the folder
    for id, fname in enumerate(sorted(os.listdir(semantic_dir))):
        if not fname.endswith(".png"):
            continue

        if id % 100 == 0:
            print(f"Processing {id}/{len(os.listdir(semantic_dir))}")

        img_path = os.path.join(semantic_dir, fname)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not read {fname}")
            continue

        # Convert BGR to RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Flatten to list of RGB tuples
        pixels = img_rgb.reshape(-1, 3)
        for color in np.unique(pixels, axis=0):
            unique_colors.add(tuple(color))

    print(f"\nFound {len(unique_colors)} unique colors across all semantic images:")
    for color in sorted(unique_colors):
        print(color)

def color_to_label(color_dict):
    for color, label in color_dict.items():
        # Color to verify
        rgb = color

        # Create a 100x100 image filled with this color
        img = np.ones((100, 100, 3), dtype=np.uint8)
        img[:] = rgb

        plt.imshow(img)
        plt.title(f"{label}: {rgb}")
        plt.axis('off')
        plt.show()

def convert_to_grayscale_semantic(color_to_id):
    semantic_dir = "/home/yuan/airsim/data/semantic"
    output_dir = "/home/yuan/airsim/data/replica_format/semantic_class"

    # === Get and sort all semantic image files ===
    image_files = sorted([
        f for f in os.listdir(semantic_dir)
        if f.lower().endswith(('.png'))
    ])

    # === Process with sequential naming ===
    for idx, fname in enumerate(tqdm(image_files, desc="Converting semantic images")):
        input_path = os.path.join(semantic_dir, fname)
        output_name = f"semantic_class_{idx}.png"
        output_path = os.path.join(output_dir, output_name)

        # Load image
        rgb_img = np.array(Image.open(input_path))

        # Initialize grayscale label map
        label_map = np.zeros((rgb_img.shape[0], rgb_img.shape[1]), dtype=np.uint8)

        # Map RGB color to class ID
        for color, class_id in color_to_id.items():
            mask = np.all(rgb_img == color, axis=-1)
            label_map[mask] = class_id

        # Save as grayscale image
        Image.fromarray(label_map).save(output_path)

def convert_gt_trajectory():
    input_file = "/home/yuan/airsim/data/groundtruth.txt"  # TUM format
    output_file = "/home/yuan/airsim/data/traj.txt"  # Hier-SLAM format

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

    print(f"done! gt trajectory saved to {output_file}")

def visualize_one_semantic_image(image):
    # Load grayscale label image (values are class IDs)
    label = np.array(Image.open(image))
    print(label.shape)
    plt.imshow(label)
    plt.axis('off')

    # Scale values for visibility: e.g., class 1 → 10, class 6 → 60
    enhanced = label * 10

    # Display with matplotlib's default colormap or 'gray'
    plt.imshow(enhanced, cmap='nipy_spectral')  # or 'tab10', 'viridis', etc.
    plt.colorbar()
    plt.title(f"Semantic Label Visualization (scaled x10)")
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    # convert airsim data to candidate replicace format
    convert_airsim_to_replica_format()

    # extract all visual color and its label
    # find_all_unique_color()

    # define color corresponding label
    color_dict = {
        (0, 0, 0): 'background',
        (81, 13, 36): 'power',
        (89, 121, 72): 'car',
        (112, 105, 191): 'bush',
        (115, 176, 195): 'sign',
        (153, 108, 6): 'tree',
        (206, 190, 59): 'furniture'
    }

    color_to_label(color_dict)

    # convert to grayscale for hierslam
    color_to_id = {
        (0, 0, 0): 0,  # background
        (81, 13, 36): 6,  # power
        (89, 121, 72): 3,  # car
        (112, 105, 191): 2,  # bush
        (115, 176, 195): 7,  # sign
        (153, 108, 6): 1,  # tree
        (206, 190, 59): 5  # furniture
    }

    # convert color semantic to gray semantic
    convert_to_grayscale_semantic(color_to_id)

    # convert tum to hierslam format
    convert_gt_trajectory()

    # visualize, in order to better visualize, times 10 for each pixel
    # image = '/home/yuan/airsim/data/replica_format/semantic_class/semantic_class_0.png'
    # visualize_one_semantic_image(image)

