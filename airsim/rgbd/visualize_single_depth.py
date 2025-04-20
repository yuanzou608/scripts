import matplotlib.pyplot as plt
import numpy as np
import cv2


# Load the depth image
depth_path = "/home/yuan/airsim/data/depth_left_raw/depth_000000.npy"
depth_image = np.load(depth_path)

# Normalize depth values for visualization
depth_min = np.min(depth_image)
depth_max = np.max(depth_image)

# avoid infinite distance
depth_image[depth_image > 300] = 300
if depth_max > depth_min:  # Avoid division by zero
    depth_clamped = depth_image

    # Normalize depth values to 8-bit for visualization
    depth_normalized = cv2.normalize(depth_clamped, None, 0, 255, cv2.NORM_MINMAX)
    depth_8bit = depth_normalized.astype(np.uint8)

    # depth_normalized = cv2.normalize(depth_image, None, 0, 65535, cv2.NORM_MINMAX)
    # depth_uint16 = depth_normalized.astype(np.uint16)

    # Convert to BGR for imshow
    depth_vis = cv2.cvtColor(depth_8bit, cv2.COLOR_GRAY2BGR)

# Plot the depth image
plt.figure(figsize=(12, 5))

# Subplot 1: Depth Image
plt.subplot(1, 2, 1)
plt.imshow(depth_vis, cmap='jet')
plt.colorbar(label="Depth Value (Normalized)")
plt.title("Depth Image Visualization")
plt.axis("off")

# Subplot 2: Histogram of Depth Values
plt.subplot(1, 2, 2)
plt.hist(depth_image.flatten(), bins=50, color='blue', alpha=0.7)
plt.xlabel("Depth Value")
plt.ylabel("Frequency")
plt.title("Histogram of Depth Values")

# Show the plots
plt.tight_layout()
plt.show()

