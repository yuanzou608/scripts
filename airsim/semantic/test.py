import matplotlib.pyplot as plt
import numpy as np

color_dict = {
    (0, 0, 0): 'background',
    (81, 13, 36): '-1',
    (89, 121, 72): '-1',
    (112, 105, 191): '-1',
    (115, 176, 195): '-1',
    (153, 108, 6): '-1',
    (206, 190, 59): '-1'
}

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
