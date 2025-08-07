import numpy as np
from math import cos, sin, pi

# Define angles in radians
alpha = -90 * pi / 180  # -π/2
beta = 0 * pi / 180     # 0
gamma = -60 * pi / 180  # -π/3

# Rotation matrix around Z-axis (Yaw)
Y = np.array([
    [cos(alpha), -sin(alpha), 0],
    [sin(alpha), cos(alpha),  0],
    [0,          0,           1]
])

# Rotation matrix around Y-axis (Pitch)
P = np.array([
    [cos(beta),  0, sin(beta)],
    [0,          1, 0],
    [-sin(beta), 0, cos(beta)]
])

# Rotation matrix around X-axis (Roll)
R = np.array([
    [1, 0,           0],
    [0, cos(gamma), -sin(gamma)],
    [0, sin(gamma),  cos(gamma)]
])

# Composite rotation matrix
T_YPR = np.matmul(np.matmul(Y, P), R)

# Round to 3 decimal places
T_YPR_rounded = np.round(T_YPR, 4)

print(T_YPR_rounded)
