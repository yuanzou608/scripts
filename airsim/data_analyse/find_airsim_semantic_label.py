import numpy as np
import cv2
from pathlib import Path

semantic_dir = Path("/home/yuan/airsim/data/semantic")
ids = set()

for p in semantic_dir.glob("*.png"):
    img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    ids.update(np.unique(img).tolist())

print("出现过的标签ID有：", sorted(ids))
