import airsim
from collections import Counter

client = airsim.MultirotorClient()  # or CarClient, etc.
client.confirmConnection()
id_to_objects = {}

object_names = client.simListSceneObjects()
print(len(object_names))

for name in object_names:
    label = client.simGetSegmentationObjectID(name)
    if label not in id_to_objects:
        id_to_objects[label] = []
    id_to_objects[label].append(name)

# Print unknown/remaining labels
for label_id in sorted(id_to_objects.keys()):
    print(f"Label {label_id} → {len(id_to_objects[label_id])} objects")


with open("unknown_labels.txt", "w") as f:
    for label_id, names in sorted(id_to_objects.items()):
        # if label_id != 255 or label_id != -1:  # Skip already-known background
            for name in names:
                f.write(f"{label_id} {name}\n")
