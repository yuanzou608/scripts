import os

# Paths
image_folder = '/home/yuan/dataset/textslam/Seq_01_no_text/images'
text_folder = '/home/yuan/dataset/textslam/Seq_01_no_text/text'

# Create the output folder if it doesn't exist
os.makedirs(text_folder, exist_ok=True)

# Loop over all files in the image folder
for filename in os.listdir(image_folder):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
        # Get the image name without extension
        name_without_ext = os.path.splitext(filename)[0]

        # Create output file paths
        dete_path = os.path.join(text_folder, f"{name_without_ext}_dete.txt")
        mean_path = os.path.join(text_folder, f"{name_without_ext}_mean.txt")

        # Create empty files
        open(dete_path, 'w').close()
        open(mean_path, 'w').close()

print("Empty .txt files created for each image.")
