import airsim
import cv2
import numpy as np
import time


# Connect to AirSim running inside Docker
client = airsim.MultirotorClient()  # Change IP if needed
client.confirmConnection()

# # Set up video properties
# frame_width = 672 # Match settings.json
# frame_height = 376
# fps = 30  # Frames per second

camera_name = "front_left_custom"


print("Press 'q' to stop recording...")

camera_info = client.simGetCameraInfo(camera_name)
print('camera_info: ', camera_info)

# Start video capture loop
while True:
    # Capture an image from the front_center_custom camera

    requests = [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)]
    responses = client.simGetImages(requests)

    # Convert response to an image
    if responses and responses[0].image_data_uint8:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)  # Convert bytes to 1D array
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)  # Reshape to 3D array

        # Convert from RGB (AirSim) to BGR (OpenCV)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

        # Display image using OpenCV
        cv2.imshow("AirSim Video", img_bgr)


    else:
        print("Error: No image data received")

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
print("Video saved as airsim_output.avi")
