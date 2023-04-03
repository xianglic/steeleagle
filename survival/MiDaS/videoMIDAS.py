import cv2
import torch
import urllib.request
import numpy as np

import matplotlib.pyplot as plt

filename = "hotmetal1fps.mp4"

model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
#model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
#model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

print("Loading model...")
midas = torch.hub.load("intel-isl/MiDaS", model_type)
print("Loaded model.")

device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()
print("Model set to evaluation mode.")

midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
    transform = midas_transforms.dpt_transform
else:
    transform = midas_transforms.small_transform

print("Starting video...")

# Create a VideoCapture object
cap = cv2.VideoCapture(filename)

# Check if camera opened successfully
if (cap.isOpened() == False): 
  print("Unable to read camera feed")

# Default resolutions of the frame are obtained.The default resolutions are system dependent.
# We convert the resolutions from float to integer.
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
scrapY, scrapX = frame_height//3, frame_width//3

# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
counter = 0
while(True):
  ret, frame = cap.read()

  counter += 1
  print("Frame {0}".format(counter))

  if counter < 50:
      continue

  if ret == True: 

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    input_batch = transform(rgb_frame).to(device)

    with torch.no_grad():
        prediction = midas(input_batch)

        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=frame.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    depth_map = prediction.cpu().numpy()

    depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)
    depth_map = (depth_map*255).astype(np.uint8)
    #depth_map = cv2.applyColorMap(depth_map , cv2.COLORMAP_MAGMA)
    
    cv2.rectangle(frame, (scrapX,scrapY), (frame.shape[1]-scrapX, frame.shape[0]-scrapY), (255,255,0), thickness=1)
    depth_map[depth_map >= 190] = 0
    depth_map[depth_map != 0] = 255
    depth_map = depth_map[scrapY : frame_height - scrapY, scrapX : frame_width - scrapX]

    # convert the grayscale image to binary image
    ret, thresh = cv2.threshold(depth_map, 254, 255, 0)
    # find contours in the binary image
    contours, h = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    try:
        c = max(contours, key=cv2.contourArea)
        # calculate moments for each contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (scrapX + cX, scrapY + cY), 5, (0, 255, 0), -1)
        cv2.putText(frame, "safe", (scrapX + cX, scrapY + cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    except:
        pass

    cv2.imshow('Frame', frame)
    cv2.imshow('Depth Map', depth_map)
    # Write the frame into the file 'output.avi'
    out.write(depth_map)

    # Press Q on keyboard to stop recording
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # Break the loop
  else:
    break  

# When everything done, release the video capture and video write objects
cap.release()
out.release()

# Closes all the frames
cv2.destroyAllWindows()
