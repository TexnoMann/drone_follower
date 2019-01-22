#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import cv2

# Required colors
color_min = np.ones((3), dtype=np.uint8)*255
color_max = np.ones((3), dtype=np.uint8)*0

def nothing(x):
    pass

def generateCircleMask(frame, center, radius):
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask

# Circle params
center      = (100, 100)
radius      = 20
color       = (0, 255, 0)
thickness   = 3
lineType    = 7

# Video capture
cap = cv2.VideoCapture(0)
print("OpenCV " + cv2.__version__)

# Capture frame-by-frame
ret, frame = cap.read()
mask = generateCircleMask(frame, center, radius)

# Logging
data = [];
record = False
test = False

# Create trackbars
cv2.namedWindow('frame')

cv2.createTrackbar('H_min', 'frame', 0, 255, nothing)
cv2.createTrackbar('S_min', 'frame', 0, 255, nothing)
cv2.createTrackbar('V_min', 'frame', 0, 255, nothing)

cv2.createTrackbar('H_max', 'frame', 0, 255, nothing)
cv2.createTrackbar('S_max', 'frame', 0, 255, nothing)
cv2.createTrackbar('V_max', 'frame', 0, 255, nothing)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    key = cv2.waitKey(33)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_mask    = hsv[np.where(mask == 255)]
    hsv_mean    = np.mean(hsv_mask, axis=0)
    hsv_min     = np.min(hsv_mask, axis=0)
    hsv_max     = np.max(hsv_mask, axis=0)

    rgb_mean = np.mean(frame[np.where(mask == 255)], axis=0)

    # Draw to frame
    cv2.circle(frame, center, radius, rgb_mean, thickness, lineType)
    if record:
        # print(hsv_max)
        data.append(hsv_max)
        color_min = np.minimum(color_min, hsv_max)
        color_max = np.maximum(color_max, hsv_max)
        cv2.setTrackbarPos('H_min','frame', color_min[0])
        cv2.setTrackbarPos('S_min','frame', color_min[1])
        cv2.setTrackbarPos('V_min','frame', color_min[2])
        cv2.setTrackbarPos('H_max','frame', color_max[0])
        cv2.setTrackbarPos('S_max','frame', color_max[1])
        cv2.setTrackbarPos('V_max','frame', color_max[2])

    if test:
        # get current positions of trackbars
        color_min[0] = cv2.getTrackbarPos('H_min','frame')
        color_min[1] = cv2.getTrackbarPos('S_min','frame')
        color_min[2] = cv2.getTrackbarPos('V_min','frame')
        color_max[0] = cv2.getTrackbarPos('H_max','frame')
        color_max[1] = cv2.getTrackbarPos('S_max','frame')
        color_max[2] = cv2.getTrackbarPos('V_max','frame')
        frame = cv2.inRange(hsv, color_min, color_max)

    cv2.imshow('frame', frame)

    if key == ord('q'):
        break
    elif key == ord('r'):
        record = True
        test = False
    elif key == ord('s'):
        test = True
        record = False

data = np.array(data)
end = data.shape[0]

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
