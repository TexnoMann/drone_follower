#!/usr/bin/env python2

# ROS
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import numpy as np
import cv2
import threading

## Setup
topicName = "/ardrone/front/image_raw"


# Circle params
center      = (100, 100)
radius      = 20
color       = (0, 255, 0)
thickness   = 3
lineType    = 7

# Required colors
color_min = np.ones((3), dtype=np.uint8)*255
color_max = np.ones((3), dtype=np.uint8)*0

frame = None
bridge = None
image_sub = None
cap = None

def nothing(x):
    pass

def generateCircleMask(frame, center, radius):
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    return mask

def imgCallback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

def process():
    global frame
    global topicName
    global cap
    global color_min
    global color_max
    global rospy

    # Capture frame-by-frame
    if (topicName == ""):
        ret, frame = cap.read()

    while (not isinstance(frame, np.ndarray) and not rospy.is_shutdown()):
        print("Wait for message in topic {}".format(topicName))
        rospy.sleep(1)

    if (rospy.is_shutdown()):
        return

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

    while(not rospy.is_shutdown()):
        # Capture frame-by-frame
        if (topicName == ""):
            ret, frame = cap.read()

        pic = frame
        key = cv2.waitKey(33)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)
        hsv_mask    = hsv[np.where(mask == 255)]
        hsv_mean    = np.mean(hsv_mask, axis=0)
        hsv_min     = np.min(hsv_mask, axis=0)
        hsv_max     = np.max(hsv_mask, axis=0)

        rgb_mean = np.mean(pic[np.where(mask == 255)], axis=0)

        # Draw to pic
        cv2.circle(pic, center, radius, rgb_mean, thickness, lineType)
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
            pic = cv2.inRange(hsv, color_min, color_max)

        cv2.imshow('frame', pic)

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
    if (topicName == ""):
        cap.release()
    cv2.destroyAllWindows()

###
### Main code
###

rospy.init_node('hsv_detector')

print("OpenCV " + cv2.__version__)

if (topicName == ""):
    # Video capture
    cap = cv2.VideoCapture(0)
else:
    bridge = CvBridge()
    image_sub = rospy.Subscriber(topicName, Image, imgCallback)

imgProcThr = threading.Thread(target = process)
imgProcThr.start()

if (cap == None):
    rospy.spin()

imgProcThr.join()