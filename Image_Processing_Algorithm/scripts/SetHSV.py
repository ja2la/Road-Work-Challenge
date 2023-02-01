#!/usr/bin/env python3

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
import argparse
import math


def empty(img):
    pass


class SetHSV:
    def __init__(self):
        # Retrieve RGB Image
        self.rgb_image = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        
        # Retrieve Depth Image
        self.depth_image = message_filters.Subscriber("/camera/depth_registered/image_raw",Image)
        
        # Synchronize both Images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image, self.depth_image], 1, 1)
        
        # Call the callback function
        self.ts.registerCallback(self.callback)

    
    
    def callback(self, rgb_image, depth_image):
        # CV Bridge to convert ROS image in CV2 Image
        br = CvBridge()
        
        # Convert RGB Image with CV Bridge
        try :
            rgb_image = br.imgmsg_to_cv2(rgb_image,"bgr8")
        except CvBridgeError as e :
            print(e)
        
        # Convert Depth Image with CV Bridge
        try :
            depth_point = br.imgmsg_to_cv2(depth_image, "32FC1")
        except CvBridgeError as e :
            print(e)
            
            
        # Get the value of the trackbar postition
        hue_min = cv2.getTrackbarPos("hue_min", "Trackbar")
        hue_max = cv2.getTrackbarPos("hue_max", "Trackbar")
        sat_min = cv2.getTrackbarPos("sat_min", "Trackbar")
        sat_max = cv2.getTrackbarPos("sat_max", "Trackbar")
        val_min = cv2.getTrackbarPos("val_min", "Trackbar")
        val_max = cv2.getTrackbarPos("val_max", "Trackbar")
        
       
        # Set HSV values
        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])
        
        # Make the mask with HSV (White pixels = red color)
        mask = cv2.inRange(rgb_image, lower, upper)
        
        # Apply close and open morphology to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, kernel)
        
        
        # Display RGB Image and the Mask
        im1 = cv2.resize(rgb_image,(360,250))
        cv2.imshow("RGB Image",im1)
        cv2.waitKey(3)
        im2 = cv2.resize(morph,(360,250))
        cv2.imshow("Mask with HSV",im2)
        cv2.waitKey(3)
        
   

        
def main(args):
    # Display Trackbars
    cv2.namedWindow('Trackbar')
    cv2.createTrackbar("hue_min", "Trackbar", 0, 179, empty)
    cv2.createTrackbar("hue_max", "Trackbar", 179, 179, empty)
    cv2.createTrackbar("sat_min", "Trackbar", 0, 255, empty)
    cv2.createTrackbar("sat_max", "Trackbar", 255, 255, empty)
    cv2.createTrackbar("val_min", "Trackbar", 0, 255, empty)
    cv2.createTrackbar("val_max", "Trackbar", 255, 255, empty)
    
    # Initialize the node
    set_function = SetHSV()
    rospy.init_node('Image_Processing_Set_HSV', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
   
	   
