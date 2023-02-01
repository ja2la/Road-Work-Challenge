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


def empty(img):
    pass


class Image_Processing:
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
            
        # Convert Depth Image into matrix of float32    
        depth_array = np.array(depth_point,dtype=np.float32)
   
        
        # Trackbar values that are satisfying to extract red bars
        hue_min = 0
        hue_max = 128
        sat_min = 0
        sat_max = 255
        val_min = 156
        val_max = 255
        
        # Set HSV values
        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])
        
        # Make the mask with HSV (White pixels = red color)
        mask = cv2.inRange(rgb_image, lower, upper)
        
        
        # Apply close and open morphology to remove noise
        kernelcl = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        kernelop = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        morph = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelcl)
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, kernelop)
        
        
        # Do connected components processing to extract labels and centroids
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(morph, None, None, None, 8, cv2.CV_16U)
        
        # Get all areas from stats[label_start_id:label_stop_id, area_flag] 
        areas = stats[0:, cv2.CC_STAT_AREA]
        
        # Draw labels, get centroids and draw centroids
        result = morph.copy()
        result[:] = 0
        pts = []

        # Initialize Depth array to remove nan values (we set them at 1.1)
        depth_array_1 = depth_array.copy()
        depth_array_1[np.where(np.isnan(depth_array))]=1.1
        
        # Extract dimensions of the image
        hh, ww = rgb_image.shape[:2]
        
        depth_centroid = []
        for i in range(1, nlabels): # labels start at 1 not 0
            # if the area labeled is big anough
            if areas[i] >= 700 and areas[i] <= ww*hh/5:
                
                # Extract the centroid of the labeled bar
                pt = centroids[i]
                pts.append(pt)
                
                # Extract the position of the centroid
                cx = int(round(pt[0])-1)
                cy = int(round(pt[1])-1)
                
                # Extract the minimal depth of in a area around the centroid
                a =  np.amin(depth_array_1[cy-2:cy+2,cx-6:cx+6])
                depth_centroid.append(a)

                # If the the centroid is at less than 1m from the user
                # Display the bar on the mask (used later for the navigation part)
                if (depth_centroid[-1]<1):
                    result[labels == i] = 255
                    result[cy-2:cy+2,cx-2:cx+2] = 125
                    
        # Improve the mask to navigate better
        # Apply the mask with only nearest bars on the RGB Image
        red_color = cv2.bitwise_and(rgb_image, rgb_image, mask=result)
        # Convert in grey scale
        gray = cv2.cvtColor(red_color, cv2.COLOR_BGR2GRAY)
        # Use gaussian blur filter to remove noise
        blur_or = cv2.GaussianBlur(gray,(51,51),0)
        height = gray.shape[0]
        width = gray.shape[1]
        vertices = [(0, 0),(0, height / 4),( width , height / 4),(width, 0)]
        cv2.fillPoly(gray, np.int32([vertices]), 0)
        blur = cv2.GaussianBlur(gray,(51,51),0)
        final = (blur.T - 0.95 * np.average(blur_or,axis = 1).T).T
        final = np.absolute(-final + np.absolute(final)) / 2
        final = cv2.convertScaleAbs(final)
        _,threshold = cv2.threshold(final,np.average(final),255,cv2.THRESH_BINARY) 
        
        # Convert the new image in white/black Image
        threshold = cv2.convertScaleAbs(threshold)
        
        # Use histogram
        hist = np.sum(threshold[ int(height/ 4) : height ][:], axis=0)
        
        # Set the maximum of the histogram
        max1 = np.max(hist[0:(int(width/2)+1)])      
        max2 = np.max(hist[int(width/2):width])
         
         
        A = 0
        sum_0 = 0
        sum_1 = 0
        peakth = 10000
        count = 0
        
        # If the maximum of the histogram of the left part is above 10000,
        # increase sum_0 counter
        if(max1>10000):
            for j in range (int(width/2)):
                if(hist[j]>(peakth)):
                    sum_0 = sum_0+1
                    
        # If the maximum of the histogram of the right part is above 10000,
        # increase sum_1 counter
        if(max2>10000):
            for j in range (int(width/2)):
                if(hist[j+int(width/2)-1]>(peakth)):
                    sum_1 = sum_1+1
                    
        
        # If the depth_centroid array is empty, add a "far" point 
        # bacause an empty array may cause an error
        if not depth_centroid :
            depth_centroid.append(1.1)
            
        # Initialize topic   
        nav_pub = rospy.Publisher("/nav_topic",String)
        
        # Conditions to publish the right instruction
        # The user is at less than 1m from the obstacle AND there are less pixels on the left of the mask
        if((sum_0-sum_1)>0 and np.min(depth_centroid)<1):
            print("Left")
            nav_pub.publish(String("Left"))
        
        # The user is at less than 1m from the obstacle AND there are less pixels on the right of the mask   
        elif((sum_1-sum_0)>0 and np.min(depth_centroid)<1):
            print("Right")
            nav_pub.publish(String("Right"))
        
        # The user is at more than 1m from the obstacle
        elif(np.min(depth_centroid)>1) :
            self.stop = True
            print("Go forward")
            nav_pub.publish(String("Go forward"))
        
        # Else we ask the user to scan the environement (step on the right or the left)
        else :
            print("scan")
            nav_pub.publish(String("Scan environment"))
            #playsound('scan.mp3')
        
        
        # Display RGB Image, the Mask and the Mask with only near obstacles
        #im1 = cv2.resize(rgb_image,(360,250))
        #cv2.imshow("RGB Image",im1)
        #cv2.waitKey(3)
        #im2 = cv2.resize(morph,(360,250))
        #cv2.imshow("Mask with HSV",im2)  
        #cv2.waitKey(3)
        #im3 = cv2.resize(result,(360,250))
        #cv2.imshow("Mask with Centroids",im3)  
        #cv2.waitKey(3)

        
        
def main(args):
    # Initialize the node
    Img_Proc = Image_Processing()
    rospy.init_node('Image_Processing_Node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
   
	   
