#!/usr/bin/env python3

from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from playsound import playsound
import time

class instruction:

    def __init__(self):
        # Retrieve Instructions
        self.ins = rospy.Subscriber("/nav_topic", String, self.callback)
        # cpt = 2 because we need a counter that is a multiple of 2
        self.cpt = 2
        self.stock = []
        self.t0 =time.time()
        self.i = 0
        
        
    def callback(self,data):
        # If the time spent is between 0.8s and 2s, we store instructions (it is not necessary to store data before 0.4s because the user moves)
        if((time.time()-self.t0)/self.cpt > 0.4 and (time.time()-self.t0)/self.cpt < 1) :
            self.stock.append(data.data)
            self.prec = []
            
        # If the time spet is more than 1s
        elif ((time.time()-self.t0)/self.cpt >= 1):
            # i+1 because we do the loop twice
            self.i+=1
            
            # cpt + 2 because we need a counter that is a multiple of 2
            self.cpt+=2
            
            # Extract the instruction that appears the most
            instruction = max(self.stock,key = self.stock.count)
            # And store it in a list
            self.prec.append(instruction)            
            
            # If we did the loop twice, we can take the last most appeared instruction (it is more stable because it avoids the movements of the user and we took 2 times more time to process the information)
            if(not(self.i%2)):
                if (self.prec[-1] == "Left"):
                    playsound('left.mp3')
                    print("left")
                
                elif (self.prec[-1] == "Right"):
                    playsound('right.mp3')
                    print("right")
                
                elif (self.prec[-1] == "Go forward"):
                    playsound('forward.mp3')
                    print("go forward")
                
                elif (self.prec[-1] == "Scan environment"):
                    playsound('scan.mp3')
                    print("scan")
                    
            self.stock = []


def main(args):
    # Initialize the node
    ins = instruction()
    rospy.init_node('Vocal_Instruction_Node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
