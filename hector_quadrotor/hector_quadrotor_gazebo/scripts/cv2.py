#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class CannyFilter(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/front_cam/camera/image",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

          
        
        img = cv2.resize(cv_image,(450,350))

        #The canny detector uses two parameters appart from the image:
        #The minimum and maximum intensity gradient
        minV = 30
        maxV = 100

        edges = cv2.Canny(img,minV,maxV)
        cv2.imshow('Original',img)
        cv2.imshow('Edges',edges)

        cv2.waitKey(1)



def main():
    canny_filter_object = CannyFilter()
    rospy.init_node('canny_filter_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

