#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class LoadFace(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        

        face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/frontalface.xml')
        eyes_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/eye.xml')

        
        img_original = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/chris.jpg')
        

        img_original = cv2.resize(img_original,(500,300))

        img = cv2.resize(img_original,(500,300))
        
        

        

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        


        ScaleFactor = 1.2
      

        minNeighbors = 3

        
        face = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
        


        for (x,y,w,h) in face:
            
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)  
            roi = img[y:y+h, x:x+w]

            eyes = eyes_cascade.detectMultiScale(roi)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi, (ex,ey),(ex+ew,ey+eh),(0,255,0),2)


          

        cv2.imshow('Face_original',img_original)
        
            

        cv2.imshow('Face',img)
        
                

        cv2.waitKey(1)



def main():
    load_face_object = LoadFace()
    rospy.init_node('load_face_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

