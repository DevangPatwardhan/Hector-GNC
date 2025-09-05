#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ShowingImage(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/front_cam/camera/image", Image, self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):
        try:
            # We select bgr8 because it's the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow('image', cv_image)
        cv2.waitKey(10)  # Adjust the delay as needed

def main():
    rospy.init_node('line_following_node', anonymous=True)
    showing_image_object = ShowingImage()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
