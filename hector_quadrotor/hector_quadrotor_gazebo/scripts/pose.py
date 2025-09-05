#!/usr/bin/env /bin/python3

 

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    pose = msg.pose.pose.position
    x = pose.x
    y = pose.y
    z = pose.z
    rospy.loginfo("Drone Pose - X: {}, Y: {}, Z: {}".format(x, y, z))

def main():
    rospy.init_node('odom_subscriber', anonymous=True)
    rospy.Subscriber('/ground_truth/state', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

