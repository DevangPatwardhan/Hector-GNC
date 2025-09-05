#!/usr/bin/env python3



import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import sys

class MazebotGTG:
    def __init__(self):
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        self.robot_pose.z = data.pose.pose.position.z

    def goal_movement(self):
        self.goal_pose.x = float(input("Enter goal_pose.x ): "))
        self.goal_pose.y = float(input("Enter goal_pose.y ): "))
        #self.goal_pose.x =  3
        #self.goal_pose.y =  5

        self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x) ** 2 + (self.goal_pose.y - self.robot_pose.y) ** 2)
        self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
        print('1.DG:',format(self.distance_to_goal),'AG:',format(self.angle_to_goal))

        if self.distance_to_goal > 0.1:
            print('11.lv:',format(robot_pose.x),'AV:',format(robot_pose.z))
            vel_msg = Twist()
            vel_msg.linear.x = min(0.5, self.distance_to_goal)  # Max linear velocity set to 0.5 m/s
            vel_msg.angular.z = min(1.0, max(-1.0, self.angle_to_goal))  # Max angular velocity set to 1 rad/s
            self.velocity_publisher.publish(vel_msg)
            print('2.DG:',format(self.distance_to_goal),'AG:',format(self.angle_to_goal))
        else:
            # Stop the robot if it's close to the goal
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)
            print('3.DG:',format(self.distance_to_goal),'AG:',format(self.angle_to_goal))

def main(args=None):
    rospy.init_node('go_to_goal', anonymous=True)
    mazebot_gtg = MazebotGTG()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        mazebot_gtg.goal_movement()
        rate.sleep()

if __name__ == '__main__':
    main()

   

