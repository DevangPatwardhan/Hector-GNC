#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt
import math

class MazebotGTG:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.obstacle_avoidance)
        
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angular_velocity_scale = 1
        self.goal_reached_threshold = 0.1  
        self.obstacle_threshold = 1.5  # Distance to consider an obstacle
        self.obstacle_detected = False

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y

    def obstacle_avoidance(self, data):
        regions = {
            'left': min(min(data.ranges[0:30]), 6),
            'mid': min(min(data.ranges[30:60]), 6),
            'right': min(min(data.ranges[60:90]), 6)
        }

        if min(regions.values()) < self.obstacle_threshold:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def move_to_goal(self):
        self.goal_pose.x = 10  # Goal X position
        self.goal_pose.y = 10  # Goal Y position

        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.8  # Rotate to avoid obstacle
            else:
                self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
                self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
                angle_difference = self.angle_to_goal - self.robot_pose.z

               # linear_speed = self.distance_to_goal * 0.5
                angular_velocity = self.angular_velocity_scale * angle_difference

                self.vel_msg.linear.x = 0.5#linear_speed
                self.vel_msg.angular.z = angular_velocity

            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                rospy.loginfo("Reached the goal!")
                break

def main():
    rospy.init_node('mazebot_navigation')
    mazebot = MazebotGTG()
    mazebot.move_to_goal()

    rospy.spin()

if __name__ == '__main__':
    main()
