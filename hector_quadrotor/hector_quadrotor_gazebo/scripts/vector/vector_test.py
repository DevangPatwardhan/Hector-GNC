#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, asin
import math


class TurtleBotGTG:
    def __init__(self, robot_name, node):
        self.robot_name = robot_name
        self.velocity_publisher = rospy.Publisher('/' + robot_name + '/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/' + robot_name + '/odom', Odometry, self.get_turtlebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.goal_reached_threshold = 0.1  
        self.angular_velocity_scale = 0.5

    def get_turtlebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.z) = self.euler_from_quaternion(*quaternion)

    def goal_movement(self):
        self.goal_pose.x = float(input("Enter goal_pose.x for " + self.robot_name + ": "))
        self.goal_pose.y = float(input("Enter goal_pose.y for " + self.robot_name + ": "))

        while not rospy.is_shutdown():
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)

            angle_difference = self.angle_to_goal - self.robot_pose.z

            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            linear_speed = self.distance_to_goal 
            angular_velocity = self.angular_velocity_scale * angle_difference

            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = angular_velocity

            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal for " + self.robot_name + "!")
                break

    def euler_from_quaternion(self, x, y, z, w):
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

if __name__ == '__main__':
    rospy.init_node('go_to_goal', anonymous=True)  # Initialize node once
    try:
        turtlebot1_gtg = TurtleBotGTG('turtlebot3', rospy)
        turtlebot1_gtg.goal_movement()

        turtlebot2_gtg = TurtleBotGTG('wheel', rospy)
        turtlebot2_gtg.goal_movement()
    except rospy.ROSInterruptException:
        pass
