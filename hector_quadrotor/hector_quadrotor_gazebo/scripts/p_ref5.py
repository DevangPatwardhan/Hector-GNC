#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import math

class MazebotGTG:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angular_velocity_scale = 0.5
        self.linear_velocity_scale = 0.5
        self.goal_reached_threshold = 0.1

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        self.robot_pose.z = data.pose.pose.position.z
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        (_, _, self.robot_pose_yaw) = self.euler_from_quaternion(*quaternion)

    def goal_movement(self):
        x = self.robot_pose.x
        y = self.robot_pose.y
        z = self.robot_pose.z

        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))
        self.goal_pose.z = float(input("Enter goal_pose.z: "))

        while not rospy.is_shutdown():
            K_linear = self.linear_velocity_scale
            K_angular = self.angular_velocity_scale

            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + 
                                         (self.goal_pose.y - self.robot_pose.y)**2 + 
                                         (self.goal_pose.z - self.robot_pose.z)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)

            current_yaw = self.robot_pose_yaw
            angle_difference = self.angle_to_goal - current_yaw

            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            angular_velocity = K_angular * angle_difference
            linear_speed = K_linear * self.distance_to_goal

            self.vel_msg.linear.x = 0.5  # linear_speed
            self.vel_msg.linear.z = K_linear * (self.goal_pose.z - self.robot_pose.z)
            self.vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
                self.vel_msg.linear.z = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal!")
                break

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rospy.init_node('go_to_goal', anonymous=True)
    mazebot_gtg = MazebotGTG()
    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        mazebot_gtg.goal_movement()
        rate.sleep()

if __name__ == '__main__':
    main()

