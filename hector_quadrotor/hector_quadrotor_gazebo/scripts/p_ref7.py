#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, sin, cos
import numpy as np

class MazebotGTG:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
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
        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))
        self.goal_pose.z = float(input("Enter goal_pose.z: "))

        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            K_linear = self.linear_velocity_scale
            K_angular = self.angular_velocity_scale

            # Calculate distance to goal
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + 
                                         (self.goal_pose.y - self.robot_pose.y)**2 + 
                                         (self.goal_pose.z - self.robot_pose.z)**2)

            # Calculate azimuth to goal
            self.azimuth_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)

            current_yaw = self.robot_pose_yaw
            angle_difference = self.azimuth_to_goal - current_yaw

            if angle_difference > pi:
                angle_difference -= 2 * pi
            elif angle_difference < -pi:
                angle_difference += 2 * pi

            angular_velocity = K_angular * angle_difference

            # First align the drone to the goal direction
            if abs(angle_difference) > 0.1:  # Some small threshold to start moving
                self.vel_msg.linear.x = 0
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0
                self.vel_msg.angular.z = angular_velocity
            else:
                # Calculate direction vector
                direction_vector = np.array([
                    self.goal_pose.x - self.robot_pose.x,
                    self.goal_pose.y - self.robot_pose.y,
                    self.goal_pose.z - self.robot_pose.z
                ])
                direction_norm = np.linalg.norm(direction_vector)
                if direction_norm > 0:
                    direction_unit_vector = direction_vector / direction_norm
                else:
                    direction_unit_vector = direction_vector

                # Calculate velocities
                linear_speed = K_linear * direction_norm
                self.vel_msg.linear.x = linear_speed * direction_unit_vector[0]
                self.vel_msg.linear.y = linear_speed * direction_unit_vector[1]
                self.vel_msg.linear.z = linear_speed * direction_unit_vector[2]
                self.vel_msg.angular.z = 0  # Stop rotating

            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal!")
                break

            rate.sleep()

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

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

