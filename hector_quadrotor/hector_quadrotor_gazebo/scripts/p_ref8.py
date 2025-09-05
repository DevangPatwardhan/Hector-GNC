#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi, degrees
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
        self.angular_velocity_scale = 1
        self.linear_velocity_scale = 1
        self.goal_reached_threshold = 0.1  

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.z) = self.euler_from_quaternion(*quaternion)

    def return_pose(self):
    
        robot_x=self.robot_pose.x
        robot_y=self.robot_pose.y
        robot_yaw=self.robot_pose.z
        return robot_x,robot_y,robot_yaw

    def goal_movement(self,goal_x,goal_y):
        self.goal_pose.x = goal_x
        self.goal_pose.y = goal_y

        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            K_linear = self.linear_velocity_scale

            # Calculate distance to goal
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            # Calculate angle to goal
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
            
            current_yaw = self.robot_pose.z
            angle_difference = self.angle_to_goal - current_yaw

            # Normalize angle difference to be within -pi to pi
            if angle_difference > pi:
                angle_difference -= 2 * pi
            elif angle_difference < -pi:
                angle_difference += 2 * pi

            angular_velocity = self.angular_velocity_scale * angle_difference

            # First align the drone to the goal direction
            if abs(angle_difference) > 0.1:  # Some small threshold to start moving
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = angular_velocity
            else:
                # Move towards the goal
                self.vel_msg.linear.x = 0.5  # Constant linear velocity
                self.vel_msg.angular.z = 0  # Stop rotating

            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
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
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rospy.init_node('go_to_goal', anonymous=True)
    mazebot_gtg = MazebotGTG()
    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        x = float(input("Enter goal_pose.x: "))
        y = float(input("Enter goal_pose.y: "))
        mazebot_gtg.goal_movement(x,y)
        pos_x,pos_y,pos_yaw=mazebot_gtg.return_pose()
        print("pos_x",pos_x)
        print("pos_y",pos_y)
        print("pos_yaw",pos_yaw)
        rate.sleep()

if __name__ == '__main__':
    main()

