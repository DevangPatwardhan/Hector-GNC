#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import math
import sys

class MazebotGTG:
    def __init__(self):
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angle_offset = 0
        self.angular_velocity_scale = 1.0
        self.goal_reached_threshold = 0.5  

    def get_mazebot_pose(self, data):
        
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.z) = self.euler_from_quaternion(*quaternion)

    def goal_movement(self):
        global x
        global y, z

        x = self.robot_pose.x
        y = self.robot_pose.y
        z = self.robot_pose.z

        self.goal_pose.x = float(input("Enter goal_pose.x ): "))
        self.goal_pose.y = float(input("Enter goal_pose.y ): "))
        #self.angle_offset = float(input("Enter angle_offset ): "))

        while not rospy.is_shutdown():
            

            #self.distance_to_goal = sqrt(pow((self.goal_pose.x - self.robot_pose.x), 2) + pow((self.goal_pose.y - self.robot_pose.y), 2))
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            #self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x) + self.angle_offset
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
            current_yaw = self.robot_pose.z
            angle_difference = self.angle_to_goal - current_yaw
            #self.angle_to_turn = self.angle_to_goal - self.robot_pose.z
            print('1.DG:',format(self.distance_to_goal),'AG:',format(self.angle_to_goal))

            # Ensure the angle difference is within the range of -pi to pi
            if angle_difference > 3.14159:
                
                
                angle_difference -= 2 * 3.14159
            elif angle_difference < -3.14159:
                
                
                angle_difference += 2 * 3.14159
        
            angular_velocity = self.angular_velocity_scale * angle_difference

        
            self.vel_msg.linear.x = 0.5
            self.vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                
                

                self.vel_msg.linear.x = 0 # Constant linear velocity
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                print ('x=', format(self.robot_pose.x), 'y=',format(self.robot_pose.y),"yaw",format(self.robot_pose.z))
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
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        mazebot_gtg.goal_movement()
        print ("pose callback:")
        print ('init x = {}'.format(x)) 
        print ('init y = {}'.format(y))
        print ('init yaw = {}'.format(z))

        rate.sleep()

if __name__ == '__main__':
    main()

