#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import math
import sys
from geometry_msgs.msg import PoseStamped
#from tf.transformations import euler_from_quaternion

class MazebotGTG:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.drone_pose_callback)
        self.goal_pose_subscriber = rospy.Subscriber('/goal_pose', PoseStamped, self.goal_pose_callback)
        self.drone_position = Point()
        self.goal_position = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_offset = 0
        self.angular_velocity_scale = 1.0
        self.goal_reached_threshold = 0.5    
        self.loop_rate = rospy.Rate(10)  # 10 Hz

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

    def goal_pose_callback(self, msg):
        self.goal_position = msg.pose.position

    def drone_pose_callback(self, msg):
        self.drone_position = msg.pose.position
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        #euler = euler_from_quaternion(quaternion)
        #self.drone_heading = euler[2]  # Yaw angle
        (_, _, self.drone_heading) = self.euler_from_quaternion(*quaternion)
    

    def calculate_angle_difference(self):
        if self.goal_position and self.drone_position:
            goal_angle = atan2(self.goal_position.y - self.drone_position.y, self.goal_position.x - self.drone_position.x)
            angle_difference = self.drone_heading - goal_angle
            return angle_difference

    def control_drone(self):
        angle_difference = self.calculate_angle_difference()
        if angle_difference:
            self.kp=0.5
            # Apply proportional control to adjust the drone's movement
            # You might need to adjust this control based on your specific requirements
            # For example, you may need to limit the range of angle_difference or consider angular wrap-around
            angular_velocity = -self.kp * angle_difference  # Negative sign to turn towards the goal
            # Publish control commands to control the drone's movement
            # This could involve publishing to topics such as /cmd_vel
            # Your actual implementation will depend on your ARDrone ROS packages
            rospy.loginfo("Angular Velocity Command: {}".format(angular_velocity))
            self.vel_msg.linear.x = 0.3  # Forward speed of the robot
            self.vel_msg.angular.z = angular_velocity * self.angular_velocity_scale
            self.velocity_publisher.publish(self.vel_msg)

            if is_close_enough_to_goal():
                rospy.loginfo("Goal reached!")
                return

        self.loop_rate.sleep()

def is_close_enough_to_goal(self):
    distance_to_goal = sqrt((self.drone_position.x - self.goal_position.x) ** 2 +
                             (self.drone_position.y - self.goal_position.y) ** 2)
    return abs(distance_to_goal - self.distance_to_goal) < self.goal_reached_threshold

if __name__ == '__main__':
    try:
        mazebot_gtg = MazebotGTG()
        
        while not rospy.is_shutdown():
            mazebot_gtg.control_drone()
    except Exception as e:
        print("Error in main loop")
        print(str(e))