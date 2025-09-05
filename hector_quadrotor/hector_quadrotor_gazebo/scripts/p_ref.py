#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt


class ARDroneController:
    def __init__(self):
          # 10 Hz
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.pose_callback)
        self.current_pose = Point()
        self.goal_reached_threshold = 0.5  # Threshold distance to consider goal reached
        self.angular_velocity_scale = 1.0  # Scale factor for angular velocity control

    def pose_callback(self, data):
        

        
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        (_, _,self.current_z ) = tf.transformations.euler_from_quaternion(quaternion)

    def get_distance_to_goal(self, goal_x, goal_y):
        #current_x = self.current_pose.pose.position.x
        #current_y = self.current_pose.pose.position.y
        distance = sqrt((goal_x - self.current_x)**2 + (goal_y - self.current_y)**2)
        return distance

    def get_angle_to_goal(self, goal_x, goal_y):
        #current_x = self.current_pose.pose.position.x
        #current_y = self.current_pose.pose.position.y
        angle = atan2(goal_y - self.current_y, goal_x - self.current_x)
        return angle

    def move_to_goal(self, goal_x, goal_y):
        while not rospy.is_shutdown():
            distance = self.get_distance_to_goal(goal_x, goal_y)
            

            angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
            current_yaw = self.get_yaw_from_pose()

            # Calculate the difference in angles between current orientation and angle to goal
            angle_difference = angle_to_goal - current_yaw

            # Ensure the angle difference is within the range of -pi to pi
            if angle_difference > 3.14159:
                angle_difference -= 2 * 3.14159
            elif angle_difference < -3.14159:
                angle_difference += 2 * 3.14159

            # Apply proportional control to adjust angular velocity
            angular_velocity = self.angular_velocity_scale * angle_difference

            # Control the linear and angular velocities
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.5  # Constant linear velocity
            cmd_vel_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(cmd_vel_msg)

            if distance < self.goal_reached_threshold:
                cmd_vel_msg.linear.x = 0 # Constant linear velocity
                cmd_vel_msg.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rospy.loginfo("Reached the goal!")
                break

            self.rate.sleep()

    def get_yaw_from_pose(self):
        # This function retrieves the yaw (rotation around the z-axis) from the current pose
       
        return current_z  # Yaw is the third element of the euler angles

if __name__ == '__main__':
    try:
        rospy.init_node('ar_drone_controller')
        rate = rospy.Rate(10)
        controller = ARDroneController()
        goal_x = float(input("Enter the goal x coordinate: "))
        goal_y = float(input("Enter the goal y coordinate: "))
        while not rospy.is_shutdown():
            
            controller.move_to_goal(goal_x, goal_y)
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
