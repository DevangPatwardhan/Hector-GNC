#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math
import random

class DroneMotionControl:
    def __init__(self):
        rospy.init_node('drone_motion_control', anonymous=True)

        # Publishers
        self.pub_twist = rospy.Publisher('/command/twist', TwistStamped, queue_size=10)
        self.pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Subscriber
        self.sub_pose = rospy.Subscriber('/ground_truth/state', Odometry, self.pose_callback)

        # Current pose of the drone
        self.current_pose = Odometry()

        # Initialize twist message for motion control
        self.twist_msg = TwistStamped()

        # Initialize motion command variables
        self.linear_speed = 1.0  # Adjust as needed
        self.angular_speed = 1.0  # Adjust as needed

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_marker(self, position, marker_id):
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust this frame ID according to your setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "random_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # fully opaque
        marker.color.r = 0.0
        marker.color.g = 1.0  # green
        marker.color.b = 0.0

        self.pub_marker.publish(marker)

    def generate_random_point(self):
        random_point = Point()
        random_point.x = self.current_pose.pose.pose.position.x + random.uniform(-5.0, 5.0)
        random_point.y = self.current_pose.pose.pose.position.y + random.uniform(-5.0, 5.0)
        random_point.z = self.current_pose.pose.pose.position.z + random.uniform(-5.0, 5.0)
        return random_point

    def generate_random_points(self, num_points):
        random_points = []
        for _ in range(num_points):
            random_point = self.generate_random_point()
            random_points.append(random_point)
        return random_points

    def publish_random_points_markers(self, random_points):
        for idx, point in enumerate(random_points):
            self.publish_marker(point, idx)

    def motion_control(self):
        rate = rospy.Rate(10)  # Publish commands at 10 Hz

        while not rospy.is_shutdown():
            command = input("Enter motion command (forward, backward, left, right, up, down, cw, ccw, stop, random_points): ")

            if command == "stop":
                self.twist_msg.twist.linear.x = 0.0
                self.twist_msg.twist.linear.y = 0.0
                self.twist_msg.twist.linear.z = 0.0
                self.twist_msg.twist.angular.z = 0.0
            elif command == "forward":
                self.twist_msg.twist.linear.x = self.linear_speed
            elif command == "backward":
                self.twist_msg.twist.linear.x = -self.linear_speed
            elif command == "left":
                self.twist_msg.twist.linear.y = self.linear_speed
            elif command == "right":
                self.twist_msg.twist.linear.y = -self.linear_speed
            elif command == "up":
                self.twist_msg.twist.linear.z = self.linear_speed
            elif command == "down":
                self.twist_msg.twist.linear.z = -self.linear_speed
            elif command == "cw":  # Clockwise rotation
                self.twist_msg.twist.angular.z = self.angular_speed
            elif command == "ccw":  # Counterclockwise rotation
                self.twist_msg.twist.angular.z = -self.angular_speed
            elif command == "random_points":
                num_points = int(input("Enter number of random points to generate: "))
                random_points = self.generate_random_points(num_points)
                self.publish_random_points_markers(random_points)
            else:
                rospy.logwarn("Invalid command. Please enter a valid command.")
                continue

            self.twist_msg.header.stamp = rospy.Time.now()
            self.pub_twist.publish(self.twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneMotionControl()
        controller.motion_control()
    except rospy.ROSInterruptException:
        pass

