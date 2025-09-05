#!/usr/bin/env python3

import rospy
import random
import math
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

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

        # Initialize robot's pose
        self.robot_pose = Point()

        # Initialize twist message for motion control
        self.twist_msg = TwistStamped()

        # Initialize motion command variables
        self.linear_speed = 1.0  # Adjust as needed
        self.angular_speed = 1.0  # Adjust as needed

    def pose_callback(self, msg):
        self.current_pose = msg

        # Update robot's pose
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        self.robot_pose.z = msg.pose.pose.position.z

    def generate_random_point(self):
        # Sensor position
        sensor_x, sensor_y, sensor_z = self.robot_pose.x, self.robot_pose.y, self.robot_pose.z

        # Random distance
        r = random.uniform(0.1, 5)

        # Random angles
        theta = random.uniform(-math.pi, math.pi)
        phi = random.uniform(-math.pi / 12, math.pi / 12)

        # Convert spherical to Cartesian coordinates
        x = r * math.cos(phi) * math.cos(theta)
        y = r * math.cos(phi) * math.sin(theta)
        #z = r * math.sin(phi)
        z = 0

        # Translate to sensor's position
        x_translated = x + sensor_x
        y_translated = y + sensor_y
        z_translated = z + sensor_z

        return x_translated, y_translated, z_translated

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust this frame ID according to your setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "random_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # fully opaque
        marker.color.r = 1.0  # red
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.pub_marker.publish(marker)

    def motion_control(self):
        rate = rospy.Rate(10)  # Publish commands at 10 Hz

        while not rospy.is_shutdown():
            command = input("Enter motion command (forward, backward, left, right, up, down, cw, ccw, stop, random): ")

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
            elif command == "random":
                x, y, z = self.generate_random_point()
                self.publish_marker(x, y, z)
                continue
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

