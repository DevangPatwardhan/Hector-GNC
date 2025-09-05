#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return x, y, z, w

def calculate_angles(point, point2):
    # Calculate the vector components
    vector_x = point2.x - point.x
    vector_y = point2.y - point.y
    vector_z = point2.z - point.z

    # Calculate the magnitude of the vector
    magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)

    # Calculate angles with x, y, z axes
    theta_x = math.atan2(vector_y, vector_z)
    theta_y = math.atan2(-vector_x, math.sqrt(vector_y**2 + vector_z**2))
    theta_z = math.atan2(math.sin(theta_x) * vector_x + math.cos(theta_x) * vector_y, vector_z)

    return theta_x, theta_y, theta_z

def euclidean_distance(point, point2):
    # Calculate the vector components
    vector_x = point2.x - point.x
    vector_y = point2.y - point.y
    vector_z = point2.z - point.z

    # Calculate the magnitude of the vector
    magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)

    

    return magnitude

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('shape_publisher')

    # Create a publisher object
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Get coordinates of two points from the user
    point_coords = [float(coord) for coord in
                    input("Enter coordinates of point 1 (comma separated x, y, z): ").split(',')]
    point2_coords = [float(coord) for coord in
                     input("Enter coordinates of point 2 (comma separated x, y, z): ").split(',')]

    # Extract x, y, z coordinates from the input
    point = Point()
    point.x = point_coords[0]
    point.y = point_coords[1]
    point.z = point_coords[2]

    point2 = Point()
    point2.x = point2_coords[0]
    point2.y = point2_coords[1]
    point2.z = point2_coords[2]

    theta_x, theta_y, theta_z = calculate_angles(point, point2)
    
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = euler_to_quaternion(theta_x, theta_y, theta_z)

    # Create a marker object
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "shapes"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"
    marker.action = Marker.ADD
    marker.pose.position = point  # Starting point
    marker.pose.orientation.x = quaternion_x
    marker.pose.orientation.y = quaternion_y
    marker.pose.orientation.z = quaternion_z
    marker.pose.orientation.w = -quaternion_w
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = euclidean_distance(point, point2)-0.5
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    # Publish the marker
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rospy.sleep(1)
