#!/usr/bin/env python3

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialize the ROS node
rospy.init_node('shape_publisher')

# Create a publisher object
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Function to calculate Euclidean distance between two points
def euclidean_distance(point1, point2):
    return math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2 + (point2.z - point1.z) ** 2)

# Function to set orientation of the marker based on start and goal points
def set_marker_orientation(marker, start_point, goal_point):
    delta_x = goal_point.x - start_point.x
    delta_y = goal_point.y - start_point.y
    delta_z = goal_point.z - start_point.z

    # Calculate yaw angle
    yaw = math.atan2(delta_y, delta_x)

    # Calculate pitch angle
    pitch = math.atan2(delta_z, math.sqrt(delta_x ** 2 + delta_y ** 2))

    # Set orientation
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = math.sin(pitch / 2)
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = math.cos(pitch / 2 + yaw)

# Get start and goal points from user input
start_point = Point()
goal_point = Point()

start_point.x = float(input("Enter start point x-coordinate: "))
start_point.y = float(input("Enter start point y-coordinate: "))
start_point.z = float(input("Enter start point z-coordinate: "))

goal_point.x = float(input("Enter goal point x-coordinate: "))
goal_point.y = float(input("Enter goal point y-coordinate: "))
goal_point.z = float(input("Enter goal point z-coordinate: "))

# Set the frame ID and timestamp
marker = Marker()
marker.header.frame_id = "base_link"
marker.header.stamp = rospy.Time.now()

# Set the namespace and id for this marker
marker.ns = "shapes"
marker.id = 0

# Set the marker type to MESH_RESOURCE
marker.type = Marker.MESH_RESOURCE

# Set the mesh resource to the absolute path of your mesh file
marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"

# Set the marker action
marker.action = Marker.ADD

# Set the scale of the marker
marker.scale.x = 1
marker.scale.y = 1
#marker.scale.z = euclidean_distance(start_point, goal_point)
marker.scale.z = 1

# Set the color of the marker
marker.color.r = 0.5
marker.color.g = 0.5
marker.color.b = 0.5
marker.color.a = 1.0

# Set marker orientation based on start and goal points
set_marker_orientation(marker, start_point, goal_point)

# Publish the marker
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rospy.sleep(1)
