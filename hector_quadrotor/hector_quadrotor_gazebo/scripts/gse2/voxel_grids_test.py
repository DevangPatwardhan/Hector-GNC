#!/usr/bin/env python3

# Import the necessary libraries
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Initialize the ROS node
rospy.init_node('voxel_grid_visualizer')

# Create a publisher for the Marker
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Wait for a connection to RViz
rospy.sleep(1)

# Create a Marker message
marker = Marker()
marker.header.frame_id = "base_link"
marker.type = marker.CUBE_LIST
marker.action = marker.ADD
marker.scale.x = 0.1  # Voxel size in the x dimension
marker.scale.y = 0.1  # Voxel size in the y dimension
marker.scale.z = 0.1  # Voxel size in the z dimension

# Set the color of the voxels
marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green color

# Define the voxel grid dimensions
grid_size_x = 10
grid_size_y = 10
grid_size_z = 10

# Populate the Marker message with points representing the voxel grid
for x in range(grid_size_x):
    for y in range(grid_size_y):
        for z in range(grid_size_z):
            p = Point(x * 0.1, y * 0.1, z * 0.1)
            marker.points.append(p)

# Publish the Marker
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rospy.sleep(0.1)


