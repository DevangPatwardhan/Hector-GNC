#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialize the ROS node
rospy.init_node('shape_publisher')

# Create a publisher object
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Function to get user input for height and radius
def get_user_input():
    height = float(input("Enter the height of the shape: "))
    radius = float(input("Enter the radius of the shape: "))
    return height, radius

# Get user input for height and radius
height, radius = get_user_input()

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

# Set the pose of the marker (position and orientation)
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0
marker.pose.orientation.y = 0
marker.pose.orientation.z = 0
marker.pose.orientation.w = 1.0

# Set the scale of the marker based on user input
marker.scale.x = radius * 2
marker.scale.y = radius * 2
marker.scale.z = height

# Set the color of the marker
marker.color.r = 0.5
marker.color.g = 0.5
marker.color.b = 0.5
marker.color.a = 1.0

# Publish the marker
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rospy.sleep(1)

