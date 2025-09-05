#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialize the ROS node
rospy.init_node('shape_publisher')

# Create a publisher object
marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Set the frame ID and timestamp
marker = Marker()
marker.header.frame_id = "base_link"
marker.header.stamp = rospy.Time.now()

# Set the namespace and id for this marke
marker.ns = "shapes"
marker.id = 0

# Set the marker type to MESH_RESOURCE
marker.type = Marker.MESH_RESOURCE

# Set the mesh resource to a path of your mesh file
#marker.mesh_resource = "package://my_package/meshes/cone_with_hemisphere.stl"
# Set the mesh resource to the absolute path of your mesh file
marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"


# Set the marker action
marker.action = Marker.ADD

# Set the pose of the marker (position and orientation)
marker.pose.position.x = 1
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0
marker.pose.orientation.y = 0
marker.pose.orientation.z = 0
marker.pose.orientation.w = 1

# Set the scale of the marker
marker.scale.x = 1
marker.scale.y = 1
marker.scale.z = 1

# Set the color of the marker
marker.color.r = 0.5
marker.color.g = 0.5
marker.color.b = 0.5
marker.color.a = 1.0

# Publish the marker
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rospy.sleep(1)
