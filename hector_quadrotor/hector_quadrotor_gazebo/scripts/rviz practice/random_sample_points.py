#!/usr/bin/env python3

import rospy
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Initialize the ROS node
rospy.init_node('random_points')

# Create a publisher on the visualization_marker topic
pub = rospy.Publisher('visualization_markerr', Marker, queue_size=10)

# Set the rate of publishing
rate = rospy.Rate(1)  # 1 Hz

while not rospy.is_shutdown():
    # Create a new marker of type POINTS
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.POINTS
    marker.action = marker.ADD

    # Set the scale of the points
    marker.scale.x = 0.2  # Width
    marker.scale.y = 0.2  # Height

    # Set the color to green
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Alpha (opacity)

    # Generate random points and add them to the marker
    for _ in range(10):  # Generate 10 random points
        p = Point()
        p.x = random.uniform(-5, 5)
        p.y = random.uniform(-5, 5)
        p.z = random.uniform(-5, 5)
        marker.points.append(p)

    # Publish the marker
    pub.publish(marker)

    # Sleep according to the set rate
    rate.sleep()
