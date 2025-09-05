#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
import random

def generate_random_point(x, y, z, minx, maxx, miny, maxy, minz, maxz):
    # Generate a random point within the specified ranges
    random_x = random.uniform(minx, maxx)
    random_y = random.uniform(miny, maxy)
    random_z = random.uniform(minz, maxz)
    return x + random_x, y + random_y, z + random_z

def publish_marker(x, y, z):
    # Initialize the ROS node
    rospy.init_node('random_point_marker', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create the marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "random_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker
        marker.scale.x = 5
        marker.scale.y = 5
        marker.scale.z = 5

        # Set the color of the marker
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        # Given point
        user_x = 1.0
        user_y = 2.0
        user_z = 3.0

        # Specified ranges
        min_x = -1.0
        max_x = 1.0
        min_y = -1.0
        max_y = 1.0
        min_z = -1.0
        max_z = 1.0

        # Generate a random point
        rand_x, rand_y, rand_z = generate_random_point(user_x, user_y, user_z, min_x, max_x, min_y, max_y, min_z, max_z)
        
        # Publish the marker to RViz
        publish_marker(rand_x, rand_y, rand_z)

    except rospy.ROSInterruptException:
        pass
