#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

def move_point_x():
    rospy.init_node("move_point_x_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    
    # Create a marker for the point
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    point_marker.ns = "moving_point"
    point_marker.action = Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = Marker.POINTS
    point_marker.scale.x = 0.1  # Point size
    point_marker.scale.y = 0.1
    point_marker.color.g = 1.0  # Green color
    point_marker.color.a = 1.0  # Alpha

    # Initial position of the point
    moving_point = Point()
    moving_point.x = 0.0
    moving_point.y = 2.0
    moving_point.z = 0.0

    # Add the point to the marker
    point_marker.points.append(moving_point)

    # Time to move the point
    start_time = time.time()
    duration = 5  # seconds

    while not rospy.is_shutdown() and (time.time() - start_time) < duration:
        # Update the x-coordinate of the point
        moving_point.x += 0.1  # Adjust this value to change the speed of the movement

        # Clear the previous points and add the updated point
        point_marker.points = []
        point_marker.points.append(moving_point)

        # Update the timestamp for the marker
        point_marker.header.stamp = rospy.Time.now()

        # Publish the updated marker
        marker_pub.publish(point_marker)

        # Sleep for a short time before the next update
        rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        move_point_x()
    except rospy.ROSInterruptException:
        pass
