#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_two_points():
    rospy.init_node("two_points_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "two_points"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.scale.x = 0.1  # Point size
    points_marker.scale.y = 0.1
    points_marker.color.g = 1.0  # Green color
    points_marker.color.a = 1.0  # Alpha

    # Define the positions for the two points
    point1 = Point()
    point1.x = 1.0
    point1.y = 2.0
    point1.z = 0.0

    point2 = Point()
    point2.x = 1.5
    point2.y = 2.5
    point2.z = 0.0

    # Add the points to the marker
    points_marker.points.append(point1)
    points_marker.points.append(point2)

    # Publish the points marker
    while not rospy.is_shutdown():
        marker_pub.publish(points_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_two_points()
    except rospy.ROSInterruptException:
        pass
