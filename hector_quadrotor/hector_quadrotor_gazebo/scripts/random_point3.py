#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def generate_two_points_and_connect():
    rospy.init_node("two_points_and_line_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    r = rospy.Rate(30)  # Publish rate (30 Hz)

    # Create a marker for points
    points = Marker()
    points.header.frame_id = "base_link"  # Replace with a valid frame ID
    points.header.stamp = rospy.Time.now()
    points.ns = "two_points"
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS
    points.scale.x, points.scale.y = 0.1, 0.1  # Set point size to 1 cm
    points.color.g, points.color.a = 1.0, 1.0  # Set points to green color

    # Generate two points
    for i in range(2):
        p = Point()
        p.x, p.y, p.z = i, i, i
        points.points.append(p)

    # Create a marker for the line
    line = Marker()
    line.header.frame_id = "base_link"  # Replace with a valid frame ID
    line.header.stamp = rospy.Time.now()
    line.ns = "line_between_points"
    line.action = Marker.ADD
    line.pose.orientation.w = 1.0
    line.id = 1
    line.type = Marker.LINE_STRIP
    line.scale.x = 0.01  # Set line width to 1 cm
    line.color.r, line.color.a = 1.0, 1.0  # Set line to red color
    line.points = points.points  # Use the same points for the line

    while not rospy.is_shutdown():
        marker_pub.publish(points)
        marker_pub.publish(line)
        r.sleep()

if __name__ == "__main__":
    try:
        generate_two_points_and_connect()
    except rospy.ROSInterruptException:
        pass
