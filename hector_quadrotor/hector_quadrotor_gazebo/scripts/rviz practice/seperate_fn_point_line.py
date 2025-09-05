#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def get_point_input(point_name):
    x = float(input(f"Enter {point_name} x-coordinate: "))
    y = float(input(f"Enter {point_name} y-coordinate: "))
    z = float(input(f"Enter {point_name} z-coordinate: "))
    return Point(x=x, y=y, z=z)

def publish_two_points(marker_pub, point1, point2):
    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"
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

    # Add the points to the marker
    points_marker.points.append(point1)
    points_marker.points.append(point2)

    # Publish the points marker
    marker_pub.publish(points_marker)

def publish_line_between_two_points(marker_pub, point1, point2):
    # Create a marker for the line
    line_marker = Marker()
    line_marker.header.frame_id = "base_link"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "line_between_two_points"
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.id = 1
    line_marker.type = Marker.LINE_STRIP
    line_marker.scale.x = 0.05  # Line width
    line_marker.color.r = 1.0  # Red color
    line_marker.color.g = 0.0
    line_marker.color.b = 0.0
    line_marker.color.a = 1.0  # Alpha

    # Add the points to the line marker
    line_marker.points.append(point1)
    line_marker.points.append(point2)

    # Publish the line marker
    marker_pub.publish(line_marker)

if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node("points_and_line_rviz", anonymous=True)
        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        # Get points from user input
        point1 = get_point_input("Point 1")
        point2 = get_point_input("Point 2")
        
        # Publish points and line
        publish_two_points(marker_pub, point1, point2)
        publish_line_between_two_points(marker_pub, point1, point2)

        # Keep the script alive
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
