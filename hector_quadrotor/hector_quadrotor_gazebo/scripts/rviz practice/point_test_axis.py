#!/usr/bin/env python3

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA


def publish_points(point, point2, marker_pub):
    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "points"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.scale.x = 0.1  # Point size
    points_marker.scale.y = 0.1
    points_marker.color.g = 1.0  # Green color
    points_marker.color.a = 1.0  # Alpha

    # Add the points to the marker
    points_marker.points.append(point)
    points_marker.points.append(point2)

    # Publish the points marker
    marker_pub.publish(points_marker)


def publish_axes(marker_pub):
    # Create a marker for the axes
    axes_marker = Marker()
    axes_marker.header.frame_id = "base_link"
    axes_marker.header.stamp = rospy.Time.now()
    axes_marker.ns = "axes"
    axes_marker.action = Marker.ADD
    axes_marker.pose.orientation.w = 1.0
    axes_marker.id = 0
    axes_marker.type = Marker.LINE_LIST
    axes_marker.scale.x = 0.02  # Line width
    axes_marker.color.a = 1.0  # Alpha

    # Define colors for x, y, z axes
    red = ColorRGBA(1.0, 0.0, 0.0, 1.0)    # Red for x-axis
    green = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green for y-axis
    blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)   # Blue for z-axis

    # Add x, y, z axes
    origin = Point()
    end_x = Point(x=1.0)
    end_y = Point(y=1.0)
    end_z = Point(z=1.0)

    axes_marker.points.extend([origin, end_x, origin, end_y, origin, end_z])

    # Assign colors to axes
    axes_marker.colors.extend([red, red, green, green, blue, blue])

    # Publish the axes marker
    marker_pub.publish(axes_marker)


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('point_plotter')

    # Create a publisher object for markers
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Get coordinates of two points from the user
    point_coords = [float(coord) for coord in
                    input("Enter coordinates of point 1 (comma separated x, y, z): ").split(',')]
    point2_coords = [float(coord) for coord in
                     input("Enter coordinates of point 2 (comma separated x, y, z): ").split(',')]

    # Extract x, y, z coordinates from the input
    point = Point()
    point.x = point_coords[0]
    point.y = point_coords[1]
    point.z = point_coords[2]

    point2 = Point()
    point2.x = point2_coords[0]
    point2.y = point2_coords[1]
    point2.z = point2_coords[2]

    # Publish the points
    publish_points(point, point2, marker_pub)

    # Publish the coordinate axes
    publish_axes(marker_pub)

    rospy.spin()
