#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random

def generate_sample_points(min_x, max_x, min_y, max_y, min_z, max_z, num_points):
    points = []
    for _ in range(num_points):
        point = Point()
        point.x = random.uniform(min_x, max_x)
        point.y = random.uniform(min_y, max_y)
        point.z = random.uniform(min_z, max_z)
        points.append(point)
    return points

def visualize_points(points, color):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "sample_points"
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.id = 0
    marker.type = Marker.POINTS
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.points = points
    return marker

if __name__ == "__main__":
    rospy.init_node("sample_point_generator")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    r = rospy.Rate(10)

    # Define the workspace dimensions
    min_x, max_x = -5, 5
    min_y, max_y = -5, 5
    min_z, max_z = 0, 5

    # Generate sample points within the workspace
    num_points = 20
    sample_points = generate_sample_points(min_x, max_x, min_y, max_y, min_z, max_z, num_points)

    # Choose a different color (e.g., yellow)
    color = [1.0, 1.0, 0.0, 1.0]  # Yellow: RGB values (1, 1, 0); alpha = 1.0 (fully opaque)

    # Visualize the sample points in RViz with the chosen color
    sample_points_marker = visualize_points(sample_points, color)

    while not rospy.is_shutdown():
        marker_pub.publish(sample_points_marker)
        r.sleep()
