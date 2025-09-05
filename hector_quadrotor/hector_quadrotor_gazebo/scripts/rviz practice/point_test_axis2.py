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
    points_marker.type = Marker.LINE_STRIP
    points_marker.scale.x = 0.05  # Line width
    points_marker.color.r = 1.0  # Red color
    points_marker.color.a = 1.0  # Alpha

    # Set pink color
    points_marker.color.r = 1.0
    points_marker.color.g = 0.0
    points_marker.color.b = 1.0

    # Add the points to the marker
    points_marker.points.append(point)
    points_marker.points.append(point2)

    # Publish the points marker
    marker_pub.publish(points_marker)

    # Create markers for the points
    point_marker1 = Marker()
    point_marker1.header.frame_id = "base_link"
    point_marker1.header.stamp = rospy.Time.now()
    point_marker1.ns = "points"
    point_marker1.action = Marker.ADD
    point_marker1.pose.orientation.w = 1.0
    point_marker1.id = 1
    point_marker1.type = Marker.SPHERE
    point_marker1.scale.x = 0.1  # Sphere size
    point_marker1.scale.y = 0.1
    point_marker1.scale.z = 0.1
    point_marker1.color.r = 0.0  # Red color
    point_marker1.color.g = 1.0  # Green color
    point_marker1.color.b = 1.0  # Blue color
    point_marker1.color.a = 1.0  # Alpha
    point_marker1.pose.position = point

    point_marker2 = Marker()
    point_marker2.header.frame_id = "base_link"
    point_marker2.header.stamp = rospy.Time.now()
    point_marker2.ns = "points"
    point_marker2.action = Marker.ADD
    point_marker2.pose.orientation.w = 1.0
    point_marker2.id = 2
    point_marker2.type = Marker.SPHERE
    point_marker2.scale.x = 0.1  # Sphere size
    point_marker2.scale.y = 0.1
    point_marker2.scale.z = 0.1
    point_marker2.color.r = 0.0  # Red color
    point_marker2.color.g = 1.0  # Green color
    point_marker2.color.b = 1.0  # Blue color
    point_marker2.color.a = 1.0  # Alpha
    point_marker2.pose.position = point2

    # Publish the point markers
    marker_pub.publish(point_marker1)
    marker_pub.publish(point_marker2)

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

def calculate_angles(point, point2):
    # Calculate the vector components
    vector_x = point2.x - point.x
    vector_y = point2.y - point.y
    vector_z = point2.z - point.z

    # Calculate the magnitude of the vector
    magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)

    # Calculate angles with x, y, z axes
    theta_x = math.degrees(math.acos(vector_x / magnitude))
    theta_y = math.degrees(math.acos(vector_y / magnitude))
    theta_z = math.degrees(math.acos(vector_z / magnitude))

    return theta_x, theta_y, theta_z

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

    # Calculate angles with x, y, z axes
    theta_x, theta_y, theta_z = calculate_angles(point, point2)
    print("Angle with x-axis: {:.2f} degrees".format(theta_x))
    print("Angle with y-axis: {:.2f} degrees".format(theta_y))
    print("Angle with z-axis: {:.2f} degrees".format(theta_z))

    rospy.spin()

