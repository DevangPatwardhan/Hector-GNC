#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def generate_uniform_points(num_points_x, num_points_y, num_points_z, distance):
    rospy.init_node("uniform_points_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    r = rospy.Rate(30)  # Publish rate (30 Hz)

    # Create a marker for points
    points = Marker()
    points.header.frame_id = "base_link"  # Replace with a valid frame ID
    points.header.stamp = rospy.Time.now()
    points.ns = "uniform_points"
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS
    points.scale.x, points.scale.y = 0.1, 0.1  # Set point size to 1 cm
    points.color.g, points.color.a = 1.0, 1.0  # Set points to green color

    # Calculate scale factor based on the distance between points
    scale_factor = distance

    # Generate uniform points in a num_points_x x num_points_y x num_points_z grid
    for x in range(num_points_x):
        for y in range(num_points_y):
            for z in range(num_points_z):
                p = Point()
                p.x, p.y, p.z = x * scale_factor, y * scale_factor, z * scale_factor
                points.points.append(p)

    # Get user input to remove a specific point
    try:
        remove_x = int(input("Enter the X coordinate of the point to remove: "))
        remove_y = int(input("Enter the Y coordinate of the point to remove: "))
        remove_z = int(input("Enter the Z coordinate of the point to remove: "))
        # Convert user input into index and remove the point
        remove_index = (remove_z * num_points_y + remove_y) * num_points_x + remove_x
        if remove_index < len(points.points):
            points.points.pop(remove_index)
        else:
            rospy.logwarn("Point to remove is out of range.")
    except ValueError:
        rospy.logerr("Invalid input. Please enter integer values.")

    while not rospy.is_shutdown():
        marker_pub.publish(points)
        r.sleep()

if __name__ == "__main__":
    try:
        num_points_x = int(input("Enter the number of points in X dimension: "))
        num_points_y = int(input("Enter the number of points in Y dimension: "))
        num_points_z = int(input("Enter the number of points in Z dimension: "))
        distance = float(input("Enter the distance between points (in meters): "))
        generate_uniform_points(num_points_x, num_points_y, num_points_z, distance)
    except rospy.ROSInterruptException:
        pass
