#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def generate_uniform_points(num_points_x, num_points_y, num_points_z, distance, marker_pub):
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

    # Publish the points marker
    marker_pub.publish(points)

def publish_shape(height, radius, marker_pub):
    # Set the frame ID and timestamp
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()

    # Set the namespace and id for this marker
    marker.ns = "shapes"
    marker.id = 0

    # Set the marker type to MESH_RESOURCE
    marker.type = Marker.MESH_RESOURCE

    # Set the mesh resource to the absolute path of your mesh file
    marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"

    # Set the marker action
    marker.action = Marker.ADD

    # Set the pose of the marker (position and orientation)
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    # Set the scale of the marker based on user input
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.scale.z = height

    # Set the color of the marker
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    # Publish the marker
    marker_pub.publish(marker)

def main():
    rospy.init_node("combined_node")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # Adjust the rate as needed

    try:
        num_points_x = int(input("Enter the number of points in X dimension: "))
        num_points_y = int(input("Enter the number of points in Y dimension: "))
        num_points_z = int(input("Enter the number of points in Z dimension: "))
        distance = float(input("Enter the distance between points: "))
        height = float(input("Enter the height of the shape: "))
        radius = float(input("Enter the radius of the shape: "))
        
        while not rospy.is_shutdown():
            generate_uniform_points(num_points_x, num_points_y, num_points_z, distance, marker_pub)
            publish_shape(height, radius, marker_pub)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
