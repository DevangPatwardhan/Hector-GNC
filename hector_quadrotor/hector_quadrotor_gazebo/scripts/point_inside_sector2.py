#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import trimesh
import numpy as np

def is_inside_mesh(mesh, point):
    # Check if the given point is inside the mesh using ray tracing
    # Create a ray from the point in an arbitrary direction
    ray_origins = np.array([[point.x, point.y, point.z]])
    ray_directions = np.array([[0, 0, 1]])  # Z direction

    # Perform the ray-mesh intersection test
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_directions)

    # If the number of intersections is odd, the point is inside the mesh
    return len(locations) % 2 == 1

def generate_uniform_points(num_points_x, num_points_y, num_points_z, distance, mesh):
    # Create a marker for points
    points = Marker()
    points.header.frame_id = "base_link"
    points.header.stamp = rospy.Time.now()
    points.ns = "uniform_points"
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS
    points.scale.x = points.scale.y = 0.1  # Set point size to 1 cm
    points.color.g = points.color.a = 1.0  # Set points to green color

    # Create a marker for the mesh
    mesh_marker = Marker()
    mesh_marker.header.frame_id = "base_link"
    mesh_marker.header.stamp = rospy.Time.now()
    mesh_marker.ns = "mesh"
    mesh_marker.id = 1
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.action = Marker.ADD
    mesh_marker.pose.orientation.w = 1.0
    mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = 1
    mesh_marker.color.r = mesh_marker.color.g = mesh_marker.color.b = mesh_marker.color.a = 1.0  # Red color with full opacity
    mesh_marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"

    # Generate uniform points in a grid
    for x in range(num_points_x):
        for y in range(num_points_y):
            for z in range(num_points_z):
                p = Point()
                p.x = x * distance
                p.y = y * distance
                p.z = z * distance
                if is_inside_mesh(mesh, p):
                    points.points.append(p)

    # Publish the points and mesh markers
    marker_pub.publish(points)
    marker_pub.publish(mesh_marker)

if __name__ == "__main__":
    try:
        rospy.init_node("uniform_points_rviz")
        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        # Load the mesh from the file
        mesh_path = "/home/ros/stl/r_0.5_h-1.stl"
        mesh = trimesh.load(mesh_path, process=False)

        # Set the number of points and the distance between them
        num_points_x, num_points_y, num_points_z = 10, 10, 10  # Example values
        distance = 0.1  # Example value in meters

        # Call the function to generate points and visualize them
        generate_uniform_points(num_points_x, num_points_y, num_points_z, distance, mesh)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
