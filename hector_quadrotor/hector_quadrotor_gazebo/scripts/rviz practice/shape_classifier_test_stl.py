#!/usr/bin/env python3

import rospy
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def angle_between_vectors(X, Pi_X):
    dot_product = np.dot(X, Pi_X)
    cos_angle = dot_product / (np.linalg.norm(X) * np.linalg.norm(Pi_X))
    return np.arccos(np.clip(cos_angle, -1.0, 1.0))

def calculate_distance(X, Pi_X):
    distance = np.linalg.norm(X - Pi_X)  # Euclidean distance
    return distance

def construct_normal_vector(X, Pi_X):
    v = Pi_X - X
    v_magnitude = np.linalg.norm(v)
    u = v / v_magnitude
    n_i_X = np.array([u[1], u[2], -u[0]])  # Permute components to construct normal vector
    return n_i_X

def calculate_ri(X, P_o, ni_X, theta_i):
    V_i_X = P_o - X
    dot_product = np.dot(ni_X, V_i_X)
    magnitude_ni_X = np.linalg.norm(ni_X)
    magnitude_V_i_X = np.linalg.norm(V_i_X)
    angle = np.arccos(dot_product / (magnitude_ni_X * magnitude_V_i_X))
    ri = angle - theta_i
    if ri < 0:
        print("Point Pi_X lies within the angle theta_i")
    else:
        print("Point Pi_X lies outside the angle theta_i")
    return ri

def set_marker_orientation(marker, start_point, goal_point):
    direction = np.array([goal_point.x - start_point.x, 
                          goal_point.y - start_point.y, 
                          goal_point.z - start_point.z])
    direction /= np.linalg.norm(direction)  # Normalize direction vector
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    if np.any(direction != 0):
        marker.pose.orientation.x = direction[0]
        marker.pose.orientation.y = direction[1]
        marker.pose.orientation.z = direction[2]
        marker.pose.orientation.w = 1 + np.dot([0, 0, 1], direction)

def publish_shape_marker(start_point, goal_point, mesh_resource, marker_pub):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "shapes"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = mesh_resource
    marker.action = Marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = np.linalg.norm([goal_point.x - start_point.x, 
                                     goal_point.y - start_point.y, 
                                     goal_point.z - start_point.z])
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    set_marker_orientation(marker, start_point, goal_point)
    marker_pub.publish(marker)

def publish_points(point1, point2, point3, marker_pub):
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"
    point_marker.header.stamp = rospy.Time.now()
    point_marker.ns = "points"
    point_marker.action = Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = Marker.POINTS
    point_marker.scale.x = 0.1
    point_marker.scale.y = 0.1
    point_marker.color.g = 1.0
    point_marker.color.a = 1.0
    point_marker.points.append(point1)
    point_marker.points.append(point2)
    point_marker.points.append(point3)
    marker_pub.publish(point_marker)

if __name__ == "__main__":
    rospy.init_node('shape_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    X_coords = [float(coord) for coord in input("Enter coordinates of point X (comma separated x, y, z): ").split(',')]
    start_point = Point(x=X_coords[0], y=X_coords[1], z=X_coords[2])
    Pi_X_coords = [float(coord) for coord in input("Enter coordinates of point Pi_X (comma separated x, y, z): ").split(',')]
    goal_point = Point(x=Pi_X_coords[0], y=Pi_X_coords[1], z=Pi_X_coords[2])
    P_o_coords = [float(coord) for coord in input("Enter coordinates of point P_o (comma separated x, y, z): ").split(',')]
    p_point = Point(x=P_o_coords[0], y=P_o_coords[1], z=P_o_coords[2])
    X = np.array(X_coords)
    Pi_X = np.array(Pi_X_coords)
    ri_X = calculate_distance(X, Pi_X)
    print("Minimum distance from point X to obstacle i (ri_X):", ri_X)
    n_i_X = construct_normal_vector(X, Pi_X)
    print("Normal vector n_i;X:", n_i_X)
    theta_i = 1.74533  # Set theta_i
    ri = calculate_ri(X, np.array(P_o_coords), n_i_X, theta_i)
    print("ri:", ri)
    publish_points(start_point, goal_point, p_point, marker_pub)
    mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"
    publish_shape_marker(start_point, goal_point, mesh_resource, marker_pub)
