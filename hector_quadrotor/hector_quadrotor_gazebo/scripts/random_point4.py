#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math

class Node():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2 + (node1.z - node2.z)**2)

def get_random_node(x_max, y_max, z_max):
    x = random.uniform(0, x_max)
    y = random.uniform(0, y_max)
    z = random.uniform(0, z_max)
    return Node(x, y, z)

def get_nearest_node(rrt, random_node):
    nearest_node = rrt[0]
    min_dist = distance(rrt[0], random_node)
    for node in rrt:
        if distance(node, random_node) < min_dist:
            nearest_node = node
            min_dist = distance(node, random_node)
    return nearest_node

def add_node_to_rrt(rrt, nearest_node, random_node):
    new_node = Node(random_node.x, random_node.y, random_node.z)
    new_node.parent = nearest_node
    rrt.append(new_node)
    return new_node

def generate_rrt(start_x, start_y, start_z, dest_x, dest_y, dest_z, x_max=5, y_max=5, z_max=5):
    start_node = Node(start_x, start_y, start_z)
    dest_node = Node(dest_x, dest_y, dest_z)
    rrt = [start_node]

    while True:
        random_node = get_random_node(x_max, y_max, z_max)
        nearest_node = get_nearest_node(rrt, random_node)
        new_node = add_node_to_rrt(rrt, nearest_node, random_node)

        if distance(new_node, dest_node) < 0.1:  # If the new node is close enough to the destination
            dest_node.parent = new_node
            rrt.append(dest_node)
            break

    return rrt

if __name__ == "__main__":
    rospy.init_node("rrt_path_visualization")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    start_x = float(input("Enter start x: "))
    start_y = float(input("Enter start y: "))
    start_z = float(input("Enter start z: "))
    dest_x = float(input("Enter destination x: "))
    dest_y = float(input("Enter destination y: "))
    dest_z = float(input("Enter destination z: "))
    rrt = generate_rrt(start_x, start_y, start_z, dest_x, dest_y, dest_z)

    # Create a marker for the path
    path_marker = Marker()
    path_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "rrt_path"
    path_marker.action = Marker.ADD
    path_marker.pose.orientation.w = 1.0
    path_marker.id = 0
    path_marker.type = Marker.LINE_STRIP
    path_marker.scale.x = 0.01  # Set line width to 1 cm
    path_marker.color.r, path_marker.color.a = 1.0, 1.0  # Set line to red color

    # Add the nodes in the path to the marker
    node = rrt[-1]  # Start from the destination node
    while node is not None:
        p = Point()
        p.x, p.y, p.z = node.x, node.y, node.z
        path_marker.points.append(p)
        node = node.parent

    # Publish the path marker
    while not rospy.is_shutdown():
        marker_pub.publish(path_marker)
        rospy.sleep(1)
