#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex, point):
        self.vertices[vertex] = point

    def add_edge(self, start_vertex, end_vertex, cost):
        self.edges.append((start_vertex, end_vertex, cost))

def publish_graph(g):
    rospy.init_node("graph_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "graph"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.scale.x = 0.1  # Point size
    points_marker.scale.y = 0.1
    points_marker.color.r = 0.0
    points_marker.color.g = 1.0  # Green color
    points_marker.color.b = 0.0
    points_marker.color.a = 1.0  # Alpha

    # Create a marker for the lines
    lines_marker = Marker()
    lines_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    lines_marker.header.stamp = rospy.Time.now()
    lines_marker.ns = "graph"
    lines_marker.action = Marker.ADD
    lines_marker.pose.orientation.w = 1.0
    lines_marker.id = 1
    lines_marker.type = Marker.LINE_LIST
    lines_marker.scale.x = 0.05  # Line width
    lines_marker.color.r = 0.0
    lines_marker.color.g = 0.0
    lines_marker.color.b = 1.0  # Blue color
    lines_marker.color.a = 1.0  # Alpha

    # Add the vertices to the points marker
    for vertex_name, vertex_point in g.vertices.items():
        points_marker.points.append(vertex_point)

    # Add the edges to the lines marker
    for start_vertex, end_vertex, _ in g.edges:
        start_point = g.vertices[start_vertex]
        end_point = g.vertices[end_vertex]
        lines_marker.points.append(start_point)
        lines_marker.points.append(end_point)

    # Publish the points and lines markers
    while not rospy.is_shutdown():
        marker_pub.publish(points_marker)
        marker_pub.publish(lines_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        g = Graph()

        # Define vertices with 3D coordinates
        vertices = {'A': Point(x=0, y=0, z=0), 'B': Point(x=1, y=1, z=0), 'C': Point(x=2, y=0, z=0), 'D': Point(x=1, y=-1, z=0)}

        # Define edges with associated costs
        edges = [('A', 'B', 5), ('B', 'C', 3), ('C', 'D', 4), ('D', 'A', 2), ('A', 'C', 1)]

        # Add vertices and edges to the graph
        for vertex, point in vertices.items():
            g.add_vertex(vertex, point)
        for edge in edges:
            g.add_edge(*edge)

        publish_graph(g)
    except rospy.ROSInterruptException:
        pass
