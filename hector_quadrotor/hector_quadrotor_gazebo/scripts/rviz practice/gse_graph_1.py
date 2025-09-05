#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
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
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    markers = MarkerArray()

    # Create markers for points and text
    for idx, (vertex_name, vertex_point) in enumerate(g.vertices.items()):
        # Green point marker
        point_marker = Marker()
        point_marker.header.frame_id = "base_link"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "graph"
        point_marker.id = idx * 2
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position = vertex_point
        point_marker.scale.x = 0.2
        point_marker.scale.y = 0.2
        point_marker.scale.z = 0.2
        point_marker.color.r = 0.0
        point_marker.color.g = 1.0
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0

        # Red text marker for vertex name
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "graph"
        text_marker.id = idx * 2 + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = vertex_point
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = vertex_name

        markers.markers.append(point_marker)
        markers.markers.append(text_marker)

    # Create a marker array for the lines
    lines_markers = MarkerArray()

    # Add the edges to the lines markers
    for idx, (start_vertex, end_vertex, _) in enumerate(g.edges):
        lines_marker = Marker()
        lines_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
        lines_marker.header.stamp = rospy.Time.now()
        lines_marker.ns = "graph"
        lines_marker.id = idx
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.pose.orientation.w = 1.0
        lines_marker.scale.x = 0.05  # Line width
        lines_marker.color.r = 0.0
        lines_marker.color.g = 0.0
        lines_marker.color.b = 1.0  # Blue color
        lines_marker.color.a = 1.0  # Alpha

        start_point = g.vertices[start_vertex]
        end_point = g.vertices[end_vertex]
        lines_marker.points.append(start_point)
        lines_marker.points.append(end_point)

        lines_markers.markers.append(lines_marker)

    while not rospy.is_shutdown():
        marker_pub.publish(markers)
        marker_pub.publish(lines_markers)
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
