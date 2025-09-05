#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex, x, y, z, color):
        point = Point(x=x, y=y, z=z)
        self.vertices[vertex] = (point, color)

    def add_edge(self, start_vertex, end_vertex, cost):
        self.edges.append((start_vertex, end_vertex, cost))

def publish_graph(g, start, goal, shortest_path):
    rospy.init_node("graph_rviz")
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    markers = MarkerArray()

    # Create markers for points and text
    for idx, (vertex_name, (vertex_point, vertex_color)) in enumerate(g.vertices.items()):
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
        point_marker.color = vertex_color  # Set the color attribute
        point_marker.color.a = 1.0

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
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = vertex_name

        markers.markers.append(point_marker)
        markers.markers.append(text_marker)

    # Create a marker array for the lines
    lines_markers = MarkerArray()

    # Add the edges to the lines markers
    for idx, (start_vertex, end_vertex, _) in enumerate(g.edges):
        lines_marker = Marker()
        lines_marker.header.frame_id = "base_link"
        lines_marker.header.stamp = rospy.Time.now()
        lines_marker.ns = "graph"
        lines_marker.id = idx
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.pose.orientation.w = 1.0
        lines_marker.scale.x = 0.05
        lines_marker.color.r = 0.0
        lines_marker.color.g = 0.0
        lines_marker.color.b = 1.0
        lines_marker.color.a = 1.0

        start_point = g.vertices[start_vertex][0]
        end_point = g.vertices[end_vertex][0]
        lines_marker.points.append(start_point)
        lines_marker.points.append(end_point)

        lines_markers.markers.append(lines_marker)

    # Create markers for the path
    path_markers = MarkerArray()
    if shortest_path:
        for i in range(len(shortest_path) - 1):
            start_point = g.vertices[shortest_path[i]][0]
            end_point = g.vertices[shortest_path[i + 1]][0]

            path_marker = Marker()
            path_marker.header.frame_id = "base_link"
            path_marker.header.stamp = rospy.Time.now()
            path_marker.ns = "graph"
            path_marker.id = i
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.1
            path_marker.color.r = 1.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0
            path_marker.points.append(start_point)
            path_marker.points.append(end_point)

            path_markers.markers.append(path_marker)

    while not rospy.is_shutdown():
        marker_pub.publish(markers)
        marker_pub.publish(lines_markers)
        marker_pub.publish(path_markers)
        rate.sleep()

if __name__ == "__main__":
    try:
        g = Graph()

        # Define vertices with 3D coordinates and colors
        g.add_vertex('a', 0, 0, 0, ColorRGBA(1.0, 0.0, 0.0, 1.0))  # Red color
        g.add_vertex('b', 1, 1, 0, ColorRGBA(0.0, 0.0, 1.0, 1.0))  # Blue color
        g.add_vertex('c', 2, 0, 0, ColorRGBA(0.0, 1.0, 0.0, 1.0))  # Green color
        g.add_vertex('d', 1, -1, 0, ColorRGBA(1.0, 1.0, 0.0, 1.0))  # Yellow color

        # Define edges with associated costs
        g.add_edge('a', 'b', 7)
        g.add_edge('b', 'c', 3)
        g.add_edge('c', 'd', 4)
        g.add_edge('d', 'a', 2)
        g.add_edge('a', 'c', 1)

        start = input("Enter start vertex: ")
        goal = input("Enter goal vertex: ")
        shortest_path = ['a', 'c', 'b']  # Example shortest path

        print("Shortest path:", shortest_path)

        publish_graph(g, start, goal, shortest_path)
    except rospy.ROSInterruptException:
        pass
