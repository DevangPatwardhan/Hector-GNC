#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import heapq
import math

class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex, x, y, z, color):
        self.vertices[vertex] = {'position': (x, y, z), 'color': color}

    def add_edge(self, start_vertex, end_vertex, cost):
        self.edges.append((start_vertex, end_vertex, cost))

    def get_neighbors(self, vertex):
        neighbors = []
        for edge in self.edges:
            if edge[0] == vertex:
                neighbors.append((edge[1], edge[2]))  # (neighbor, cost)
            elif edge[1] == vertex:
                neighbors.append((edge[0], edge[2]))  # (neighbor, cost)
        return neighbors

def dijkstra(graph, start, goal):
    queue = [(0, start, [])]  # (cost, current_vertex, path)
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)
        if current not in visited:
            visited.add(current)
            path = path + [current]
            if current == goal:
                return path
            for neighbor, neighbor_cost in graph.get_neighbors(current):
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + neighbor_cost, neighbor, path))
    return None

def euclidean_distance(point1, point2):
    return math.sqrt(sum([(p1 - p2) ** 2 for p1, p2 in zip(point1, point2)]))

def publish_graph(g, start, goal, shortest_path):
    rospy.init_node("graph_rviz")
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    markers = MarkerArray()

    for idx, (vertex_name, data) in enumerate(g.vertices.items()):
        x, y, z = data['position']
        color = data['color']

        point_marker = Marker()
        point_marker.header.frame_id = "base_link"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "graph"
        point_marker.id = idx
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position = Point(x=x, y=y, z=z)
        point_marker.scale = Vector3(0.2, 0.2, 0.2)
        point_marker.color = color
        markers.markers.append(point_marker)

        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "graph"
        text_marker.id = idx + len(g.vertices)
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(x=x, y=y, z=z)
        text_marker.pose.orientation = Quaternion()
        text_marker.scale = Vector3(0.1, 0.1, 0.1)
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text_marker.text = vertex_name
        markers.markers.append(text_marker)

    lines_markers = MarkerArray()

    for idx, (start_vertex, end_vertex, _) in enumerate(g.edges):
        lines_marker = Marker()
        lines_marker.header.frame_id = "base_link"
        lines_marker.header.stamp = rospy.Time.now()
        lines_marker.ns = "graph"
        lines_marker.id = idx + 2 * len(g.vertices)
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.pose.orientation = Quaternion()
        lines_marker.scale = Vector3(0.05, 0.05, 0.05)
        lines_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)

        start_point = g.vertices[start_vertex]['position']
        end_point = g.vertices[end_vertex]['position']
        lines_marker.points.append(Point(x=start_point[0], y=start_point[1], z=start_point[2]))
        lines_marker.points.append(Point(x=end_point[0], y=end_point[1], z=end_point[2]))
        lines_markers.markers.append(lines_marker)

    path_marker = Marker()
    path_marker.header.frame_id = "base_link"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "graph"
    path_marker.id = 2 * len(g.vertices) + len(g.edges)
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation = Quaternion()
    path_marker.scale = Vector3(0.07, 0.07, 0.07)
    path_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    for vertex in shortest_path:
        x, y, z = g.vertices[vertex]['position']
        path_marker.points.append(Point(x=x, y=y, z=z))

    markers.markers.append(path_marker)
    
    marker_pub.publish(markers)
    marker_pub.publish(lines_markers)
    rate.sleep()

    #while not rospy.is_shutdown():
        #marker_pub.publish(markers)
        #marker_pub.publish(lines_markers)
        #rate.sleep()

if __name__ == "__main__":
    try:
        g = Graph()

        g.add_vertex('a', 0, 0, 0, ColorRGBA(1.0, 0.0, 0.0, 1.0))
        g.add_vertex('b', 1, 1, 0, ColorRGBA(0.0, 0.0, 1.0, 1.0))
        g.add_vertex('c', 2, 0, 0, ColorRGBA(0.0, 1.0, 0.0, 1.0))
        g.add_vertex('d', 1, -1, 0, ColorRGBA(1.0, 1.0, 0.0, 1.0))

        g.add_edge('a', 'b', 7)
        g.add_edge('b', 'c', 3)
        g.add_edge('c', 'd', 4)
        g.add_edge('d', 'a', 2)
        g.add_edge('a', 'c', 1)

        start = input("Enter start vertex: ")
        goal = input("Enter goal vertex: ")

        shortest_path = dijkstra(g, start, goal)
        if shortest_path:
            print("Shortest path:", shortest_path)
        else:
            print("No path found.")

        publish_graph(g, start, goal, shortest_path)

        # Calculate and print Euclidean distances between each pair of vertices
        print("\nEuclidean distances between vertices:")
        for vertex1 in g.vertices:
            for vertex2 in g.vertices:
                if vertex1 != vertex2:
                    distance = euclidean_distance(g.vertices[vertex1]['position'], g.vertices[vertex2]['position'])
                    print(f"Distance between {vertex1} and {vertex2}: {distance}")

    except rospy.ROSInterruptException:
        pass
