#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
import heapq
import math

class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex, x, y, z, color):
        self.vertices[vertex] = (x, y, z, color)

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

def is_connected(graph, start, goal):
    visited = set()
    stack = [start]

    while stack:
        vertex = stack.pop()
        if vertex == goal:
            return True
        if vertex not in visited:
            visited.add(vertex)
            neighbors = graph.get_neighbors(vertex)
            for neighbor, _ in neighbors:
                if neighbor not in visited:
                    stack.append(neighbor)
    return False

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

def publish_graph(g, start, goal, shortest_path):
    #rospy.init_node("graph_rviz")
    #marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    #rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        update_markers(g, start, goal, shortest_path, marker_pub)
        rate.sleep()

def update_markers(g, start, goal, shortest_path, marker_pub):
    markers = MarkerArray()

    # Create markers for points
    for idx, (vertex_name, (x, y, z, color)) in enumerate(g.vertices.items()):
        # Create a marker for each vertex
        point_marker = Marker()
        point_marker.header.frame_id = "base_link"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "graph"
        point_marker.id = idx
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position = Point(x=x, y=y, z=z)
        point_marker.scale = Vector3(0.2, 0.2, 0.2)
        point_marker.color = color  # Use color from the vertex
        markers.markers.append(point_marker)

        # Create a marker for the text above the point
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "graph"
        text_marker.id = idx + len(g.vertices)  # Offset by number of vertices
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(x=x, y=y, z=z + 0.5)  # Adjust z coordinate to position above the point
        text_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
        text_marker.scale = Vector3(0.1, 0.1, 0.1)
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White color
        text_marker.text = vertex_name
        markers.markers.append(text_marker)

    # Create a marker array for the lines
    lines_markers = MarkerArray()

    # Add the edges to the lines markers
    for idx, (start_vertex, end_vertex, _) in enumerate(g.edges):
        lines_marker = Marker()
        lines_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
        lines_marker.header.stamp = rospy.Time.now()
        lines_marker.ns = "graph"
        lines_marker.id = idx + 2 * len(g.vertices)  # Offset by number of vertices
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
        lines_marker.scale = Vector3(0.05, 0.05, 0.05)  # Line width
        lines_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue color for edges

        start_point = g.vertices[start_vertex][:3]
        end_point = g.vertices[end_vertex][:3]
        lines_marker.points.append(Point(x=start_point[0], y=start_point[1], z=start_point[2]))
        lines_marker.points.append(Point(x=end_point[0], y=end_point[1], z=end_point[2]))

        lines_markers.markers.append(lines_marker)

    # Highlight the shortest path
    path_marker = Marker()
    path_marker.header.frame_id = "base_link"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "graph"
    path_marker.id = 2 * len(g.vertices) + len(g.edges)  # Offset by number of vertices and edges
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
    path_marker.scale = Vector3(0.07, 0.07, 0.07)  # Line width
    path_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color for path

    if shortest_path:
        for vertex in shortest_path:
            x, y, z, _ = g.vertices[vertex]
            path_marker.points.append(Point(x=x, y=y, z=z))
        markers.markers.append(path_marker)

    marker_pub.publish(markers)
    marker_pub.publish(lines_markers)

def euclidean_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

def print_distances(graph):
    for vertex1, (x1, y1, z1, _) in graph.vertices.items():
        for vertex2, (x2, y2, z2, _) in graph.vertices.items():
            if vertex1 != vertex2:
                distance = euclidean_distance((x1, y1, z1), (x2, y2, z2))
                print(f"Distance between {vertex1} and {vertex2}: {distance:.2f}")

def print_graph(graph):
    print("Vertices:")
    for vertex, (x, y, z, color) in graph.vertices.items():
        print(f"Vertex: {vertex}, Position: ({x}, {y}, {z}), Color: {color}")
    print("\nEdges:")
    for start_vertex, end_vertex, cost in graph.edges:
        print(f"Edge: {start_vertex} -> {end_vertex}, Cost: {cost}")

def add_vertex(graph,vertex, marker_pub, x_rand, y_rand, z_rand):
    #vertex_name = input("Enter vertex name: ")
    vertex_name = vertex
    x = x_rand
    y = y_rand
    z = z_rand
    r = 0
    g_value = 0
    b = 1
    a = 1
    color = ColorRGBA(r, g_value, b, a)
    graph.add_vertex(vertex_name, x, y, z, color)
    update_markers(graph, None, None, None, marker_pub)
    return vertex_name

def add_edge(graph, marker_pub, current_node, nearest_node):
    if len(graph.vertices) < 2:
        print("At least two vertices are required to add an edge.")
        return
    vertex1 = current_node
    vertex2 = nearest_node
    edge_cost = float(input("Enter edge cost: "))
    graph.add_edge(vertex1, vertex2, edge_cost)
    update_markers(graph, None, None, None, marker_pub)
    
def nearest(g, point):
    min_distance = float('inf')
    nearest_vertex = None

    for vertex, (x, y, z, _) in g.vertices.items():
        distance = euclidean_distance((x, y, z), point)
        if distance < min_distance:
            min_distance = distance
            nearest_vertex = vertex

    return nearest_vertex

if __name__ == "__main__":
    try:
        g = Graph()
        #global i
        i=0

        rospy.init_node("graph_rviz")
        marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        # Get the start and goal vertices
        start_vertex = input("Enter start vertex name: ")
        x = float(input("Enter x coordinate for start vertex: "))
        y = float(input("Enter y coordinate for start vertex: "))
        z = float(input("Enter z coordinate for start vertex: "))
        r = 0
        g_value = 0
        b = 1
        a = 1
        start_color = ColorRGBA(r, g_value, b, a)
        g.add_vertex(start_vertex, x, y, z, start_color)

        goal_vertex = input("Enter goal vertex name: ")
        x = float(input("Enter x coordinate for goal vertex: "))
        y = float(input("Enter y coordinate for goal vertex: "))
        z = float(input("Enter z coordinate for goal vertex: "))
        r = 0
        g_value = 0
        b = 1
        a = 1
        goal_color = ColorRGBA(r, g_value, b, a)
        g.add_vertex(goal_vertex, x, y, z, goal_color)

        while True:
            i=i+1
            if is_connected(g, start_vertex, goal_vertex):
                print("\nConnected")
                break

            print("\nNo path found. Please add more vertices and edges.")
            #point = (4, 4, 5)
            vertex_name = f"node{i}"
            print("current_vertex name:",vertex_name)
            x = float(input("Enter x coordinate for goal vertex: "))
            y = float(input("Enter y coordinate for goal vertex: "))
            z = float(input("Enter z coordinate for goal vertex: "))
            point=(x,y,z)
            nearest_vertex = nearest(g, point)
            print("Nearest vertex:", nearest_vertex)
            new_vertex = add_vertex(g,vertex_name, marker_pub, point[0], point[1], point[2])
            add_edge(g, marker_pub, new_vertex, nearest_vertex)

        print("\nGraph:")
        print_graph(g)

        print("\nEuclidean Distances:")
        print_distances(g)

        shortest_path = dijkstra(g, start_vertex, goal_vertex)
        if shortest_path:
            print("\nShortest path:", shortest_path)
        else:
            print("No path found.")

    except rospy.ROSInterruptException:
        pass
