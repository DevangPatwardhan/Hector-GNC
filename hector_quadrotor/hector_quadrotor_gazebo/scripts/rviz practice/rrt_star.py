#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math


def get_point_input(point_name):
    x = float(input(f"Enter {point_name} x-coordinate: "))
    y = float(input(f"Enter {point_name} y-coordinate: "))
    z = float(input(f"Enter {point_name} z-coordinate: "))
    return Point(x=x, y=y, z=z)


# Define the RRT* algorithm functions here
class RRTStar:
    class Node:
        """Represents a node in the RRT* tree"""
        def __init__(self, point):
            self.point = point
            self.parent = None
            self.cost = 0.0  # Cost to reach this node

    def __init__(self, start, goal, search_space, step_size, max_iter, radius):
        self.start = self.Node(start)
        self.goal = self.Node(goal)
        self.search_space = search_space
        self.step_size = step_size
        self.max_iter = max_iter
        self.radius = radius
        self.tree = []

    def distance(self, point1, point2):
        p1 = (point1.x, point1.y, point1.z)
        p2 = (point2.x, point2.y, point2.z)
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def nearest_node(self, sample):
        nearest = self.tree[0]
        for node in self.tree:
            if self.distance(node.point, sample) < self.distance(nearest.point, sample):
                nearest = node
        return nearest

    def sample_free(self):
        """Sample a random point in the search space"""
        x = random.uniform(self.search_space[0][0], self.search_space[0][1])
        y = random.uniform(self.search_space[1][0], self.search_space[1][1])
        z = random.uniform(self.search_space[2][0], self.search_space[2][1])
        return Point(x=x, y=y, z=z)

    def steer(self, nearest, sample):
        direction = (sample.x - nearest.point.x, sample.y - nearest.point.y, sample.z - nearest.point.z)
        length = math.sqrt(sum(d ** 2 for d in direction))
        unit_direction = tuple(d / length for d in direction)

        new_point = Point(x=nearest.point.x + self.step_size * unit_direction[0],
                          y=nearest.point.y + self.step_size * unit_direction[1],
                          z=nearest.point.z + self.step_size * unit_direction[2])
        return self.Node(new_point)

    def collision_free(self, nearest, new_node):
        """Check if the path between nodes is free of obstacles (dummy function)"""
        # Implement actual collision checking here
        return True

    def cost(self, from_node, to_node):
        """Calculate the cost to move from one node to another"""
        return from_node.cost + self.distance(from_node.point, to_node.point)

    def rewire(self, new_node):
        """Rewire the tree based on new node"""
        near_nodes = [node for node in self.tree if self.distance(node.point, new_node.point) <= self.radius]
        for node in near_nodes:
            if self.cost(new_node, node) < node.cost and self.collision_free(new_node, node):
                node.parent = new_node
                node.cost = self.cost(new_node, node)
                self.propagate_cost_to_leaves(node)

    def propagate_cost_to_leaves(self, parent_node):
        """Propagate cost changes to leaves of the tree"""
        for node in self.tree:
            if node.parent == parent_node:
                node.cost = self.cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def run(self):
        
        self.tree.append(self.start)
        for i in range(self.max_iter):
            
            sample = self.sample_free()
            nearest = self.nearest_node(sample)
            new_node = self.steer(nearest, sample)
            if self.collision_free(nearest, new_node):
                
                near_nodes = [node for node in self.tree if self.distance(node.point, new_node.point) <= self.radius]
                if near_nodes:
                    
                    new_node.cost = min([node.cost + self.distance(node.point, new_node.point) for node in near_nodes])
                    new_node.parent = min(near_nodes, key=lambda node: node.cost)
                    self.tree.append(new_node)
                    self.rewire(new_node)
                    if self.distance(new_node.point, self.goal.point) <= self.step_size:
                        
                        
                        return self.get_path(new_node)
                else:
                    
                    # No near nodes found, continue to the next iteration
                    continue
        return None


    def get_path(self, end_node):
        """Reconstruct the path from end node to start node"""
        path = []
        while end_node is not None:
            path.append(end_node.point)
            end_node = end_node.parent
        return path[::-1]  # Return reversed path

    def print_graph(self):
        """Print the graph (nodes and edges)"""
        print("Graph (g(v, e)):")
        for node in self.tree:
            if node.parent:
                print(f"Edge: {node.parent.point} -> {node.point}")
            else:
                print(f"Root Node: {node.point}")
        print("")

# Rest of the code for ROS node and publishing markers

def publish_two_points(point1, point2):
    rospy.init_node("two_points_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(100)  # 1 Hz

    # Create a marker for the points
    points_marker = Marker()
    points_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "two_points"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w = 1.0
    points_marker.id = 0
    points_marker.type = Marker.POINTS
    points_marker.scale.x = 0.1  # Point size
    points_marker.scale.y = 0.1
    points_marker.color.g = 1.0  # Green color
    points_marker.color.a = 1.0  # Alpha

    # Add the points to the marker
    points_marker.points.append(point1)
    points_marker.points.append(point2)

    # Publish the points marker
    while not rospy.is_shutdown():
        marker_pub.publish(points_marker)
        rate.sleep()

def publish_path(path_points):
    rospy.init_node("rrt_path_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1000)  # 1
    # Create a marker for the path
    path_marker = Marker()
    path_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "rrt_path"
    path_marker.action = Marker.ADD
    path_marker.pose.orientatioros@ashwin-ASUS-TUF-Gaming-F15-FX506HF-FX506HF:~/catkin_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/scripts$ rosrun hector_quadrotor_gazebo rrt_star2.py
Traceback (most recent call last):
  File "/home/ros/catkin_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/scripts/rviz practice/rrt_star2.py", line 142, in <module>
    main()
  File "/home/ros/catkin_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/scripts/rviz practice/rrt_star2.py", line 124, in main
    start_point = get_point_input("start")
NameError: name 'get_point_input' is not defined
n.w = 1.0
    path_marker.id = 0
    path_marker.type = Marker.LINE_STRIP
    path_marker.scale.x = 0.05  # Line width
    path_marker.color.r = 1.0  # Red color
    path_marker.color.a = 1.0  # Alpha

    # Add the points to the marker
    path_marker.points = path_points

    # Publish the path marker
    while not rospy.is_shutdown():
        marker_pub.publish(path_marker)
        rate.sleep()


if __name__ == "__main__":
    # Get start and goal nodes from user input
    start_node = get_point_input("Start Node")
    goal_node = get_point_input("Goal Node")
    publish_two_points(start_node, goal_node)

    # Define search space (example: 10x10x10 cube centered at the origin)
    search_space = [(0, 5), (0, 5), (0, 5)]

    # Create an RRT* instance
    rrt_star = RRTStar(start=start_node, goal=goal_node, search_space=search_space, step_size=1, max_iter=2000, radius=2.0)

    # Run the RRT* algorithm
    path_points = rrt_star.run()

    if path_points:
        # Convert path points to ROS Point messages
        path_points_ros = [Point(x=p.x, y=p.y, z=p.z) for p in path_points]

        # Print the graph
        rrt_star.print_graph()

        # Publish the path points to RViz
        publish_path(path_points_ros)  # Uncomment this when publishing path points to RViz
    else:
        print("Failed to find a path from start to goal within the specified number of iterations.")

