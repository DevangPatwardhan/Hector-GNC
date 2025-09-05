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


def publish_two_points(start_point, goal_point):
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

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
    points_marker.points.append(start_point)
    points_marker.points.append(goal_point)
    marker_pub.publish(points_marker)
    rate.sleep()


def publish_path(path):
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    path_marker = Marker()
    path_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "path"
    path_marker.action = Marker.ADD
    path_marker.pose.orientation.w = 1.0
    path_marker.id = 1
    path_marker.type = Marker.LINE_STRIP
    path_marker.scale.x = 0.05  # Line width
    path_marker.color.r = 1.0  # Red color
    path_marker.color.a = 1.0  # Alpha

    path_marker.points = path

    marker_pub.publish(path_marker)
    rate.sleep()

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

    def steer(self, nearest , sample):
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
        """Run the RRT* algorithm"""
        self.tree.append(self.start)
        for i in range(self.max_iter):
            sample = self.sample_free()
            nearest = self.nearest_node(sample)
            new_node = self.steer(nearest, sample)
            if self.collision_free(nearest, new_node):
                near_nodes = [node for node in self.tree if self.distance(node.point, new_node.point) <= self.radius]
                new_node.cost = min([self.cost(node, new_node) for node in near_nodes] + [self.cost(nearest, new_node)])
                new_node.parent = nearest
                for node in near_nodes:
                    if self.cost(new_node, node) < node.cost and self.collision_free(new_node, node):
                        node.parent = new_node
                        node.cost = self.cost(new_node, node)
                self.tree.append(new_node)

                # Check if new_node is close to goal
                if self.distance(new_node.point, self.goal.point) <= self.step_size:
                    return self.generate_path(self.goal, new_node)

        # If max iterations reached without finding path, return None
        return None

    def generate_path(self, goal, last_node):
        """Generate the path from start to goal"""
        path = [goal.point]
        current_node = last_node
        while current_node.parent is not None:
            path.append(current_node.point)
            current_node = current_node.parent
        path.append(self.start.point)
        path.reverse()
        return path

# Main function to run the RRT* algorithm
def main():
    rospy.init_node('rrt_star')

    # Define the search space as a tuple of min and max for x, y, and z
    search_space = ((0, 5), (0, 5), (0, 5))

    # Define the start and goal points
    start_point = get_point_input("start")
    goal_point = get_point_input("goal")
    publish_two_points(start_point, goal_point)

    # Create an instance of RRTStar with the start and goal points
    rrt_star = RRTStar(start=start_point, goal=goal_point, search_space=search_space,
                       step_size=1, max_iter=2000, radius=1.5)

    # Run the RRT* algorithm
    path = rrt_star.run()

    if path:
        print("Path found:")
        for point in path:
            print(f"({point.x}, {point.y}, {point.z})")
        publish_path(path)  # Visualize the path
    else:
        print("No path found.")

if __name__ == '__main__':
    main()


