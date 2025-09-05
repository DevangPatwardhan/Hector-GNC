#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math

# Helper functions
def distance(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

def RandomPosition(x_max, y_max, z_max):
    return Point(
        x=random.uniform(0, x_max),
        y=random.uniform(0, y_max),
        z=random.uniform(0, z_max)
    )

def IsInObstacle(Xnew, obstacles):
    # Define your obstacle space here
    for obs in obstacles:
        if distance(Xnew, obs) < obs.radius:
            return True
    return False

def Nearest(vertices, Xnew):
    closest_vertex = None
    min_dist = float('inf')
    for v in vertices:
        dist = distance(Xnew, v)
        if dist < min_dist:
            closest_vertex = v
            min_dist = dist
    return closest_vertex

def Chain(Xnew, Xnearest):
    # Here you would normally consider your motion model and collision checking
    return [Xnearest, Xnew]

def Qgoal(goal, Xnew, threshold):
    return distance(Xnew, goal) < threshold

# Main RRT function
def rrt(start, goal, lim, x_max, y_max, z_max, obstacles):
    G = {'V': [start], 'E': []}
    counter = 0

    while counter < lim:
        Xnew = RandomPosition(x_max, y_max, z_max)
        if IsInObstacle(Xnew, obstacles):
            continue
        Xnearest = Nearest(G['V'], Xnew)
        Link = Chain(Xnew, Xnearest)
        G['V'].append(Xnew)
        G['E'].append(Link)
        counter += 1
        if Qgoal(goal, Xnew, 0.5):  # Threshold of 0.5 units to consider as goal
            break
    return G

# Visualization functions
def visualize_points(points):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.id = 0
    marker.type = Marker.POINTS
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.g = 1.0
    marker.color.a = 1.0
    marker.points = points
    return marker

def visualize_path(G):
    line = Marker()
    line.header.frame_id = "base_link"
    line.header.stamp = rospy.Time.now()
    line.ns = "path"
    line.action = Marker.ADD
    line.pose.orientation.w = 1.0
    line.id = 1
    line.type = Marker.LINE_STRIP
    line.scale.x = 0.02
    line.color.r = 1.0
    line.color.a = 1.0
    for edge in G['E']:
        line.points.append(edge[0])
        line.points.append(edge[1])
    return line

if __name__ == "__main__":
    rospy.init_node("rrt_path_planning")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    r = rospy.Rate(30)

    # Define your start and goal points here
    start_point = Point(x=0, y=0, z=0)
    goal_point = Point(x=5, y=5, z=5)

    # Define the maximum extents of your space
    x_max, y_max, z_max = 10, 10, 10

    # Define your obstacles here (as points with a radius for simplicity)
    obstacles = [Point(x=2, y=2, z=2, radius=1), Point(x=4, y=4, z=4, radius=1)]

    # Generate the RRT graph
    rrt_graph = rrt(start_point, goal_point, 1000, x_max, y_max, z_max, obstacles)

    # Visualize the uniform points and the path
    points_marker = visualize_points([v for v in rrt_graph['V']])
    path_marker = visualize_path(rrt_graph)

    while not rospy.is_shutdown():
        marker_pub.publish(points_marker)
        marker_pub.publish(path_marker)
        r.sleep()
