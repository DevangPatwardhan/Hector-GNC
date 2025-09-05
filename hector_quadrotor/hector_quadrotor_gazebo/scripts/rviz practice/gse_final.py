#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
import heapq
import math

class GSE:
    def __init__(self):
        self.vertices = []
        self.obstacles = []
        self.adj = {}
        self.projector = None

    def solve(self, start, goal):
        if not self.vertices:
            self.addVertex(goal)

        while True:
            self.addVertex(start)
            self.addVertex(goal)

            for i, v in enumerate(self.vertices):
                if self.shape(v, start):
                    self.adj.setdefault(i, []).append(len(self.vertices) - 2)
                    self.adj.setdefault(len(self.vertices) - 2, []).append(i)

                if self.shape(v, goal):
                    self.adj.setdefault(i, []).append(len(self.vertices) - 1)
                    self.adj.setdefault(len(self.vertices) - 1, []).append(i)

            if len(self.adj) >= 2:
                path_indices = self.dijkstra(len(self.vertices) - 1, len(self.vertices) - 2)
                if path_indices:
                    path = [self.vertices[i] for i in path_indices]
                    break

        rospy.loginfo("Created {} states".format(len(self.vertices)))
        return path

    def addVertex(self, state):
        self.vertices.append(state)
        return len(self.vertices) - 1

    def shape(self, state1, state2):
        # Assuming circular shape with radius 1
        return math.sqrt((state1[0] - state2[0]) ** 2 + (state1[1] - state2[1]) ** 2) >= 1.0

    def dijkstra(self, start, end):
        pq = [(0, start)]
        dist = {start: 0}
        prev = {}

        while pq:
            d, u = heapq.heappop(pq)
            if u == end:
                path = []
                while u in prev:
                    path.insert(0, u)
                    u = prev[u]
                path.insert(0, start)
                return path

            for v in self.adj[u]:
                alt = dist[u] + math.sqrt((self.vertices[u][0] - self.vertices[v][0]) ** 2 + (self.vertices[u][1] - self.vertices[v][1]) ** 2)
                if alt < dist.get(v, float('inf')):
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(pq, (alt, v))

        return None

    def addObstacle(self, obs):
        self.obstacles.append(obs)

    def setProjector(self, projector):
        self.projector = projector

def visualize_path(start, goal, path):
    rospy.init_node('path_visualizer')
    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        # Visualize start point
        start_marker = Marker()
        start_marker.header.frame_id = "base_link"
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = "points"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position = Point(start[0], start[1], 0)
        start_marker.pose.orientation.w = 1.0
        start_marker.scale = Vector3(0.2, 0.2, 0.2)
        start_marker.color = ColorRGBA(0, 1, 0, 1)  # Green
        marker_array.markers.append(start_marker)

        # Visualize goal point
        goal_marker = Marker()
        goal_marker.header.frame_id = "/map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = "points"
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = Point(goal[0], goal[1], 0)
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale = Vector3(0.2, 0.2, 0.2)
        goal_marker.color = ColorRGBA(0, 1, 0, 1)  # Green
        marker_array.markers.append(goal_marker)

        # Visualize path
        for i in range(len(path) - 1):
            line = Marker()
            line.header.frame_id = "/map"
            line.header.stamp = rospy.Time.now()
            line.ns = "path"
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.1
            line.color.r = 1.0
            line.color.a = 1.0
            line.points.append(Point(path[i][0], path[i][1], 0))
            line.points.append(Point(path[i + 1][0], path[i + 1][1], 0))
            marker_array.markers.append(line)

        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    planner = GSE()

    # Accept start and goal points from user
    start = (float(input("Enter start point x coordinate: ")), float(input("Enter start point y coordinate: ")))
    goal = (float(input("Enter goal point x coordinate: ")), float(input("Enter goal point y coordinate: ")))

    # Solve for path
    path = planner.solve(start, goal)

    # Visualize path
    visualize_path(start, goal, path)
