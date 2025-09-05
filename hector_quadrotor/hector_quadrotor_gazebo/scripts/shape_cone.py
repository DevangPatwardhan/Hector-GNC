#!/usr/bin/env python3
#!/usr/bin/env python



import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math

class RRTPlanner:
    def __init__(self):
        rospy.init_node('rrt_planner', anonymous=True)
        self.tree_pub = rospy.Publisher('/rrt_tree', Marker, queue_size=10)
        self.goal_pub = rospy.Publisher('/rrt_goal', Marker, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_user_input(self):
        initial_x = float(input("Enter initial x position: "))
        initial_y = float(input("Enter initial y position: "))
        initial_z = float(input("Enter initial z position: "))
        goal_x = float(input("Enter goal x position: "))
        goal_y = float(input("Enter goal y position: "))
        goal_z = float(input("Enter goal z position: "))
        return (Point(initial_x, initial_y, initial_z), Point(goal_x, goal_y, goal_z))

    def generate_rrt(self):
        initial_position, goal_position = self.get_user_input()

        # Initialize the RRT tree with the initial position
        tree = [initial_position]

        while not rospy.is_shutdown():
            # Generate a random point in the space
            random_point = Point()
            random_point.x = random.uniform(-5, 5)
            random_point.y = random.uniform(-5, 5)
            random_point.z = random.uniform(-5, 5)

            # Find the nearest node in the tree to the random point
            nearest_node = self.find_nearest_node(tree, random_point)

            # Extend the tree towards the random point
            new_node = self.extend_tree(nearest_node, random_point)

            # Publish the new node as a marker
            self.publish_marker(new_node)

            # Check if the new node is close enough to the goal
            if self.is_near_goal(new_node, goal_position):
                rospy.loginfo("Goal reached!")
                self.publish_marker(goal_position, True)
                break

            self.rate.sleep()

    def find_nearest_node(self, tree, point):
        min_dist = float('inf')
        nearest_node = None
        for node in tree:
            dist = self.euclidean_distance(node, point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def extend_tree(self, nearest_node, random_point):
        max_dist = 0.5  # Maximum distance to extend towards random point
        dist = self.euclidean_distance(nearest_node, random_point)
        if dist <= max_dist:
            return random_point
        else:
            scale = max_dist / dist
            new_node = Point()
            new_node.x = nearest_node.x + scale * (random_point.x - nearest_node.x)
            new_node.y = nearest_node.y + scale * (random_point.y - nearest_node.y)
            new_node.z = nearest_node.z + scale * (random_point.z - nearest_node.z)
            return new_node

    def publish_marker(self, node, is_goal=False):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.pose.position = node
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        if is_goal:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0
        self.tree_pub.publish(marker)

    def is_near_goal(self, node, goal_position):
        threshold = 0.5  # Threshold distance to consider goal reached
        dist = self.euclidean_distance(node, goal_position)
        return dist < threshold

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)

if __name__ == '__main__':
    try:
        planner = RRTPlanner()
        planner.generate_rrt()
    except rospy.ROSInterruptException:
        pass
