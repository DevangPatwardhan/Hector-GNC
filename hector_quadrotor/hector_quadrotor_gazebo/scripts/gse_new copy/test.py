#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher:
    def __init__(self):
        rospy.init_node("two_points_and_vector_rviz", anonymous=True)  # Added anonymous=True to ensure unique node names
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # Added a rate to control the publishing frequency

    def publish_two_points(self, point1, point2):
        # Create a marker for the points
        points_marker = Marker()
        points_marker.header.frame_id = "base_link"  # Ensure this matches the fixed frame in RViz
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

        # Add the two points to the marker
        points_marker.points.append(point1)
        points_marker.points.append(point2)

        # Publish the points marker
        self.marker_pub.publish(points_marker)
        self.rate.sleep()  # Sleep to maintain the publishing rate

    def publish_vector_between_points(self, point1, point2):
        # Create a marker for the vector arrow
        vector_marker = Marker()
        vector_marker.header.frame_id = "base_link"  # Ensure this matches the fixed frame in RViz
        vector_marker.header.stamp = rospy.Time.now()
        vector_marker.ns = "vector_between_points"
        vector_marker.action = Marker.ADD
        vector_marker.pose.orientation.w = 1.0
        vector_marker.id = 1  # Changed ID to be unique for each marker
        vector_marker.type = Marker.ARROW
        vector_marker.scale.x = 0.05  # Arrow shaft diameter
        vector_marker.scale.y = 0.1  # Arrow head diameter
        vector_marker.scale.z = 0.2  # Arrow head length
        vector_marker.color.r = 1.0  # Red color
        vector_marker.color.a = 1.0  # Alpha

        # Set the start and end points of the vector
        vector_marker.points.append(point1)
        vector_marker.points.append(point2)

        # Publish the vector marker
        self.marker_pub.publish(vector_marker)
        self.rate.sleep()  # Sleep to maintain the publishing rate

def main():
    try:
        marker_publisher = MarkerPublisher()
        point1 = Point(x=0.0, y=0.0, z=0.0)  # Example point, replace with actual values
        point2 = Point(x=1.0, y=1.0, z=1.0)  # Example point, replace with actual values

        while not rospy.is_shutdown():
            marker_publisher.publish_two_points(point1, point2)
            marker_publisher.publish_vector_between_points(point1, point2)
            marker_publisher.rate.sleep()  # Sleep to maintain the loop rate
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
