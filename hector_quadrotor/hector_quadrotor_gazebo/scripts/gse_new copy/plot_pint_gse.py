#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_two_points(point1, point2, marker_pub):
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
    points_marker.color.r = 0.0
    points_marker.color.g = 1.0  # Green color
    points_marker.color.b = 0.0
    points_marker.color.a = 1.0  # Alpha

    # Add the two points to the marker
    points_marker.points.append(point1)
    points_marker.points.append(point2)

    # Publish the points marker
    marker_pub.publish(points_marker)

def publish_vector_between_points(point1, point2, marker_pub):
    # Create a marker for the vector arrow
    vector_marker = Marker()
    vector_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    vector_marker.header.stamp = rospy.Time.now()
    vector_marker.ns = "vector_between_points"
    vector_marker.action = Marker.ADD
    vector_marker.pose.orientation.w = 1.0
    vector_marker.id = 0
    vector_marker.type = Marker.ARROW
    vector_marker.scale.x = 0.05  # Arrow shaft diameter
    vector_marker.scale.y = 0.1  # Arrow head diameter
    vector_marker.scale.z = 0.2  # Arrow head length
    vector_marker.color.r = 1.0
    vector_marker.color.g = 0.0  # Red color
    vector_marker.color.b = 0.0
    vector_marker.color.a = 1.0  # Alpha

    # Calculate vector direction
    vector_direction = Point()
    vector_direction.x = point2.x - point1.x
    vector_direction.y = point2.y - point1.y
    vector_direction.z = point2.z - point1.z

    # Set arrow position to the midpoint between the two points
    #vector_marker.pose.position.x = (point1.x + point2.x) / 2
    #vector_marker.pose.position.y = (point1.y + point2.y) / 2
    #vector_marker.pose.position.z = (point1.z + point2.z) / 2
    
    vector_marker.pose.position.x = point1.x 
    vector_marker.pose.position.y = point1.y 
    vector_marker.pose.position.z = point1.z 

    # Set arrow orientation
    vector_marker.pose.orientation.x = 0.0
    vector_marker.pose.orientation.y = 0.0
    vector_marker.pose.orientation.z = 0.0
    vector_marker.pose.orientation.w = 1.0

    # Set arrow direction
    vector_marker.points.append(point1)
    vector_marker.points.append(point2)

    # Publish the vector marker
    marker_pub.publish(vector_marker)

if __name__ == "__main__":
    try:
        rospy.init_node("two_points_and_vector_rviz")
        marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        
        # Rate for publishing
        rate = rospy.Rate(1)  # 1 Hz

        # Get points from user input
        point1 = Point(x=0.0, y=0.0, z=0.0)  # Example point, you can change this
        point2 = Point(x=1.0, y=1.0, z=1.0)  # Example point, you can change this
        
        

        # Main loop
        while not rospy.is_shutdown():
            publish_two_points(point1, point2, marker_pub)
            publish_vector_between_points(point1, point2, marker_pub)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
