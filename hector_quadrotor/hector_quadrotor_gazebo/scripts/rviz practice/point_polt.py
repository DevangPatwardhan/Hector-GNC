#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_single_point(point):
    rospy.init_node("single_point_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a marker for the single point
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    point_marker.header.stamp = rospy.Time.now()
    point_marker.ns = "single_point"
    point_marker.action = Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = Marker.POINTS
    point_marker.scale.x = 0.1  # Point size
    point_marker.scale.y = 0.1
    point_marker.color.g = 1.0  # Green color
    point_marker.color.a = 1.0  # Alpha

    # Set the position of the point
    single_point = Point()
    single_point.x = point.x
    single_point.y = point.y
    single_point.z = point.z

    # Add the point to the marker
    point_marker.points.append(single_point)

    # Publish the single point marker
    while not rospy.is_shutdown():
        marker_pub.publish(point_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        # Define the coordinates for the single point
        x_coord = 1
        y_coord = 1
        z_coord = 1
        random_point=Point(x_coord,y_coord,z_coord)

        publish_single_point(random_point)
    except rospy.ROSInterruptException:
        pass
