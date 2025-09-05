#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Callback function to print the position
def print_position(marker_pub, point_marker):
    rospy.sleep(5)  # Wait for the point to finish moving
    final_position = point_marker.points[-1]  # Get the last position of the point
    rospy.loginfo(f"Final position of the point: x={final_position.x}, y={final_position.y}, z={final_position.z}")
    marker_pub.publish(point_marker)  # Publish the final position

def move_point_x():
    rospy.init_node("move_point_x_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    
    # Create a marker for the point
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"
    point_marker.ns = "moving_point"
    point_marker.action = Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = Marker.POINTS
    point_marker.scale.x = 0.1
    point_marker.scale.y = 0.1
    point_marker.color.g = 1.0
    point_marker.color.a = 1.0

    # Initial position of the point
    moving_point = Point()
    moving_point.x = 0.0
    moving_point.y = 2.0
    moving_point.z = 0.0

    # Add the point to the marker
    point_marker.points.append(moving_point)

    # Publish the initial marker
    marker_pub.publish(point_marker)

    # Move the point along the x-axis for 5 seconds
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 5:
        moving_point.x += 0.1
        point_marker.points[-1] = moving_point  # Update the point's position
        point_marker.header.stamp = rospy.Time.now()
        marker_pub.publish(point_marker)
        rospy.sleep(0.1)

    # After moving, call the function to print the position
    print(f"Final position of the point: x={moving_point.x}, y={moving_point.y}, z={moving_point.z}")
    print_position(marker_pub, point_marker)

if __name__ == "__main__":
    try:
        move_point_x()
    except rospy.ROSInterruptException:
        pass
