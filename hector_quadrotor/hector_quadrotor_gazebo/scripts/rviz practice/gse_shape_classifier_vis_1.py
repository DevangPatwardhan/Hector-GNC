#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def get_point_input(point_name):
    x = float(input(f"Enter {point_name} x-coordinate: "))
    y = float(input(f"Enter {point_name} y-coordinate: "))
    z = float(input(f"Enter {point_name} z-coordinate: "))
    return Point(x=x, y=y, z=z)

def publish_points(point,point3, marker_pub):
    # Create a marker for the point
    point_marker = Marker()
    point_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    point_marker.header.stamp = rospy.Time.now()
    point_marker.ns = "points"
    point_marker.action = Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = Marker.POINTS
    point_marker.scale.x = 0.1  # Point size
    point_marker.scale.y = 0.1
    point_marker.color.g = 1.0  # Green color
    point_marker.color.a = 1.0  # Alpha
    mid_point = Point()
    mid_point.x = 0
    mid_point.y = 0
    mid_point.z = 0

    # Add the point to the marker
    point_marker.points.append(point)
    point_marker.points.append(point3)

    # Publish the point marker
    marker_pub.publish(point_marker)

def publish_line(point1, point2, marker_pub):
    # Create a marker for the line
    line_marker = Marker()
    line_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "line"
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.scale.x = 0.05  # Line width
    line_marker.color.r = 0.0
    line_marker.color.g = 0.0
    line_marker.color.b = 1.0  # Blue color
    line_marker.color.a = 1.0  # Alpha
    
    # Calculate the radius point
    radius = Point()
    radius.x = point2.x + 2  # Access x component directly
    radius.y = point2.y + 2  # Access y component directly
    radius.z = point2.z  # Access z component directly

    # Add the points to the line marker
    line_marker.points.append(point1)
    line_marker.points.append(point2)
    line_marker.points.append(radius)

    # Publish the line marker
    marker_pub.publish(line_marker)

    
def publish_axes(marker_pub):
    # Create a marker for the axes
    axes_marker = Marker()
    axes_marker.header.frame_id = "base_link"
    axes_marker.header.stamp = rospy.Time.now()
    axes_marker.ns = "axes"
    axes_marker.action = Marker.ADD
    axes_marker.pose.orientation.w = 1.0
    axes_marker.id = 0
    axes_marker.type = Marker.LINE_LIST
    axes_marker.scale.x = 0.02  # Line width
    axes_marker.color.a = 1.0  # Alpha

    # Define colors for x, y, z axes
    red = ColorRGBA(1.0, 0.0, 0.0, 1.0)    # Red for x-axis
    green = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green for y-axis
    blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)   # Blue for z-axis

    # Add x, y, z axes
    origin = Point()
    end_x = Point(x=1.0)
    end_y = Point(y=1.0)
    end_z = Point(z=1.0)

    axes_marker.points.extend([origin, end_x, origin, end_y, origin, end_z])

    # Assign colors to axes
    axes_marker.colors.extend([red, red, green, green, blue, blue])

    # Publish the axes marker
    marker_pub.publish(axes_marker)

def main():
    rospy.init_node("points_and_line_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    marker_pubb = rospy.Publisher("visualization_marker_line", Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    try:
        # Get points from user input
        point1 = get_point_input("Point 1")
        point2 = get_point_input("Point 2")
        point3=Point()
        point3.x=(point2.x+(0.5))
        point3.y=point2.y
        point3.z=point2.z
        

        # Publish points and line
        while not rospy.is_shutdown():
            publish_axes(marker_pub)
            publish_points(point1, point2,marker_pub)
            #publish_points(point2, marker_pub)
            publish_line(point1, point2, marker_pubb)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
