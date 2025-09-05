#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

def get_point_input(point_name):
    x = float(input(f"Enter {point_name} x-coordinate: "))
    y = float(input(f"Enter {point_name} y-coordinate: "))
    z = float(input(f"Enter {point_name} z-coordinate: "))
    return Point(x=x, y=y, z=z)

def calculate_tangent_points(circle_center, circle_radius, external_point):
    distance_to_center = math.sqrt((external_point.x - circle_center.x)**2 + (external_point.y - circle_center.y)**2)
    if distance_to_center < circle_radius:
        return [], []
    angle_to_center = math.atan2(external_point.y - circle_center.y, external_point.x - circle_center.x)
    angle_tangent1 = angle_to_center + math.acos(circle_radius / distance_to_center)
    angle_tangent2 = angle_to_center - math.acos(circle_radius / distance_to_center)
    tangent_point1 = Point(
        x=circle_center.x + circle_radius * math.cos(angle_tangent1),
        y=circle_center.y + circle_radius * math.sin(angle_tangent1),
        z=circle_center.z
    )
    tangent_point2 = Point(
        x=circle_center.x + circle_radius * math.cos(angle_tangent2),
        y=circle_center.y + circle_radius * math.sin(angle_tangent2),
        z=circle_center.z
    )
    return tangent_point1, tangent_point2

def calculate_tangent_lengths(tangent_point1, tangent_point2, circle_center):
    length_tangent1 = math.sqrt((tangent_point1.x - circle_center.x)**2 + (tangent_point1.y - circle_center.y)**2)
    length_tangent2 = math.sqrt((tangent_point2.x - circle_center.x)**2 + (tangent_point2.y - circle_center.y)**2)
    return length_tangent1, length_tangent2

def calculate_diameter_endpoints(circle_center, circle_radius, external_point):
    # Calculate the distance between the circle center and the external point
    distance_to_center = math.sqrt((external_point.x - circle_center.x)**2 + (external_point.y - circle_center.y)**2)
    
    # Calculate the angle between the radius and the line connecting the external point and the circle center
    angle_to_center = math.atan2(external_point.y - circle_center.y, external_point.x - circle_center.x)
    
    # Calculate the angle of the perpendicular line
    angle_perpendicular = angle_to_center + math.pi / 2  # 90 degrees
    
    # Calculate the endpoints of the diameter
    endpoint1 = Point(
        x=circle_center.x + distance_to_center * math.cos(angle_perpendicular),
        y=circle_center.y + distance_to_center * math.sin(angle_perpendicular),
        z=circle_center.z
    )
    endpoint2 = Point(
        x=circle_center.x - distance_to_center * math.cos(angle_perpendicular),
        y=circle_center.y - distance_to_center * math.sin(angle_perpendicular),
        z=circle_center.z
    )
    return endpoint1, endpoint2

def publish_circle_and_tangents_with_diameter(circle_center, circle_radius, tangent_point1, tangent_point2, diameter_endpoint1, diameter_endpoint2):
    rospy.init_node("circle_and_tangents_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)

    # Circle marker, tangent markers, and center point marker
    circle_marker = Marker()
    circle_marker.header.frame_id = "base_link"
    circle_marker.header.stamp = rospy.Time.now()
    circle_marker.ns = "circle"
    circle_marker.action = Marker.ADD
    circle_marker.pose.orientation.w = 1.0
    circle_marker.id = 0
    circle_marker.type = Marker.LINE_STRIP
    circle_marker.scale.x = 0.05
    circle_marker.color.r = 1.0
    circle_marker.color.g = 0.0
    circle_marker.color.b = 0.0
    circle_marker.color.a = 1.0

    num_circle_points = 100
    angle_increment = 2 * math.pi / num_circle_points
    for i in range(num_circle_points + 1):
        angle = i * angle_increment
        x = circle_center.x + circle_radius * math.cos(angle)
        y = circle_center.y + circle_radius * math.sin(angle)
        z = circle_center.z
        circle_marker.points.append(Point(x=x, y=y, z=z))

    tangent_marker1 = Marker()
    tangent_marker1.header.frame_id = "base_link"
    tangent_marker1.header.stamp = rospy.Time.now()
    tangent_marker1.ns = "tangent_line1"
    tangent_marker1.action = Marker.ADD
    tangent_marker1.pose.orientation.w = 1.0
    tangent_marker1.id = 1
    tangent_marker1.type = Marker.LINE_STRIP
    tangent_marker1.scale.x = 0.05
    tangent_marker1.color.r = 0.0
    tangent_marker1.color.g = 1.0
    tangent_marker1.color.b = 0.0
    tangent_marker1.color.a = 1.0
    tangent_marker1.points.append(external_point)
    tangent_marker1.points.append(tangent_point1)

    tangent_marker2 = Marker()
    tangent_marker2.header.frame_id = "base_link"
    tangent_marker2.header.stamp = rospy.Time.now()
    tangent_marker2.ns = "tangent_line2"
    tangent_marker2.action = Marker.ADD
    tangent_marker2.pose.orientation.w = 1.0
    tangent_marker2.id = 2
    tangent_marker2.type = Marker.LINE_STRIP
    tangent_marker2.scale.x = 0.05
    tangent_marker2.color.r = 0.0
    tangent_marker2.color.g = 0.0
    tangent_marker2.color.b = 1.0
    tangent_marker2.color.a = 1.0
    tangent_marker2.points.append(external_point)
    tangent_marker2.points.append(tangent_point2)

    center_point_marker = Marker()
    center_point_marker.header.frame_id = "base_link"
    center_point_marker.header.stamp = rospy.Time.now()
    center_point_marker.ns = "center_point"
    center_point_marker.action = Marker.ADD
    center_point_marker.pose.orientation.w = 1.0
    center_point_marker.id = 3
    center_point_marker.type = Marker.SPHERE
    center_point_marker.scale.x = 0.1
    center_point_marker.scale.y = 0.1
    center_point_marker.scale.z = 0.1
    center_point_marker.color.r = 0.0
    center_point_marker.color.g = 0.0
    center_point_marker.color.b = 1.0
    center_point_marker.color.a = 1.0
    center_point_marker.pose.position.x = circle_center.x
    center_point_marker.pose.position.y = circle_center.y
    center_point_marker.pose.position.z = circle_center.z

    diameter_marker = Marker()
    diameter_marker.header.frame_id = "base_link"
    diameter_marker.header.stamp = rospy.Time.now()
    diameter_marker.ns = "diameter_line"
    diameter_marker.action = Marker.ADD
    diameter_marker.pose.orientation.w = 1.0
    diameter_marker.id = 4
    diameter_marker.type = Marker.LINE_STRIP
    diameter_marker.scale.x = 0.05
    diameter_marker.color.r = 0.0
    diameter_marker.color.g = 1.0
    diameter_marker.color.b = 1.0
    diameter_marker.color.a = 1.0
    diameter_marker.points.append(diameter_endpoint1)
    diameter_marker.points.append(diameter_endpoint2)

    while not rospy.is_shutdown():
        marker_pub.publish(circle_marker)
        marker_pub.publish(tangent_marker1)
        marker_pub.publish(tangent_marker2)
        marker_pub.publish(center_point_marker)
        marker_pub.publish(diameter_marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        circle_center = get_point_input("Circle center")
        circle_radius = float(input("Enter radius of the circle: "))
        external_point = get_point_input("External point")

        tangent_point1, tangent_point2 = calculate_tangent_points(circle_center, circle_radius, external_point)
        diameter_endpoint1, diameter_endpoint2 = calculate_diameter_endpoints(circle_center, circle_radius, external_point)

        if tangent_point1 and tangent_point2:
            length_tangent1, length_tangent2 = calculate_tangent_lengths(tangent_point1, tangent_point2, circle_center)
            print("Length of tangent 1:", length_tangent1)
            print("Length of tangent 2:", length_tangent2)
            
            publish_circle_and_tangents_with_diameter(circle_center, circle_radius, tangent_point1, tangent_point2, diameter_endpoint1, diameter_endpoint2)
        else:
            print("The external point is inside the circle. Unable to calculate tangents.")
    except rospy.ROSInterruptException:
        pass
