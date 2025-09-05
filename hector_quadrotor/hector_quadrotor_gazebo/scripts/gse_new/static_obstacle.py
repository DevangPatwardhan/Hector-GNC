#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.02  # Line thickness
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Alpha (transparency)
    
    # Define the vertices of the wall (12 line segments)
    vertices = [
        [min_x, min_y, min_z], [max_x, min_y, min_z],
        [max_x, min_y, min_z], [max_x, max_y, min_z],
        [max_x, max_y, min_z], [min_x, max_y, min_z],
        [min_x, max_y, min_z], [min_x, min_y, min_z],
        
        [min_x, min_y, max_z], [max_x, min_y, max_z],
        [max_x, min_y, max_z], [max_x, max_y, max_z],
        [max_x, max_y, max_z], [min_x, max_y, max_z],
        [min_x, max_y, max_z], [min_x, min_y, max_z],
        
        [min_x, min_y, min_z], [min_x, min_y, max_z],
        [max_x, min_y, min_z], [max_x, min_y, max_z],
        [max_x, max_y, min_z], [max_x, max_y, max_z],
        [min_x, max_y, min_z], [min_x, max_y, max_z]
    ]
    
    # Set the vertices for the workspace Marker message
    for vertex in vertices:
        point = Point()
        point.x, point.y, point.z = vertex[0], vertex[1], vertex[2]
        marker.points.append(point)
    
    return marker

def create_sphere_marker(center_x, center_y, center_z, radius, color):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.scale.z = radius * 2
    marker.pose.position.x = center_x
    marker.pose.position.y = center_y
    marker.pose.position.z = center_z
    marker.color = color
    return marker

def generate_scene(min_x, max_x, min_y, max_y, min_z, max_z,
                   center_x1, center_y1, center_z1, radius1,
                   center_x2, center_y2, center_z2, radius2,
                   center_x3, center_y3, center_z3, radius3):
    rospy.init_node('scene_generator', anonymous=True)
    pub_workspace = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    pub_sphere1 = rospy.Publisher('shape1', Marker, queue_size=10)
    pub_sphere2 = rospy.Publisher('shape2', Marker, queue_size=10)
    pub_sphere3 = rospy.Publisher('shape3', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    
    #workspace_marker = create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z)
    #sphere_marker1 = create_sphere_marker(center_x1, center_y1, center_z1, radius1, ColorRGBA(1.0, 0.0, 0.0, 1.0))
    #sphere_marker2 = create_sphere_marker(center_x2, center_y2, center_z2, radius2, ColorRGBA(0.0, 1.0, 0.0, 1.0))
    #sphere_marker3 = create_sphere_marker(center_x3, center_y3, center_z3, radius3, ColorRGBA(0.0, 0.0, 1.0, 1.0))

    #pub_workspace.publish(workspace_marker)
    #pub_sphere1.publish(sphere_marker1)
    #pub_sphere2.publish(sphere_marker2)
    #pub_sphere3.publish(sphere_marker3)
        
    #rate.sleep()

    while not rospy.is_shutdown():
        workspace_marker = create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z)
        sphere_marker1 = create_sphere_marker(center_x1, center_y1, center_z1, radius1, ColorRGBA(1.0, 0.0, 0.0, 1.0))
        sphere_marker2 = create_sphere_marker(center_x2, center_y2, center_z2, radius2, ColorRGBA(0.0, 1.0, 0.0, 1.0))
        sphere_marker3 = create_sphere_marker(center_x3, center_y3, center_z3, radius3, ColorRGBA(0.0, 0.0, 1.0, 1.0))

        pub_workspace.publish(workspace_marker)
        pub_sphere1.publish(sphere_marker1)
        pub_sphere2.publish(sphere_marker2)
        pub_sphere3.publish(sphere_marker3)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        min_x, max_x = -5.0, 5.0
        min_y, max_y = -5.0, 5.0
        min_z, max_z = 0.0, 5.0
        
        center_x1, center_y1, center_z1 = -3.0, -3.0, 3.0
        radius1 = 0.5
        
        center_x2, center_y2, center_z2 = 4.0, 3.0, 4.0
        radius2 = 0.5
        
        center_x3, center_y3, center_z3 = 0.0, 0.0, 2.0
        radius3 = 0.75
        
        generate_scene(min_x, max_x, min_y, max_y, min_z, max_z,
                       center_x1, center_y1, center_z1, radius1,
                       center_x2, center_y2, center_z2, radius2,
                       center_x3, center_y3, center_z3, radius3)
    except rospy.ROSInterruptException:
        pass
