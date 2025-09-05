#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z):
    marker = Marker()
    marker.header.frame_id = "world"
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
    marker.header.frame_id = "world"
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

def generate_scene(min_x, max_x, min_y, max_y, min_z, max_z, spheres):
    rospy.init_node('scene_generator', anonymous=True)
    pub_workspace = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    sphere_publishers = [rospy.Publisher(f'shape{i+1}', Marker, queue_size=10) for i in range(len(spheres))]
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        workspace_marker = create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z)
        pub_workspace.publish(workspace_marker)
        
        for i, (center_x, center_y, center_z, radius, color) in enumerate(spheres):
            sphere_marker = create_sphere_marker(center_x, center_y, center_z, radius, color)
            sphere_publishers[i].publish(sphere_marker)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        min_x, max_x = -5.0, 5.0
        min_y, max_y = -5.0, 5.0
        min_z, max_z = 0.0, 5.0
        
        num_spheres = int(input("Enter the number of spheres: "))
        spheres = []

        for i in range(num_spheres):
            print(f"Enter details for sphere {i + 1}:")
            center_x = float(input("  Center X: "))
            center_y = float(input("  Center Y: "))
            center_z = float(input("  Center Z: "))
            radius = float(input("  Radius: "))
            color_r = float(input("  Color R: "))
            color_g = float(input("  Color G: "))
            color_b = float(input("  Color B: "))
            color_a = float(input("  Color A: "))
            color = ColorRGBA(color_r, color_g, color_b, color_a)
            spheres.append((center_x, center_y, center_z, radius, color))
        
        generate_scene(min_x, max_x, min_y, max_y, min_z, max_z, spheres)
    except rospy.ROSInterruptException:
        pass
