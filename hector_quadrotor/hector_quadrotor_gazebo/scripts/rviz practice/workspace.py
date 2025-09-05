#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.scale.x = 0.02  # Line thickness
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Alpha (transparency)
    return marker

def add_line(marker, start, end):
    start_point = Point(*start)
    end_point = Point(*end)
    marker.points.append(start_point)
    marker.points.append(end_point)

def generate_workspace(min_x, max_x, min_y, max_y, min_z, max_z):
    rospy.init_node('workspace_generator', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        workspace_marker = create_marker()

        # Bottom square
        add_line(workspace_marker, (min_x, min_y, min_z), (max_x, min_y, min_z))
        add_line(workspace_marker, (max_x, min_y, min_z), (max_x, max_y, min_z))
        add_line(workspace_marker, (max_x, max_y, min_z), (min_x, max_y, min_z))
        add_line(workspace_marker, (min_x, max_y, min_z), (min_x, min_y, min_z))

        # Top square
        add_line(workspace_marker, (min_x, min_y, max_z), (max_x, min_y, max_z))
        add_line(workspace_marker, (max_x, min_y, max_z), (max_x, max_y, max_z))
        add_line(workspace_marker, (max_x, max_y, max_z), (min_x, max_y, max_z))
        add_line(workspace_marker, (min_x, max_y, max_z), (min_x, min_y, max_z))

        # Connecting lines
        add_line(workspace_marker, (min_x, min_y, min_z), (min_x, min_y, max_z))
        add_line(workspace_marker, (max_x, min_y, min_z), (max_x, min_y, max_z))
        add_line(workspace_marker, (max_x, max_y, min_z), (max_x, max_y, max_z))
        add_line(workspace_marker, (min_x, max_y, min_z), (min_x, max_y, max_z))

        pub.publish(workspace_marker)
        rate.sleep()

if __name__ == '__main__':
    min_x, max_x = -5.0, 5.0
    min_y, max_y = -5.0, 5.0
    min_z, max_z = 0.0, 5.0
    generate_workspace(min_x, max_x, min_y, max_y, min_z, max_z)
