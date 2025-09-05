#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node('cone_visualization_node')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)

    # Cone parameters
    base_radius = 1.0
    height = 2.0
    segments = 20
    move_speed = 0.2  # units per second
    start_time = rospy.Time.now()
    duration = rospy.Duration.from_sec(5.0)

    while not rospy.is_shutdown():
        time_elapsed = rospy.Time.now() - start_time
        if time_elapsed >= duration:
            break

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cone"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Line width

        x = move_speed * time_elapsed.to_sec()

        points = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            px = x + base_radius * math.cos(angle)
            py = base_radius * math.sin(angle)
            points.append(Point(px, py, 0.0))  # Base points

        points.append(Point(x, 0.0, height))  # Apex point

        # Connect base points to apex
        for i in range(segments):
            marker.points.append(points[i])
            marker.points.append(points[-1])
            marker.points.append(points[(i + 1) % segments])
            marker.points.append(points[-1])

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
