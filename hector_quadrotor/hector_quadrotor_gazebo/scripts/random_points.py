#!/usr/bin/env python3
#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def generate_uniform_points():
    rospy.init_node("uniform_points_rviz")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    r = rospy.Rate(30)  # Publish rate (30 Hz)

    # Create a marker for points
    points = Marker()
    points.header.frame_id = "base_link"  # Replace with a valid frame ID
    points.header.stamp = rospy.Time.now()
    points.ns = "uniform_points"
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS
    points.scale.x, points.scale.y = 0.01, 0.01  # Set point size to 1 cm
    points.color.g, points.color.a = 1.0, 1.0  # Set points to green color

    # Generate uniform points in a 50x50x50 grid
    for x in range(50):
        for y in range(50):
            for z in range(50):
                p = Point()
                p.x, p.y, p.z = x, y, z
                points.points.append(p)

    while not rospy.is_shutdown():
        marker_pub.publish(points)
        r.sleep()

if __name__ == "__main__":
    try:
        generate_uniform_points()
    except rospy.ROSInterruptException:
        pass
