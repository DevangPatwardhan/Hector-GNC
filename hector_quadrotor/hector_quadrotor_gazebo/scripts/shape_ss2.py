#!/usr/bin/env python3
#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import ColorRGBA

def main():
    rospy.init_node('stl_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('stl_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "base_link"  # Set base frame
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "file:///home/ros/stl/cone_and_hemisphere.stl"  # Set the path to your STL file
    marker.action = Marker.ADD
    marker.pose = Pose()  # Set the pose of the model if needed
    marker.scale.x = 1  # Adjust the scale of the model if needed
    marker.scale.y = 1
    marker.scale.z = 1

    # Initialize quaternion to identity
    marker.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

    # Set color to fully opaque white
    marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rospy.sleep(1.0)  # Publish every 1 second

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
