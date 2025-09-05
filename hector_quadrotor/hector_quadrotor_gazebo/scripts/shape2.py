#!/usr/bin/env python3

#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

def generate_sphere():
    # Initialize ROS node
    rospy.init_node('shape_generator_node', anonymous=True)
    
    # Create publisher for the shape
    shape_pub = rospy.Publisher('shape', Marker, queue_size=10)
    
    # Set rate for publishing shape
    rate = rospy.Rate(10)  # 10 Hz
    
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        time_difference = (current_time - start_time).to_sec()
        
        if time_difference >= 5.0:
            break
        
        # Create a Marker message
        sphere = Marker()
        sphere.header.stamp = current_time
        sphere.header.frame_id = "map"  # Adjust frame_id as necessary
        sphere.type = Marker.SPHERE
        
        # Set the dimensions of the sphere (radius in meters)
        sphere.scale.x = 1.0
        sphere.scale.y = 1.0
        sphere.scale.z = 1.0
        
        # Set the color of the sphere
        sphere.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color, adjust as needed
        
        # Set the position of the sphere
        sphere.pose.position = Point(0.0, 0.0, time_difference)  # Move along z-axis
        
        # Publish the shape
        shape_pub.publish(sphere)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_sphere()
    except rospy.ROSInterruptException:
        pass



