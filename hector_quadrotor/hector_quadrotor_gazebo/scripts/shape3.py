#!/usr/bin/env python3



import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def generate_expanding_sphere():
    # Initialize ROS node
    rospy.init_node('shape_generator_node', anonymous=True)
    
    # Create publisher for the sphere
    shape_pub = rospy.Publisher('shape', Marker, queue_size=10)
    
    # Set rate for publishing sphere
    rate = rospy.Rate(10)  # 10 Hz
    
    start_time = rospy.Time.now()
    duration = rospy.Duration(5)  # Duration for expansion (in seconds)
    
    while not rospy.is_shutdown() and rospy.Time.now() - start_time < duration:
        # Calculate expansion factor based on elapsed time
        time_elapsed = rospy.Time.now() - start_time
        expansion_factor = time_elapsed.to_sec() / duration.to_sec()
        
        # Create a Marker message for the sphere
        sphere = Marker()
        sphere.header.stamp = rospy.Time.now()
        sphere.header.frame_id = "map"  # Adjust frame_id as necessary
        sphere.type = Marker.SPHERE
        
        # Set the dimensions of the sphere (radius in meters)
        sphere.scale.x = expansion_factor
        sphere.scale.y = expansion_factor
        sphere.scale.z = expansion_factor
        
        # Set the color of the sphere
        sphere.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
        
        # Publish the sphere
        shape_pub.publish(sphere)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_expanding_sphere()
    except rospy.ROSInterruptException:
        pass

