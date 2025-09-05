#!/usr/bin/env python3
#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

def generate_cone():
    # Initialize ROS node
    rospy.init_node('shape_generator_node', anonymous=True)
    
    # Create publisher for the shape
    shape_pub = rospy.Publisher('shape', Marker, queue_size=10)
    shape_pubb = rospy.Publisher('shapee', Marker, queue_size=10)
    
    
    # Set rate for publishing shape
    rate = rospy.Rate(1)  # 1 Hz
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        
        current_time = rospy.Time.now()
        # Create a Marker message
        
        time_difference = (current_time - start_time).to_sec()
        if time_difference >= 5.5:
            
            break
        
        cone = Marker()
        cone.header.stamp = current_time
        cone.header.frame_id = "map"  # Adjust frame_id as necessary
        cone.type = Marker.SPHERE
        
        # Set the dimensions of the cone (radius, height in meters)
        cone.scale.x = 1.0  # Radius
        cone.scale.y = 1.0  # Radius
        cone.scale.z = 2.0  # Height
        
        # Set the color of the cone
        cone.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color, adjust as needed
        cone.pose.position = Point( 1.0 ,time_difference, 1.0)
        # Publish the shape
        shape_pub.publish(cone)
        conee = Marker()
        conee.header.stamp = rospy.Time.now()
        conee.header.frame_id = "map"  # Adjust frame_id as necessary
        conee.type = Marker.SPHERE
        
        conee.scale.x = 1.0  # Radius
        conee.scale.y = 1.0  # Radius
        conee.scale.z = 2.0 
        
        
        conee.color = ColorRGBA(2.0, 0.0, 2.0, 1.0)  # Red color, adjust as needed
        conee.pose.position = Point(1.0, 5.0, 1.0)
        shape_pubb.publish(conee)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_cone()
    except rospy.ROSInterruptException:
        pass
