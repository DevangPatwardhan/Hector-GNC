#!/usr/bin/env python3
#!/usr/bin/env python



import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from math import sqrt

def generate_cones():
    # Initialize ROS node
    rospy.init_node('shape_generator_node', anonymous=True)
    
    # Create publisher for the shapes
    shape_pub = rospy.Publisher('shape', Marker, queue_size=10)
    shape_pubb = rospy.Publisher('shapee', Marker, queue_size=10)
    
    # Set rate for publishing shapes
    rate = rospy.Rate(1)  # 1 Hz
    
    start_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        time_difference = (current_time - start_time).to_sec()
        
        # Create a Marker message for the moving cone
        cone = Marker()
        cone.header.stamp = current_time
        cone.header.frame_id = "map"  # Adjust frame_id as necessary
        cone.type = Marker.SPHERE
        
        # Set the dimensions of the cone (radius in meters)
        cone.scale.x = 1.0  # Radius
        cone.scale.y = 1.0  # Radius
        cone.scale.z = 2.0  # Height
        
        # Set the color of the cone
        cone.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color, adjust as needed
        
        # Set the position of the cone (moving along the z-axis)
        cone.pose.position = Point(1.0, time_difference, 1.0)  # Move along y-axis
        
        # Publish the moving cone
        shape_pub.publish(cone)
        
        # Create a Marker message for the static cone
        static_cone = Marker()
        static_cone.header.stamp = current_time
        static_cone.header.frame_id = "map"  # Adjust frame_id as necessary
        static_cone.type = Marker.SPHERE
        
        # Set the dimensions of the static cone (radius in meters)
        static_cone.scale.x = 1.0  # Radius
        static_cone.scale.y = 1.0  # Radius
        static_cone.scale.z = 2.0  # Height
        
        # Set the color of the static cone
        static_cone.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue color, adjust as needed
        
        # Set the position of the static cone
        static_cone.pose.position = Point(1.0, 5.0, 1.0)  # Static position
        
        # Publish the static cone
        shape_pubb.publish(static_cone)
        
        # Check for collision
        if check_collision(cone.pose.position, static_cone.pose.position):
            rospy.loginfo("Hit!")
        
        rate.sleep()

def check_collision(pos1, pos2):
    # Calculate distance between two points in 3D space
    distance = sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)
    # Check if the distance is less than the sum of the radii
    if distance <= 2.0:  # Assuming both spheres have a radius of 1.0
        return True
    else:
        return False

if __name__ == '__main__':
    try:
        generate_cones()
    except rospy.ROSInterruptException:
        pass
