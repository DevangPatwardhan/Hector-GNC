#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import time

def generate_two_spheres(center_x1, center_y1, center_z1, radius1, center_x2, center_y2, center_z2, radius2):
    # Initialize ROS node
    rospy.init_node('sphere_generator_node', anonymous=True)
    
    # Create publisher for the spheres
    shape1_pub = rospy.Publisher('shape1', Marker, queue_size=10)
    shape2_pub = rospy.Publisher('shape2', Marker, queue_size=10)
    
    # Set rate for publishing spheres
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Marker message for the first sphere
    sphere1 = Marker()
    sphere1.header.stamp = rospy.Time.now()
    sphere1.header.frame_id = "base_link"  # Adjust frame_id as necessary
    sphere1.type = Marker.SPHERE
        
    # Set the dimensions of the first sphere (radius in meters)
    sphere1.scale.x = radius1 * 2
    sphere1.scale.y = radius1 * 2
    sphere1.scale.z = radius1 * 2
        
    # Set the pose of the first sphere (center point)
    sphere1.pose.position.x = center_x1
    sphere1.pose.position.y = center_y1
    sphere1.pose.position.z = center_z1
        
    # Set the color of the first sphere
    sphere1.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
        
   
    
    # Add a short delay to allow RViz to render the first sphere
    time.sleep(0.3)

    # Create a Marker message for the second sphere
    sphere2 = Marker()
    sphere2.header.stamp = rospy.Time.now()
    sphere2.header.frame_id = "base_link"  # Adjust frame_id as necessary
    sphere2.type = Marker.SPHERE
        
    # Set the dimensions of the second sphere (radius in meters)
    sphere2.scale.x = radius2 * 2
    sphere2.scale.y = radius2 * 2
    sphere2.scale.z = radius2 * 2
        
    # Set the pose of the second sphere (center point)
    sphere2.pose.position.x = center_x2
    sphere2.pose.position.y = center_y2
    sphere2.pose.position.z = center_z2
        
    # Set the color of the second sphere
    sphere2.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green color
      
      
     # Publish the first sphere
    shape1_pub.publish(sphere1)  
    # Publish the second sphere
    shape2_pub.publish(sphere2)
        
    rate.sleep()

if __name__ == '__main__':
    try:
        center_x1 = float(input("Enter x-coordinate of first sphere's center point: "))
        center_y1 = float(input("Enter y-coordinate of first sphere's center point: "))
        center_z1 = float(input("Enter z-coordinate of first sphere's center point: "))
        radius1 = float(input("Enter radius of the first sphere: "))
        
        center_x2 = float(input("Enter x-coordinate of second sphere's center point: "))
        center_y2 = float(input("Enter y-coordinate of second sphere's center point: "))
        center_z2 = float(input("Enter z-coordinate of second sphere's center point: "))
        radius2 = float(input("Enter radius of the second sphere: "))
        
        generate_two_spheres(center_x1, center_y1, center_z1, radius1, center_x2, center_y2, center_z2, radius2)
    except rospy.ROSInterruptException:
        pass
