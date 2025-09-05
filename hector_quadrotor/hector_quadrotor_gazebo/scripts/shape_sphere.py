#!/usr/bin/env python3
#!/usr/bin/env python


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from math import pi, sin, cos

def generate_cone():
    # Initialize ROS node
    rospy.init_node('cone_generator_node', anonymous=True)
    
    # Create publisher for the cone
    cone_pub = rospy.Publisher('cone', Marker, queue_size=10)
    
    # Set rate for publishing shapes
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Create a Marker message for the cone
        cone = Marker()
        cone.header.frame_id = "map"  # Adjust frame_id as necessary
        cone.header.stamp = rospy.Time.now()
        cone.ns = "cone"
        cone.id = 0
        cone.type = Marker.TRIANGLE_LIST
        cone.action = Marker.ADD
        cone.pose.orientation.w = 1.0
        cone.scale.x = 1.0
        cone.scale.y = 1.0
        cone.scale.z = 1.0
        cone.color = ColorRGBA(1.0, 0.5, 1.0, 1.0)  # Orange color, adjust as needed
        
        # Define the parameters for the cone
        radius = 1.0  # Base radius of the cone
        height = 2.0  # Height of the cone
        angle_step = 15  # Degrees to step around the circle
        
        # Calculate points for the cone
        for angle in range(0, 360, angle_step):
            # Calculate x, y for the current angle
            x = radius * cos(angle * pi / 180)
            y = radius * sin(angle * pi / 180)
            
            # Add points to create the base of the cone
            #cone.points.append(Point(x, y, 0))  # Base point
            cone.points.append(Point(0, 0, height))  # Apex of the cone
            cone.points.append(Point(radius * cos((angle + angle_step) * pi / 180), radius * sin((angle + angle_step) * pi / 180), 0))  # Next base point
        
        # Publish the cone
        cone_pub.publish(cone)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_cone()
    except rospy.ROSInterruptException:
        pass
