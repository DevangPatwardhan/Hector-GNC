#!/usr/bin/env /bin/python3
import rospy
from geometry_msgs.msg import Twist
import math

def semicircle_motion():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drone_semicircle_motion', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    radius = float(input("Enter circle radius (in meters): "))
    angular_speed = float(input("Enter angular speed (in radians/second): "))

    # Calculate duration for semicircle based on radius and angular speed
    duration = math.pi * radius / angular_speed

    start_time = rospy.Time.now().to_sec()  # Get starting time

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        if elapsed_time <= duration:  # Continue motion until duration is reached
            twist = Twist()
            twist.linear.x = radius * angular_speed  # Forward velocity for circular motion
            twist.angular.z = angular_speed  # Angular velocity for rotation
            pub.publish(twist)
        else:
        
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            break  # Exit the loop after completing the semicircle

        rate.sleep()

if __name__ == '__main__':
    try:
        semicircle_motion()
    except rospy.ROSInterruptException:
        pass

