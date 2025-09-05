#!/usr/bin/env /bin/python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def calculate_duration(arc_length, omega):
    return arc_length / omega


def semicircle_motion(radius, theta_degrees):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drone_semicircle_motion', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    # Convert angle to radians
    theta_radians = math.radians(theta_degrees)

    # Calculate arc length
    arc_length = radius * theta_radians

    # Prompt user for desired angular speed
    #omega = float(input("Enter desired angular speed (radians/second): "))
    omega= 3

    # Calculate duration
    duration = calculate_duration(arc_length, omega)

    # Timer setup
    start_time = rospy.Time.now().to_sec()  # Get starting time
    elapsed_time = 0

    while elapsed_time < duration:
        # Calculate current time step
        current_time = rospy.Time.now().to_sec()
        dt = current_time - start_time

        # Update elapsed time
        elapsed_time += dt

        # Calculate angular velocity at current time step
        omega_t = elapsed_time / duration

        # Calculate linear velocity at current time step
        v_t = omega_t * radius

        # Send velocity commands to drone control system
        twist = Twist()
        twist.linear.x = v_t  # Forward velocity for circular motion
        twist.angular.z = omega_t  # Angular velocity for rotation
        pub.publish(twist)

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    pub.publish(twist) 
    rate.sleep()  

    

if __name__ == '__main__':
    try:
        semicircle_motion(4,180)
        semicircle_motion(4,180)
    except rospy.ROSInterruptException:
        pass

