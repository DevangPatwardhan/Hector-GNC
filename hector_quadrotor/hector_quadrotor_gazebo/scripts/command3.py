#!/usr/bin/env /bin/python3
import rospy
from geometry_msgs.msg import Twist

import math
import time


from geometry_msgs.msg import TwistStamped

def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    """Rotates the drone by a specified relative angle at a given angular speed."""
    pub = rospy.Publisher('/command/twist', TwistStamped, queue_size=10)
    
    rate = rospy.Rate(10)  # Publish commands at 10 Hz

    angular_speed_rad = math.radians(angular_speed_degree)  # Convert to radians
    sign = 1 if clockwise else -1  # Adjust for clockwise/counterclockwise

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = ''

    start_time = rospy.Time.now().to_sec()
    duration = abs(relative_angle_degree) / angular_speed_degree  # Calculate duration

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        if elapsed_time <= duration:
            twist.twist.angular.z = sign * angular_speed_rad
            pub.publish(twist)
        else:
            twist.twist.angular.z = 0.0
            pub.publish(twist)
            break

        rate.sleep()

def move(speed, distance, is_forward):
    print("2")
    
    
    

    direction = 1 if is_forward else -1  # Adjust for forward/backward

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = ''

    twist.twist.linear.x =direction*speed
    

    pub.publish(twist)
    duration = distance / speed  # Calculate duration
    rospy.sleep(duration)

    twist.twist.linear.x = 0.0
    twist.twist.linear.y = 0.0
    twist.twist.linear.z = 0.0
    twist.twist.angular.z = 0.0
    pub.publish(twist)
    print("3")
    rate.sleep()




def direction(speed, distance, left_or_right):
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = ''

    direction = 1 if left_or_right else -1

    twist.twist.linear.y = direction*speed
    pub.publish(twist)
    duration = distance / speed 
    rospy.sleep(duration)

    twist.twist.linear.x = 0.0
    twist.twist.linear.y = 0.0
    twist.twist.linear.z = 0.0
    twist.twist.angular.z = 0.0
    pub.publish(twist)
    rate.sleep()

def semicircle_cw():
    pubb = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    

    #radius = float(input("Enter circle radius (in meters): "))
    #angular_speed = float(input("Enter angular speed (in radians/second): "))
    radius=3
    angular_speed=3
    

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
            pubb.publish(twist)
        else:
        
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            pubb.publish(twist)
            break  # Exit the loop after completing the semicircle

        rate.sleep()

def semicircle_ccw():
    pubb = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    

    #radius = float(input("Enter circle radius (in meters): "))
    #angular_speed = float(input("Enter angular speed (in radians/second): "))
    radius=2
    angular_speed=2
    

    # Calculate duration for semicircle based on radius and angular speed
    duration = math.pi * radius / angular_speed

    start_time = rospy.Time.now().to_sec()  # Get starting time

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        if elapsed_time <= duration:  # Continue motion until duration is reached
            twist = Twist()
            twist.linear.x = radius * angular_speed  # Forward velocity for circular motion
            twist.angular.z = -angular_speed  # Angular velocity for rotation
            pubb.publish(twist)
        else:
        
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            pubb.publish(twist)
            break  # Exit the loop after completing the semicircle

        rate.sleep()

def motion(speed, distance, is_forward):
    pubb = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    
    

    direction = 1 if is_forward else -1  # Adjust for forward/backward

    twist = Twist()
    twist.linear.x = direction*speed  
    
    pubb.publish(twist)

    duration = distance / speed  # Calculate duration
    rospy.sleep(duration)

    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    pubb.publish(twist)
    
    rate.sleep()
    

if __name__ == '__main__':

    pub = rospy.Publisher('/command/twist', TwistStamped, queue_size=10)
    rospy.init_node('drone_motion_control', anonymous=True)
    rate = rospy.Rate(20)  # Publish commands at 10 Hz
    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = ''
    pub.publish(twist)
    rospy.sleep(0.1)
    twist.twist.linear.x = 0.0
    twist.twist.linear.y = 0.0
    twist.twist.linear.z = 0.0
    twist.twist.angular.z = 0.0
    pub.publish(twist)
    rate.sleep()

    
    print("1")
    
        # Example usage:
    
    motion(5.0, 20.0, True)
    print("11")
    motion(5.0, 20.0, True)
    print("111")
    semicircle_cw()
    print("1111")
    motion(5.0, 20.0, True)
    print("11111")
    semicircle_cw()
    rospy.sleep(0.2)
    
    
