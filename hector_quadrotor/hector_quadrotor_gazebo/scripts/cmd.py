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
    """Moves the drone forward or backward by a specified distance at a given speed."""
    pub= rospy.Publisher('/command/twist', TwistStamped, queue_size=10)
    
    rate = rospy.Rate(10)  # Publish commands at 10 Hz

    direction = 1 if is_forward else -1  # Adjust for forward/backward

    twist = TwistStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = ''

    twist.twist.linear.x = direction * speed
    duration = distance / speed  # Calculate duration

    pub.publish(twist)
    rospy.sleep(duration)

    twist.twist.linear.x = 0.0  # Stop movement
    pub.publish(twist)

def direction(speed, distance, left_or_right):
    """Moves the drone left or right by a specified distance at a given speed."""
    # Implementation similar to move() function, with adjustments for linear.y
    # ... (implement the logic for left/right movement here)

if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    try:
    
        # Example usage:
        move(1.0, 2.0, True)  # Move forward 2 meters at speed 1.0
        rotate(30, 90, False)  # Rotate counterclockwise 90 degrees at 30 degrees/second
        # Add more commands as needed
    except rospy.ROSInterruptException:
        pass

