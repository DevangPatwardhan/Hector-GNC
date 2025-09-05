#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped

def motion_control():
    pub = rospy.Publisher('/hector_quadrotor_2/command/twist', TwistStamped, queue_size=10)
    rospy.init_node('drone_motion_control', anonymous=True)
    rate = rospy.Rate(10)  # Publish commands at 10 Hz

    while not rospy.is_shutdown():
        # Choose a motion command
        command = input("Enter motion command (forward, backward, left, right, up, down, cw, ccw, stop): ")
        distance = float(input("Enter distance in meter: "))

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = ''

        if command == "forward":
            twist.twist.linear.x = 1.0
        elif command == "backward":
            twist.twist.linear.x = -1.0
        elif command == "left":
            twist.twist.linear.y = 1
            #twist.twist.angular.z = 8
            
        elif command == "right":
            twist.twist.linear.y = -1.0
        elif command == "up":
            twist.twist.linear.z = 1.0
        elif command == "down":
            twist.twist.linear.z = -1.0
        elif command == "cw":  # Clockwise rotation
            twist.twist.angular.z = 1.0
        elif command == "ccw":  # Counterclockwise rotation
            twist.twist.angular.z = -1.0
        elif command == "stop":
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.z = 0.0
        else:
            print("Invalid command.")
            continue
        
        pub.publish(twist)
        duration = distance / 1.0 
        rospy.sleep(duration)

        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.z = 0.0
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        motion_control()
    except rospy.ROSInterruptException:
        pass

