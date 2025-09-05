#!/usr/bin/env /bin/python3
import rospy
from geometry_msgs.msg import Twist

def circle_motion():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('drone_circle_motion', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    radius = float(input("Enter circle radius (in meters): "))
    angular_speed = float(input("Enter angular speed (in radians/second): "))

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = radius * angular_speed  # Forward velocity for circular motion
        twist.angular.z = angular_speed  # Angular velocity for rotation

        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle_motion()
    except rospy.ROSInterruptException:
        pass

