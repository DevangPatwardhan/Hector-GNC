#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from math import sin, cos, pi

class ARDronePathController:
    def __init__(self):
        rospy.init_node('ardrone_path_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/land', Empty, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5 # rad/s
        self.move_duration = 5.0  # seconds
        self.angular_duration = pi  # radians (180 degrees)

    def takeoff(self):
        takeoff_msg = Empty()
        self.takeoff_pub.publish(takeoff_msg)
        rospy.sleep(3)  

    def land(self):
        land_msg = Empty()
        self.land_pub.publish(land_msg)
        rospy.sleep(3)  

    def move_forward(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = 5
        twist_cmd.angular.z = 0.0
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(4):
            self.cmd_vel_pub.publish(twist_cmd)
            self.rate.sleep()

    def move_in_semircle_ccw(self):
        twist_cmd = Twist()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(self.angular_duration / self.angular_speed):
            twist_cmd.linear.x = self.linear_speed
            twist_cmd.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist_cmd)
            self.rate.sleep()
        # Stop rotation
        twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_cmd)

    def move_in_semircle_cw(self):
        twist_cmd = Twist()
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(self.angular_duration / self.angular_speed):
            twist_cmd.linear.x = self.linear_speed
            twist_cmd.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(twist_cmd)
            self.rate.sleep()
        # Stop rotation
        twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_cmd)   

    def stop(self):
        twist_cmd = Twist()
        
        twist_cmd.linear.x = 0
        twist_cmd.linear.y = 0
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        self.rate.sleep()
             


    def run(self):
          
        
        self.move_forward()

        self.stop()
     
        self.move_in_semircle_ccw()

        self.stop()
	
        self.move_forward()

        self.stop()
		
        self.move_in_semircle_ccw() 

        self.stop()      
              
        self.move_forward()
        
        self.stop()

        
if __name__ == '__main__':
    controller = ARDronePathController()
    controller.run()

