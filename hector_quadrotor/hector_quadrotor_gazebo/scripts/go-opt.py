#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty

x=0
y=0
yaw=0

def poseCallback(data):
    
    global x
    global y, yaw

    robot_pose = Point()
    x= data.pose.pose.position.x
    y= data.pose.pose.position.y
    #yaw = data.pose.pose.position.z
    quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    (_, _, yaw) = euler_from_quaternion(*quaternion)

   



def go_to_goal(x_goal, y_goal):
    
    
    global a,b,c
    a = x
    b = y
    c = yaw
     

    velocity_message = Twist()
    cmd_vel_topic='cmd_vel'

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear


        K_angular = 0.5
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        
        print ("lin_vel:",velocity_message.linear.x)
        print ("ang_vel:",velocity_message.angular.z)
        print ('x=', format(x), 'y=',format(y),"yaw",format(yaw))


        if (distance <0.01):

            break


def euler_from_quaternion( x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z




if __name__ == '__main__':
    try:
        
        rospy.init_node('motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/ground_truth/state"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 
        time.sleep(2)
        rate = rospy.Rate(50)  # 10 Hz

        
        go_to_goal(1.0, 1.0)
        print ("pose callback:")
        print ('init x = {}'.format(a)) 
        print ('init y = {}'.format(b))
        print ('init yaw = {}'.format(c))
        #setDesiredOrientation(math.radians(90))
        rate.sleep()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
