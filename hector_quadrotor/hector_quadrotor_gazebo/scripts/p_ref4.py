#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import math
import sys

class MazebotGTG:
    def __init__(self):
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.obstacle_avoidance)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angle_offset = 0
        self.angular_velocity_scale =0.5
        self.goal_reached_threshold = 0.1 
        self.obstacle_threshold = 1.5  # Distance to consider an obstacle
        self.obstacle_detected = False 
        #self.regions={'left': [], 'mid':[],  'right':[]    }
        self.region_left_min=0
        self.region_mid_min =0
        self.region_right_min=0

    def get_mazebot_pose(self, data):
        
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.z) = self.euler_from_quaternion(*quaternion)
        #print("current yaw:",self.robot_pose.z)

    def obstacle_avoidance(self, data):
    
        #global region_left_min, region_mid_min ,region_right_min
        self.regions_min = {
            'left': min(min(data.ranges[0:30]), 6),
            'mid': min(min(data.ranges[30:60]), 6),
            'right': min(min(data.ranges[60:90]), 6)
        }
        self.regions = { 'left' : data.ranges[0:30],'mid' : data.ranges[30:60], 'right' : data.ranges[60:90] }
        self.region_left_min=min( min(self.regions['left']) , 6 ) ; 
        self.region_mid_min = min( min(self.regions['mid']) ,6 ); 
        self.region_right_min=min ( min(self.regions['right']) , 6 ); 
        #print(" /region_left_min/ ",self.region_left_min , "/region_mid_min/ ",self.region_mid_min , " /region_right_min/ ",self.region_right_min)

        if min(self.regions_min.values()) < self.obstacle_threshold:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        
 

    def goal_movement(self):
        global x
        global y, z ,current_y

        x = self.robot_pose.x
        y = self.robot_pose.y
        z = self.robot_pose.z

        self.goal_pose.x = float(input("Enter goal_pose.x ): "))
        self.goal_pose.y = float(input("Enter goal_pose.y ): "))
        #self.angle_offset = float(input("Enter angle_offset ): "))



        while not rospy.is_shutdown():
            
            #K_linear = 0.5
            
            
            if self.obstacle_detected:
                #self.vel_msg.linear.x = 0.0
                #self.vel_msg.angular.z = 0.8  # Rotate to avoid obstacle
                print('obstacle detected')
                duration = 1.0 / 1.0

                if(self.region_left_min!=6 and self.region_mid_min!=6 and self.region_right_min!=6):
                     
              
                    self.vel_msg.linear.y  = -1
                    #self.vel_msg.linear.x  = 3
                    #self.vel_msg.angular.z = -1

                    self.velocity_publisher.publish(self.vel_msg)

                    rospy.sleep(duration)
                    self.vel_msg.linear.x  = 0
                    self.vel_msg.linear.y  = 0
                    self.vel_msg.angular.z =0
                    self.velocity_publisher.publish(self.vel_msg)

                elif(self.region_left_min!=6 and self.region_mid_min==6 and self.region_right_min!=6):

                    print("Case 101 : Straight Movement")
                    self.vel_msg.linear.x  = 3#0.5
                    self.vel_msg.angular.z = 0.0
                    self.velocity_publisher.publish(self.vel_msg)

                    rospy.sleep(duration)
                    self.vel_msg.linear.x  = 0
                    self.vel_msg.linear.y  = 0
                    self.vel_msg.angular.z =0
                    self.velocity_publisher.publish(self.vel_msg)

                elif(self.region_left_min==6 and self.region_mid_min!=6 and self.region_right_min!=6):
                    print("Case 011 : Take Left Turn")
                    self.vel_msg.linear.y  = 1
                    
                    self.velocity_publisher.publish(self.vel_msg)

                    rospy.sleep(duration)
                    self.vel_msg.linear.x  = 0
                    self.vel_msg.linear.y  = 0
                    self.vel_msg.angular.z =0
                    self.velocity_publisher.publish(self.vel_msg)

                elif(self.region_left_min!=6 and self.region_mid_min!=6 and self.region_right_min==6):
                    print("Case 110 : Take Right Turn")
                    self.vel_msg.linear.y  = -1
                    #self.vel_msg.linear.x  = 3
                    #self.vel_msg.angular.z = -1

                    self.velocity_publisher.publish(self.vel_msg)

                    rospy.sleep(duration)
                    self.vel_msg.linear.x  = 0
                    self.vel_msg.linear.y  = 0
                    self.vel_msg.angular.z =0
                    self.velocity_publisher.publish(self.vel_msg)

                    
                elif(self.region_left_min==6 and self.region_mid_min!=6 and self.region_right_min==6):
                    print("Case 010 : Take Right Turn and straight")
                    self.vel_msg.linear.y  = -1#0.3
                    
                    self.velocity_publisher.publish(self.vel_msg)
                    
                    rospy.sleep(duration)
                    self.vel_msg.linear.x  = 0
                    self.vel_msg.linear.y  = 0
                    self.vel_msg.angular.z =0
                    self.velocity_publisher.publish(self.vel_msg)


            else:
                
                
                self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
                
                self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
                current_yaw = self.robot_pose.z
                angle_difference = self.angle_to_goal - current_yaw
                angle_difference_deg=math.degrees(angle_difference)
                #print('current angle:',angle_difference_deg)

                #self.angle_to_turn = self.angle_to_goal - self.robot_pose.z
                print('1.DG:',format(self.distance_to_goal),'AG:',format(self.angle_to_goal))

                # angle difference is within the range of -pi to pi
                #linear_speed = self.distance_to_goal * K_linear
                if angle_difference > 3.14159:
                
                    angle_difference -= 2 * 3.14159
                    print('in if current angle:',format(math.degrees(angle_difference)))
                elif angle_difference < -3.14159:
                    
                    angle_difference += 2 * 3.14159
                    print('elif current angle:',format(math.degrees(angle_difference)))
        
                angular_velocity = self.angular_velocity_scale * angle_difference

        
                self.vel_msg.linear.x = 0.5#linear_speed
                self.vel_msg.angular.z = angular_velocity
                self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                
                

                self.vel_msg.linear.x = 0 # Constant linear velocity
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                print ('x=', format(self.robot_pose.x), 'y=',format(self.robot_pose.y),"yaw",format(self.robot_pose.z))
                rospy.loginfo("Reached the goal!")

                break

            
            
        

    def euler_from_quaternion(self, x, y, z, w):
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

def main(args=None):
    rospy.init_node('go_to_goal', anonymous=True)
    mazebot_gtg = MazebotGTG()
    rate = rospy.Rate(30)  # 10 Hz
    while not rospy.is_shutdown():
        mazebot_gtg.goal_movement()
        print ("pose callback:")
        print ('init x = {}'.format(x)) 
        print ('init y = {}'.format(y))
        print ('init yaw = {}'.format(z))

        rate.sleep()

if __name__ == '__main__':
    main()

