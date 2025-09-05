#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion ,Vector3
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians, degrees
import math
from math import asin ,acos
import numpy as np

class TurtleBotGTG:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)

        self.turtle1_velocity_topic = '/hector_quadrotor_1/cmd_vel'
        self.turtle2_velocity_topic = '/hector_quadrotor_2/cmd_vel'
        self.relative_velocity = Twist()

         # Initialize turtle velocities
        self.turtle1_velocity = Twist()
        self.turtle2_velocity = Twist()       
        
        self.velocity_publisher_waffle = rospy.Publisher('/hector_quadrotor_1/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher_obstacle = rospy.Publisher('/hector_quadrotor_2/cmd_vel', Twist, queue_size=10)
        self.robot_waffle_subscriber = rospy.Subscriber('/hector_quadrotor_1/ground_truth/state', Odometry, self.get_turtlebot_pose_waffle)
        self.robot_obstacle_subscriber = rospy.Subscriber('/hector_quadrotor_2/ground_truth/state', Odometry, self.get_turtlebot_pose_obstacle)
        rospy.Subscriber(self.turtle1_velocity_topic, Twist, self.turtle1_velocity_callback)
        rospy.Subscriber(self.turtle2_velocity_topic, Twist, self.turtle2_velocity_callback)
        self.robot_pose_waffle = Point()
        self.robot_pose_obstacle = Point()
        self.goal_pose = Point()
        self.goal_pose_1 = Point()
        self.vel_msg_waffle = Twist()
        self.vel_msg_obstacle = Twist()
        self.distance_to_goal = 0.0
        self.distance_to_goal_1 = 0.0
        self.angle_to_goal = 0.0
        self.angle_to_goal_1 = 0.0
        self.goal_reached_threshold = 0.25
        self.goal_reached_threshold_1 = 0.30
        self.angular_velocity_scale = 0.5
        self.angular_velocity_scale_1 = 0.5
        self.Vrel_x = 0
        self.Vrel_y = 0
        #self.initial_yaw_obstacle = None

    def get_turtlebot_pose_waffle(self, data):
        self.robot_pose_waffle.x = data.pose.pose.position.x
        self.robot_pose_waffle.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_waffle.z) = self.euler_from_quaternion(*quaternion)

    def get_turtlebot_pose_obstacle(self, data):
        self.robot_pose_obstacle.x = data.pose.pose.position.x
        self.robot_pose_obstacle.y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_obstacle.z) = self.euler_from_quaternion(*quaternion)

    def turtle1_velocity_callback(self, msg):
        # Callback function for TurtleBot 1's velocity
        self.turtle1_velocity = msg


        # print("Turtle1 Vector3:", vector3_turtle1)       

    def turtle2_velocity_callback(self, msg):
        # Callback function for TurtleBot 2's velocity
        self.turtle2_velocity = msg

  

    def goal_movement(self):
        # Function to move the goal
        self.goal_pose.x = 5
        self.goal_pose.y = 0
        self.goal_pose_1.x = -5
        self.goal_pose_1.y = 0  
        #self.goal_pose.x = float(input("Enter goal_pose.x: "))
        #self.goal_pose.y = float(input("Enter goal_pose.y: "))
        #self.goal_pose_1.x = float(input("Enter goal_pose_1.x: "))
        #self.goal_pose_1.y = float(input("Enter goal_pose_1.y: "))
        nr=5
        #desired_orientation = 0.0
        # print("Vrel_x: ",self.Vrel_x)
        # print("Vrel_y: ",self.Vrel_y)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #self.vel_msg_obstacle.linear.x = 0.5 #read this
            #self.vel_msg_obstacle.angular.z = 0   #read this
            
            #self.velocity_publisher_obstacle.publish(self.vel_msg_obstacle)

            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose_waffle.x)**2 + (self.goal_pose.y - self.robot_pose_waffle.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose_waffle.y, self.goal_pose.x - self.robot_pose_waffle.x)
            angle_difference = self.angle_to_goal - self.robot_pose_waffle.z

            self.distance_to_goal_1 = sqrt((self.goal_pose_1.x - self.robot_pose_obstacle.x)**2 + (self.goal_pose_1.y - self.robot_pose_obstacle.y)**2)
            self.angle_to_goal_1 = atan2(self.goal_pose_1.y - self.robot_pose_obstacle.y, self.goal_pose_1.x - self.robot_pose_obstacle.x)
            angle_difference_1 = self.angle_to_goal_1 - self.robot_pose_obstacle.z

            # Ensure angle difference is within the range of -pi to pi
            if angle_difference_1 > math.pi:
                angle_difference_1 -= 2 * math.pi
            elif angle_difference_1 < -math.pi:
                angle_difference_1 += 2 * math.pi 

            
            # Adjust angular velocity for straight line movement
            angular_velocity_1 = self.angular_velocity_scale_1 * angle_difference_1

            self.vel_msg_obstacle.linear.x = 0.5  # Adjust linear velocity as needed
            self.vel_msg_obstacle.angular.z = angular_velocity_1
            self.velocity_publisher_obstacle.publish(self.vel_msg_obstacle)

            if self.distance_to_goal_1 < self.goal_reached_threshold_1:
                    self.vel_msg_obstacle.linear.x=0
                    self.vel_msg_obstacle.angular.z=0

            self.velocity_publisher_obstacle.publish(self.vel_msg_obstacle)


            vmag1 = self.turtle1_velocity.linear.x
            theta1 = self.robot_pose_waffle.z
            Vx1 = vmag1 * math.cos(theta1)
            Vy1 = vmag1 * math.sin(theta1)
            # print("Vx1: ",Vx1)
            # print("Vy1: ",Vy1)
            waffle_vector=Vector3()
            waffle_vector.x=Vx1
            waffle_vector.y=Vy1

            vmag2 = self.turtle2_velocity.linear.x
            theta2 = self.robot_pose_obstacle.z
            Vx2 = vmag2 * math.cos(theta2)
            Vy2 = vmag2 * math.sin(theta2)
            # print("Vx2: ",Vx2)
            # print("Vy2: ",Vy2)
            obs_vector=Vector3()
            obs_vector.x=Vx2
            obs_vector.y=Vy2

            self.Vrel_x = Vx1 - Vx2
            self.Vrel_y = Vy1 - Vy2
            # print("Vrel_x: ",self.Vrel_x)
            # print("Vrel_y: ",self.Vrel_y)
            #print("Vx2: ",Vx2,"Vy2: ",Vy2,"Vrel_x: ",self.Vrel_x,"Vrel_y: ",self.Vrel_y,"yaw: ",self.robot_pose_obstacle.z)


            # Distance between waffle-agent and the obstacle
            do = sqrt(((self.robot_pose_obstacle.x - self.robot_pose_waffle.x)**2) + ((self.robot_pose_obstacle.y - self.robot_pose_waffle.y)**2))

            self.relative_pos_vector= Vector3()
            self.relative_pos_vector.x=(self.robot_pose_obstacle.x - self.robot_pose_waffle.x)
            self.relative_pos_vector.y=(self.robot_pose_obstacle.y - self.robot_pose_waffle.y)
            self.relative_pos_vector.z=()
            
            # print("relative_pos_vector.x: ",self.relative_pos_vector.x)
            # print("relative_pos_vector.y: ",self.relative_pos_vector.y)
            # # Radius of the obstacle
            dmin = 1.500

            #print("dmin:",dmin)
            # print("do:",do)  
            # print("nr: ",nr) 
            if do==0:
                dmin_by_do=0

            else:
                dmin_by_do=(dmin / do)
            #print("dmin_do",dmin_by_do)

            # Calculate the angle in radians
            lamdaa_radians = np.arcsin(dmin_by_do)

            # print("lamdaa_radians:", lamdaa_radians)

            # Convert radians to degrees if necessary
            lambdaa = math.degrees(lamdaa_radians)
            #print("lambda::", lambdaa)

            # Dot product of relative velocity vector and relative position vector
            dot_product = (self.Vrel_x * self.relative_pos_vector.x) + (self.Vrel_y * self.relative_pos_vector.y)
            #print("dot_product:", dot_product)
            #dot_product = (Vrel_x * relative_pos_vector.x) + (Vrel_y * relative_pos_vector.y)
            # Magnitude of relative velocity vector
            magnitude_rel_vel = sqrt(((self.Vrel_x)**2) + ((self.Vrel_y)**2))
            #sqrt((self.relative_velocity.linear.x**2) + (self.relative_velocity.linear.y**2))
            #print("magnitude_rel_vel:", magnitude_rel_vel)
            # Magnitude of relative position vector
            magnitude_rel_pos = sqrt((self.relative_pos_vector.x**2) + (self.relative_pos_vector.y**2))
            #print("magnitude_rel_pos:", magnitude_rel_pos)
            # Angle between relative velocity vector and relative position vector
            #alphaa_radians = np.arccos(dot_product / (magnitude_rel_vel * magnitude_rel_pos))
            #cos_theta = dot_product / (magnitude_rel_vel * magnitude_rel_pos)
            
            if magnitude_rel_vel == 0 or magnitude_rel_pos == 0:
                #print("One of the magnitudes is zero. Cannot compute cosine theta.")
                thetaa = 0  # Set a default value or handle it as required
            else:
                # Calculate the cosine of the angle
                thetaa = np.arccos(dot_product / (magnitude_rel_vel * magnitude_rel_pos))

            #print("thetaa_rad:", thetaa)

            # Calculate alpha_radians safely
            #alphaa_radians = np.arccos(cos_theta)

            alphaa = math.degrees(thetaa)
            # print("alphaa:", alphaa)

            # Heading of the waffle
            psi_waffle = self.robot_pose_waffle.z   
            #print("psi_waffle:", psi_waffle)
            psi_waffle_deg = math.degrees(psi_waffle)
            #print("psi_waffle_degrees:", psi_waffle_deg)

            # Alphaa + lambdaa and Alphaa - lambdaa
            alphaa_plus_lambdaa = alphaa + lambdaa
            alphaa_minus_lambdaa = alphaa - lambdaa

            # Ensure angle difference is within the range of -pi to pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            # Commands for linear and angluar velocity
            #linear_velocity = 0.5
            angular_velocity = self.angular_velocity_scale * angle_difference
            #print("angular velocity: ", angular_velocity)
            # Enters into collision avoidance maneuver when alphaa < lambdaa, because the relative velocity lies in the collision cone

            # print("Linear velocity: ", self.turtle1_velocity.linear.x)
            # print("Angular velocity: ", self.turtle1_velocity.angular.z)
            print("do: ",do,"lambdaa: ",lambdaa,"alphaa: ",alphaa)
            if (alphaa < lambdaa) and (do<nr):  
                print("Collision avoidance maneuver activated  ====================> @do: ",do)

                # alphaa - angle between relative velocity vector and relative position vector
                # is less than lambdaa - half angle of the collision cone
                # and do - distance between waffle and obstacle less than neighbouring region
                
   
            # Steer commands based on alphaa, lambdaa and psi_waffle
                if ((psi_waffle_deg) < (alphaa_plus_lambdaa)) and ((psi_waffle_deg) > (alphaa)):
                    # steer to the left
                    self.vel_msg_waffle.linear.x=0.5
                    self.vel_msg_waffle.angular.z=((angular_velocity)+0.5)
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("Steering left")
                    print(("do: ",do),("nr: ",nr),("alpha: ",alphaa),("lambdaa: ",lambdaa))

                elif ((psi_waffle_deg) > (alphaa_minus_lambdaa)) and ((psi_waffle_deg) < (alphaa)):
                    # steer to the right
                     self.vel_msg_waffle.linear.x=0.5
                     self.vel_msg_waffle.angular.z=(-(angular_velocity)-0.5)
                     #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                     print("Steering right")
                     
                
            elif self.distance_to_goal > self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x= 0.5
                    self.vel_msg_waffle.angular.z=self.angular_velocity_scale * angle_difference
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("marching to goal")

                
            elif self.distance_to_goal <= self.goal_reached_threshold:
                    self.vel_msg_waffle.linear.x=0
                    self.vel_msg_waffle.angular.z=0
                    #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
                    print("Goal reached")

            #self.vel_msg_waffle.linear.x =  0.5
            #self.vel_msg_waffle.angular.z=angular_velocity
            #self.velocity_publisher_waffle.publish(self.vel_msg_waffle)

            self.velocity_publisher_waffle.publish(self.vel_msg_waffle)
            rate.sleep()

    def euler_from_quaternion(self, x, y, z, w):
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

if __name__ == '__main__':
    try:
        turtlebot_gtg = TurtleBotGTG()
        turtlebot_gtg.goal_movement()
        #rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
