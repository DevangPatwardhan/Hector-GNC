#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point , Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import math
from visualization_msgs.msg import Marker

class MazebotGTG:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.robot_pose = Quaternion()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angular_velocity_scale = 0.5
        self.goal_reached_threshold = 0.1

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        self.robot_pose.z = data.pose.pose.position.z
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose.w) = self.euler_from_quaternion(*quaternion)

        # Publish marker at the center of the robot
        self.publish_marker()

    def goal_movement(self):
        self.goal_pose.x = float(input("Enter goal_pose.x: "))
        self.goal_pose.y = float(input("Enter goal_pose.y: "))

        while not rospy.is_shutdown():
            K_linear = 0.5
            
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
            
            current_yaw = self.robot_pose.w
            angle_difference = self.angle_to_goal - current_yaw

            # Normalize angle difference to be between -pi and pi
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi
            
            angular_velocity = self.angular_velocity_scale * angle_difference

            self.vel_msg.linear.x = 0.5  # constant linear speed
            self.vel_msg.angular.z = angular_velocity
            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0  # stop robot
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal!")
                break

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_center"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_pose.x
        marker.pose.position.y = self.robot_pose.y
        marker.pose.position.z = self.robot_pose.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # fully opaque
        marker.color.r = 1.0  # red
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_publisher.publish(marker)

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

def main():
    rospy.init_node('go_to_goal', anonymous=True)
    mazebot_gtg = MazebotGTG()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        mazebot_gtg.goal_movement()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




from geometry_msgs.msg import Quaternion
quaternion = Quaternion()
quaternion.x = 0.0
quaternion.y = 0.0
quaternion.z = 0.0
quaternion.w = 1.0


