#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CylinderFollowing:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_data_processing, queue_size=1)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.robot_velocity = Twist()
        self.regions = {'left': [], 'mid': [], 'right': []}

    def laser_data_processing(self, data):
        self.regions = {'left': data.ranges[0:30], 'mid': data.ranges[30:60], 'right': data.ranges[60:90]}
        region_left_min = min(min(self.regions['left']), 6)
        region_mid_min = min(min(self.regions['mid']), 6)
        region_right_min = min(min(self.regions['right']), 6)
        print("/region_left_min/", region_left_min, "/region_mid_min/", region_mid_min, "/region_right_min/", region_right_min)

        # Define distance threshold for following the cylinder
        cylinder_distance_threshold = 1.0  # Adjust as needed

        if region_mid_min > cylinder_distance_threshold:
            # Far from the cylinder, turn towards it
            self.robot_velocity.linear.x = 0.5
            self.robot_velocity.angular.z = 0.2
        elif region_mid_min < cylinder_distance_threshold:
            # Close to the cylinder, turn away from it
            self.robot_velocity.linear.x = 0.5
            self.robot_velocity.angular.z = -0.2
        else:
            # Maintain distance from the cylinder, move forward
            self.robot_velocity.linear.x = 0.5
            self.robot_velocity.angular.z = 0.0

        self.velocity_publisher.publish(self.robot_velocity)

def main(args=None):
    rospy.init_node('cylinder_following')
    cylinder_follower = CylinderFollowing()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Exiting")

if __name__ == '__main__':
    main()
