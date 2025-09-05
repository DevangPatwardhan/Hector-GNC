#!/usr/bin/env python3



import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from math import floor

def occupancy_grid_publisher():
    rospy.init_node('occupancy_grid_publisher', anonymous=True)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1) # 1Hz

    width = 5
    height = 5
    resolution = 1.0
    map_data = [-1] * (width * height)  # -1: unknown, 0: free, 100: occupied

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.resolution = resolution
    occupancy_grid.info.origin.position.x = 0.0
    occupancy_grid.info.origin.position.y = 0.0
    occupancy_grid.info.origin.position.z = 0.0
    occupancy_grid.info.origin.orientation.x = 0.0
    occupancy_grid.info.origin.orientation.y = 0.0
    occupancy_grid.info.origin.orientation.z = 0.0
    occupancy_grid.info.origin.orientation.w = 1.0

    while not rospy.is_shutdown():
        # For demonstration purposes, set some cells as occupied
        map_data[6] = 100
        map_data[7] = 100
        map_data[8] = 100
        map_data[11] = 100
        map_data[12] = 100

        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.data = map_data

        pub.publish(occupancy_grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        occupancy_grid_publisher()
    except rospy.ROSInterruptException:
        pass


