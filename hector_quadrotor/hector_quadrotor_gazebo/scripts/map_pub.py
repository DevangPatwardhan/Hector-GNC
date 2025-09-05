

#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

def generate_map():
    #rospy.init_node("map_publisher",anonymous=True)
    #pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    #rate = rospy.Rate(10)  # 1 Hz publishing rate
 

    map_width = 3
    map_height = 3

   
    map_data = [0,0,0,0,0,0,0,0,0]
   # for i in range(map_height):
     #   row = []
     #   for j in range(map_width):
     #       row.append(0)
     #   map_data.append(row)

    # Option 3: Using list multiplication (uncomment the following line)
    # map_data = [[0] * map_width] * map_height

    #map_data[1][1] = 100  # Set obstacle in the center

    map_msg = OccupancyGrid()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"
    map_msg.info.width = map_width
    map_msg.info.height = map_height
    map_msg.info.resolution = 1.0  # 1 meter per cell
    map_msg.info.origin.position.x = -map_width / 2.0
    map_msg.info.origin.position.y = -map_height / 2.0
    map_msg.info.origin.orientation.w = 1.0

    map_msg.data = sum(map_data, [])  # Flatten map using sum

    return map_msg

if __name__ == "__main__":
    rospy.init_node("map_publisher",anonymous=True)
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    rate = rospy.Rate(10)  # 1 Hz publishing rate

    while not rospy.is_shutdown():
        map_msg = generate_map()
        pub.publish(map_msg)
        rate.sleep()

