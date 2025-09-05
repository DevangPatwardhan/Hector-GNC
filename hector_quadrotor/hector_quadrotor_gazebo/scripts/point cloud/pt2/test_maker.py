#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker

class BoundingBoxPublisher:
    def __init__(self):
        rospy.init_node('bounding_box_publisher', anonymous=True)
        self.marker_pub = rospy.Publisher('/bounding_box', Marker, queue_size=10)

    def publish_bounding_box(self, box):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bounding_box"
        marker.id = 0  # Always use the same ID for a single marker
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (box['x_min'] + box['x_max']) / 2
        marker.pose.position.y = (box['y_min'] + box['y_max']) / 2
        marker.pose.position.z = (box['z_min'] + box['z_max']) / 2
        marker.scale.x = box['x_max'] - box['x_min']
        marker.scale.y = box['y_max'] - box['y_min']
        marker.scale.z = box['z_max'] - box['z_min']
        marker.color.a = 0.5  # Transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

def main():
    rospy.init_node('bounding_box_publisher', anonymous=True)
    bounding_box_publisher = BoundingBoxPublisher()

    # Dummy bounding box values
    dummy_boxes = [
        {'x_min': -1.0, 'y_min': -1.0, 'z_min': -1.0, 'x_max': 1.0, 'y_max': 1.0, 'z_max': 1.0},
        {'x_min': -2.0, 'y_min': -2.0, 'z_min': -2.0, 'x_max': 2.0, 'y_max': 2.0, 'z_max': 2.0},
        {'x_min': -3.0, 'y_min': -3.0, 'z_min': -3.0, 'x_max': 3.0, 'y_max': 3.0, 'z_max': 3.0}
    ]

    rate = rospy.Rate(1)  # Publish at 1Hz
    while not rospy.is_shutdown():
        for box in dummy_boxes:
            bounding_box_publisher.publish_bounding_box(box)
        rate.sleep()

if __name__ == '__main__':
    main()

