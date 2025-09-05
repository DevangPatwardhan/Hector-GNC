#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import tf

class ObstacleDetection:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.listener = tf.TransformListener()

        rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/ground_truth/state", Odometry, self.get_robot_pose)

        self.robot_pose = np.array([0.0, 0.0, 0.0])

    def get_robot_pose(self, data):
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1] = data.pose.pose.position.y
        self.robot_pose[2] = data.pose.pose.position.z

    def pointcloud_callback(self, msg):
        try:
            (trans, rot) = self.listener.lookupTransform('/base_link', '/velodyne', rospy.Time(0))
            transform_matrix = self.listener.fromTranslationRotation(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            return

        points = np.array(list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))

        if len(points) == 0:
            rospy.logwarn("No points received")
            return

        # Transform points to the robot frame
        points = np.dot(points, transform_matrix[:3, :3].T) + transform_matrix[:3, 3]

        # Filter the point cloud
        filtered_cloud = self.filterCloud(points)

        # Separate point cloud into ground and obstacle clouds
        ground_cloud, obstacle_cloud = self.separateClouds(filtered_cloud)

        # Cluster obstacles
        obstacle_clusters = self.clustering(obstacle_cloud)

        # Compute bounding boxes for each cluster
        obstacle_boxes = self.computeBoundingBoxes(obstacle_clusters)

        # Visualize markers for obstacles and their bounding boxes
        self.visualizeMarkers(obstacle_clusters, obstacle_boxes)

    def filterCloud(self, points):
        # Filter the point cloud here, e.g., by downsampling, focusing on a specific region, and removing points corresponding to the roof of the vehicle
        # Example filtering (downsampling every 10th point)
        filtered_points = points[::10]
        return filtered_points

    def separateClouds(self, points):
        # Here you would separate the point cloud into ground and obstacle clouds based on some criteria, such as inliers or height threshold
        # For demonstration purposes, we'll just split the cloud into two halves along the z-axis
        ground_cloud = points[points[:, 2] < 0.5]
        obstacle_cloud = points[points[:, 2] >= 0.5]
        return ground_cloud, obstacle_cloud

    def clustering(self, obstacle_cloud):
        # Perform clustering to group points in the obstacle cloud into clusters representing individual obstacles
        # For demonstration purposes, we'll use a simple clustering approach (e.g., DBSCAN)
        clusters = []
        # Your clustering logic goes here
        return clusters

    def computeBoundingBoxes(self, obstacle_clusters):
        # Compute bounding boxes for each obstacle cluster
        # For demonstration purposes, we'll just return placeholder boxes
        obstacle_boxes = []
        for cluster in obstacle_clusters:
            # Compute bounding box for each cluster
            bbox = [0, 0, 0, 1, 1, 1]  # Placeholder for bounding box [min_x, min_y, min_z, max_x, max_y, max_z]
            obstacle_boxes.append(bbox)
        return obstacle_boxes

    def visualizeMarkers(self, obstacle_clusters, obstacle_boxes):
        # Visualize markers for obstacles and their bounding boxes
        for i, cluster in enumerate(obstacle_clusters):
            # Publish marker for each cluster
            self.publish_marker(cluster.centroid, i, obstacle_boxes[i])

    def publish_marker(self, centroid, idx, bbox):
        # Publish markers for obstacles and their bounding boxes
        # Marker for obstacle
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = "base_link"
        obstacle_marker.header.stamp = rospy.Time.now()
        obstacle_marker.ns = "obstacles"
        obstacle_marker.id = idx
        obstacle_marker.type = Marker.SPHERE
        obstacle_marker.action = Marker.ADD
        obstacle_marker.pose.position.x = centroid[0]
        obstacle_marker.pose.position.y = centroid[1]
        obstacle_marker.pose.position.z = centroid[2]
        obstacle_marker.pose.orientation.w = 1.0
        obstacle_marker.scale.x = 0.2
        obstacle_marker.scale.y = 0.2
        obstacle_marker.scale.z = 0.2
        obstacle_marker.color.r = 1.0
        obstacle_marker.color.g = 0.0
        obstacle_marker.color.b = 0.0
        obstacle_marker.color.a = 1.0
        self.marker_pub.publish(obstacle_marker)

        # Marker for bounding box
        box_marker = Marker()
        box_marker.header.frame_id = "base_link"
        box_marker.header.stamp = rospy.Time.now()
        box_marker.ns = "bounding_boxes"
        box_marker.id = idx
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        box_marker.pose.position.x = (bbox[0] + bbox[3]) / 2
        box_marker.pose.position.y = (bbox[1] + bbox[4]) / 2
        box_marker.pose.position.z = (bbox[2] + bbox[5]) / 2
        box_marker.pose.orientation.w = 1.0
        box_marker.scale.x = bbox[3] - bbox[0]
        box_marker.scale.y = bbox[4] - bbox[1]
        box_marker.scale.z = bbox[5] - bbox[2]
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.5
        self.marker_pub.publish(box_marker)

if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node", anonymous=True)
    detector = ObstacleDetection()
    rospy.spin()

