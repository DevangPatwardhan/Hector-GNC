#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/ground_truth/state", Odometry, self.get_robot_pose)
        self.marker_pub = rospy.Publisher('/bounding_boxes', MarkerArray, queue_size=10)

        self.robot_pose = None

    def get_robot_pose(self, msg):
        self.robot_pose = msg.pose.pose

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to Open3D point cloud
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        print(f"Number of points in the original point cloud: {len(points_list)}")

        if len(points_list) == 0:
            rospy.logwarn("Received empty point cloud")
            return

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points_list))
        
        # Process the point cloud
        filtered_cloud = self.filter_cloud(cloud)
        ground_cloud = self.segment_ground(filtered_cloud)
        clusters = self.cluster_cloud(ground_cloud)
        boxes = self.compute_bounding_boxes(clusters)

        # Publish bounding boxes
        self.publish_bounding_boxes(boxes)

    def filter_cloud(self, cloud):
        # Downsample the point cloud
        downsampled_cloud = cloud.voxel_down_sample(voxel_size=0.2)

        print(f"Number of points after downsampling: {len(downsampled_cloud.points)}")

        # Define the region of interest (ROI) to filter the point cloud
        points = np.asarray(downsampled_cloud.points)
        x_range = [-10, 10]
        y_range = [-5, 5]
        z_range = [-2, 3]
        
        roi_indices = np.where(
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )[0]
        cloud_roi = downsampled_cloud.select_by_index(roi_indices)

        print(f"Number of points after region of interest filtering: {len(cloud_roi.points)}")

        return cloud_roi

    def segment_ground(self, cloud):
        # Segment ground plane
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
        ground_cloud = cloud.select_by_index(inliers)

        print(f"Number of points in ground cloud: {len(ground_cloud.points)}")

        return ground_cloud

    def cluster_cloud(self, cloud):
        if len(cloud.points) == 0:
            rospy.logwarn("No points in ground cloud to cluster")
            return []

        # Perform DBSCAN clustering
        labels = np.array(cloud.cluster_dbscan(eps=0.5, min_points=10))

        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue  # Ignore noise
            cluster_indices = np.where(labels == label)[0]
            cluster = cloud.select_by_index(cluster_indices)
            clusters.append(cluster)

        print(f"Number of clusters found: {len(clusters)}")

        return clusters

    def compute_bounding_boxes(self, clusters):
        boxes = []
        for cluster in clusters:
            points = np.asarray(cluster.points)
            min_bound = points.min(axis=0)
            max_bound = points.max(axis=0)
            box = {
                'x_min': min_bound[0],
                'y_min': min_bound[1],
                'z_min': min_bound[2],
                'x_max': max_bound[0],
                'y_max': max_bound[1],
                'z_max': max_bound[2]
            }
            boxes.append(box)
        return boxes

    def publish_bounding_boxes(self, boxes):
        marker_array = MarkerArray()
        for i, box in enumerate(boxes):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bounding_box"
            marker.id = i
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
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    detector = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

