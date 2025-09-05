#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker

# Sensor location as per the VLP-16 setup
SENSOR_LOCATION = np.array([0.0, 0.0, 0.35])

class ObstacleDetection:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))

        if len(points) == 0:
            rospy.logwarn("No points received")
            return

        # Create an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=0.1)

        # Segment the plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
        remaining_cloud = pcd.select_by_index(inliers, invert=True)
        labels = np.array(remaining_cloud.cluster_dbscan(eps=0.2, min_points=10))

        unique_labels = set(labels)
        unique_labels.discard(-1)
        num_obstacles = len(unique_labels)
        rospy.loginfo("Number of obstacles detected: %d" % num_obstacles)

        obstacles = []
        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = remaining_cloud.select_by_index(cluster_indices)

            cluster_array = np.asarray(cluster_points.points)
            distances = np.linalg.norm(cluster_array - SENSOR_LOCATION, axis=1)
            min_cluster_distance = np.min(distances)
            closest_cluster_point = cluster_array[np.argmin(distances)]

            bbox = cluster_points.get_axis_aligned_bounding_box()
            extents = bbox.get_extent()
            horizontal_space = (extents[0], extents[1])  # width and length
            obstacles.append((min_cluster_distance, closest_cluster_point, horizontal_space))

        obstacles.sort(key=lambda x: x[0])
        for idx, (dist, point, horizontal_space) in enumerate(obstacles):
            x_dist, y_dist, z_dist = point - SENSOR_LOCATION
            rospy.loginfo(f"Obstacle {idx + 1}: Closest point {point} with distances x: {x_dist}, y: {y_dist}, z: {z_dist}")
            rospy.loginfo(f"Obstacle {idx + 1}: Width: {horizontal_space[0]}, Length: {horizontal_space[1]}")

            self.publish_marker(point, idx + 1, horizontal_space)

    def publish_marker(self, point3, idx, horizontal_space):
        # Publish the closest point as a marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point3[0]
        marker.pose.position.y = point3[1]
        marker.pose.position.z = point3[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        # Publishing the dimensions as a text marker
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "obstacle_dimensions"
        text_marker.id = idx + 100  # Unique ID for text marker
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = point3[0]
        text_marker.pose.position.y = point3[1]
        text_marker.pose.position.z = point3[2] + 0.5  # Offset the text above the point
        text_marker.scale.z = 0.5  # Text size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        text_marker.text = f"Width: {horizontal_space[0]:.2f}, Length: {horizontal_space[1]:.2f}"
        self.marker_pub.publish(text_marker)

if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node", anonymous=True)
    detector = ObstacleDetection()
    rospy.spin()

