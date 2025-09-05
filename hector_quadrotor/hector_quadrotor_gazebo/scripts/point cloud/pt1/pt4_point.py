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

        # Create an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
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
            distances = np.linalg.norm(cluster_array - self.robot_pose, axis=1)
            min_cluster_distance = np.min(distances)
            closest_cluster_point = cluster_array[np.argmin(distances)]

            bbox = cluster_points.get_axis_aligned_bounding_box()
            extents = bbox.get_extent()
            width, length = extents[0], extents[1]
            obstacles.append((min_cluster_distance, closest_cluster_point, width, length))

        obstacles.sort(key=lambda x: x[0])
        for idx, (dist, point, width, length) in enumerate(obstacles):
            rospy.loginfo(f"Obstacle {idx + 1}: Closest point {point} with distance: {dist}")
            rospy.loginfo(f"Obstacle {idx + 1}: Width: {width}, Length: {length}")

            self.publish_marker(point, idx + 1, width, length)

    def publish_marker(self, point, idx, width, length):
        # Publish the closest point as a marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
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
        text_marker.pose.position.x = point[0]
        text_marker.pose.position.y = point[1]
        text_marker.pose.position.z = point[2] + 0.5  # Offset the text above the point
        text_marker.scale.z = 0.5  # Text size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        text_marker.text = f"Width: {width:.2f}, Length: {length:.2f}"
        self.marker_pub.publish(text_marker)

if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node", anonymous=True)
    detector = ObstacleDetection()
    rospy.spin()

