#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import tf

class ObstacleDetection:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
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
        pcd = pcd.voxel_down_sample(voxel_size=0.05)

        # Segment the plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
        remaining_cloud = pcd.select_by_index(inliers, invert=True)

        # Cluster points based on distance
        labels = np.array(remaining_cloud.cluster_dbscan(eps=0.20, min_points=10))

        unique_labels = set(labels)
        unique_labels.discard(-1)
        num_obstacles = len(unique_labels)
        rospy.loginfo("Number of obstacles detected: %d" % num_obstacles)

        marker_array = MarkerArray()
        marker_id = 0

        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = remaining_cloud.select_by_index(cluster_indices)

            cluster_array = np.asarray(cluster_points.points)

            # Compute the bounding box
            bbox = cluster_points.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extents = bbox.get_extent()

            rospy.loginfo(f"Obstacle {marker_id}: Center (x, y, z) = {center}, Horizontal size = {extents[0]}, Vertical size = {extents[2]}")

            # Publish box
            self.publish_box(marker_array, bbox, marker_id)

            # Publish point at center
            self.publish_center_point(marker_array, center, marker_id + 1000)

            marker_id += 1

        self.marker_pub.publish(marker_array)

    def publish_box(self, marker_array, bbox, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        center = bbox.get_center()
        extents = bbox.get_extent()
        
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = extents[0]
        marker.scale.y = extents[1]
        marker.scale.z = extents[2]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

    def publish_center_point(self, marker_array, center, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "center_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node", anonymous=True)
    detector = ObstacleDetection()
    rospy.spin()

