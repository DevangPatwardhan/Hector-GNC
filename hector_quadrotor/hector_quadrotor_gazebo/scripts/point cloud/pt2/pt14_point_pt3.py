#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf
import math


class ObstacleDetection:
    def __init__(self):
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.listener = tf.TransformListener()

        rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/ground_truth/state", Odometry, self.get_robot_pose)

        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = Quaternion()
        self.latest_outside_point = None  # To store the latest outside point as Point object

    def get_robot_pose(self, data):
        self.robot_pose[0] = data.pose.pose.position.x
        self.robot_pose[1] = data.pose.pose.position.y
        self.robot_pose[2] = data.pose.pose.position.z
        self.robot_orientation = data.pose.pose.orientation

    def pointcloud_callback(self, msg):
        try:
            (trans, rot) = self.listener.lookupTransform('/base_footprint', '/velodyne', rospy.Time(0))
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

        # Cluster the point cloud
        labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10))

        unique_labels = set(labels)
        unique_labels.discard(-1)
        num_obstacles = len(unique_labels)
        rospy.loginfo("Number of obstacles detected: %d" % num_obstacles)

        marker_array = MarkerArray()
        marker_id = 0

        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = pcd.select_by_index(cluster_indices)

            cluster_array = np.asarray(cluster_points.points)
            distances = np.linalg.norm(cluster_array - self.robot_pose, axis=1)
            closest_cluster_point = cluster_array[np.argmin(distances)]

            # Compute the bounding box
            bbox = cluster_points.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extents = bbox.get_extent()

            rospy.loginfo("Obstacle %d: Center (x, y, z) = %s, Horizontal size = %.3f, Vertical size = %.3f" % (
                label, center, extents[0], extents[2]))

            self.publish_box(marker_array, bbox, marker_id)
            self.publish_point(marker_array, center, marker_id)
            self.latest_outside_point = self.publish_center_outside(marker_array, bbox, marker_id)
            rospy.loginfo("Returned outside points (x, y, z) = [%.3f, %.3f, %.3f]" % (
                self.latest_outside_point.x, self.latest_outside_point.y, self.latest_outside_point.z))
            marker_id += 1

        self.marker_pub.publish(marker_array)

    def publish_box(self, marker_array, bbox, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
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
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Transparency
        marker_array.markers.append(marker)

    def publish_point(self, marker_array, position, marker_id):
        point_marker = Marker()
        point_marker.header.frame_id = "base_footprint"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "obstacle_centers"
        point_marker.id = marker_id + 1000  # Ensure unique IDs for points
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x = position[0]
        point_marker.pose.position.y = position[1]
        point_marker.pose.position.z = position[2]
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.1  # Size of the point marker
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1
        point_marker.color.r = 0.0
        point_marker.color.g = 0.0
        point_marker.color.b = 1.0  # Blue color
        point_marker.color.a = 1.0
        marker_array.markers.append(point_marker)

        rospy.loginfo("Center point (x, y, z) = [%.3f, %.3f, %.3f]" % (position[0], position[1], position[2]))

    def publish_center_outside(self, marker_array, bbox, marker_id, offset=0.5):
        center = bbox.get_center()
        extents = bbox.get_extent()

        # Calculate the point outside the cuboid along the horizontal and vertical directions
        outside_point_x = (center[0]/2)+offset
        outside_point_y = (center[1]/2) +offset
        outside_point_z = (center[2])

        # Transform the outside point from the robot frame to the global frame
        outside_point_robot_frame = np.array([outside_point_x, outside_point_y, outside_point_z, 1])
        transform_matrix = self.listener.fromTranslationRotation(
            [self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]], 
            [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        )
        outside_point_global_frame = np.dot(transform_matrix, outside_point_robot_frame)[:3]

        # Create a marker for the point in the global frame
        point_marker = Marker()
        point_marker.header.frame_id = "world"  # Gazebo global frame
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "obstacle_centers_outside"
        point_marker.id = marker_id + 2000  # Ensure unique IDs for points
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x =outside_point_x
        point_marker.pose.position.y =outside_point_y
        point_marker.pose.position.z = outside_point_z
        point_marker.pose.orientation.w = 1.0
        point_marker.scale.x = 0.1  # Size of the point marker
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1
        point_marker.color.r = 1.0  # Red color
        point_marker.color.g = 0.0
        point_marker.color.b = 0.0
        point_marker.color.a = 1.0
        marker_array.markers.append(point_marker)

        rospy.loginfo("Outside center point (x, y, z) = [%.3f, %.3f, %.3f]" % (outside_point_global_frame[0], outside_point_global_frame[1], outside_point_global_frame[2]))

        return Point(outside_point_global_frame[0], outside_point_global_frame[1], outside_point_global_frame[2])

    def get_latest_outside_point(self):
        return self.latest_outside_point


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(qx, qy, qz, qw)


if __name__ == "__main__":
    rospy.init_node("obstacle_detection")
    od = ObstacleDetection()
    rospy.spin()
