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
        # Convert PointCloud2 message to Open3D point cloud
        point_cloud = self.convert_pointcloud2_to_o3d(msg)
        
        # Preprocess point cloud
        point_cloud = self.preprocess_pointcloud(point_cloud)
        
        # Visualization or further processing can be done here
        
    def convert_pointcloud2_to_o3d(self, cloud_msg):
        points_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        
        # Create Open3D point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points_list))
        
        return point_cloud
    
    def preprocess_pointcloud(self, point_cloud):
        # Example preprocessing steps
        
        # 1. Downsampling
        voxel_size = 0.1  # Define your voxel size here
        point_cloud = point_cloud.voxel_down_sample(voxel_size)
        
        # 2. Removing noise (statistical outlier removal)
        point_cloud, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        # 3. Ground plane removal (simple example using height threshold)
        points = np.asarray(point_cloud.points)
        ground_threshold = 0.2  # Define your ground threshold here
        non_ground_points = points[points[:, 2] > ground_threshold]
        point_cloud.points = o3d.utility.Vector3dVector(non_ground_points)
        
        return point_cloud

if __name__ == "__main__":
    rospy.init_node('obstacle_detection_node')
    od = ObstacleDetection()
    rospy.spin()

