#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf
from sklearn.neighbors import NearestNeighbors

# Global variables to store the robot pose and point cloud message
robot_pose = np.array([0.0, 0.0, 0.0])
robot_pose_received = False
pointcloud_msg = None
listener = None

class TrackedObstacle:
    def __init__(self, id, position, bbox):
        self.id = id
        self.position = position
        self.bbox = bbox
        self.lost_frames = 0

def get_robot_pose(data):
    global robot_pose, robot_pose_received
    robot_pose[0] = data.pose.pose.position.x
    robot_pose[1] = data.pose.pose.position.y
    robot_pose[2] = data.pose.pose.position.z
    robot_pose_received = True

def pointcloud_callback(msg):
    global pointcloud_msg
    pointcloud_msg = msg

def get_obstacles_info():
    global pointcloud_msg

    if pointcloud_msg is None:
        return []

    # Convert PointCloud2 message to Open3D point cloud
    cloud_data = list(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))
    if len(cloud_data) == 0:
        return []

    cloud_np = np.array(cloud_data, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np)

    # Downsample the point cloud
    downpcd = pcd.voxel_down_sample(voxel_size=0.1)

    # Segment the plane
    plane_model, inliers = downpcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
    outliers = downpcd.select_by_index(inliers, invert=True)

    # Cluster the points
    labels = np.array(outliers.cluster_dbscan(eps=0.5, min_points=10))
    max_label = labels.max()

    obstacles = []
    for i in range(max_label + 1):
        indices = np.where(labels == i)[0]
        if len(indices) == 0:
            continue
        cluster = outliers.select_by_index(indices)

        # Compute PCA bounding box
        points = np.asarray(cluster.points)
        mean = points.mean(axis=0)
        cov = np.cov(points - mean, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = eigvals.argsort()[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]
        rotated_points = np.dot(points - mean, eigvecs)

        min_corner = rotated_points.min(axis=0)
        max_corner = rotated_points.max(axis=0)
        bbox_corners = np.array([[min_corner[0], min_corner[1], min_corner[2]],
                                 [max_corner[0], min_corner[1], min_corner[2]],
                                 [max_corner[0], max_corner[1], min_corner[2]],
                                 [min_corner[0], max_corner[1], min_corner[2]],
                                 [min_corner[0], min_corner[1], max_corner[2]],
                                 [max_corner[0], min_corner[1], max_corner[2]],
                                 [max_corner[0], max_corner[1], max_corner[2]],
                                 [min_corner[0], max_corner[1], max_corner[2]]])
        bbox_corners = np.dot(bbox_corners, eigvecs.T) + mean

        obstacles.append((mean, bbox_corners))

    return obstacles

def publish_markers(tracked_obstacles):
    global marker_pub

    marker_array = MarkerArray()
    for obs in tracked_obstacles:
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = obs.id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        bbox = obs.bbox
        bbox_lines = [
            [bbox[0], bbox[1]], [bbox[1], bbox[2]], [bbox[2], bbox[3]], [bbox[3], bbox[0]],  # Bottom face
            [bbox[4], bbox[5]], [bbox[5], bbox[6]], [bbox[6], bbox[7]], [bbox[7], bbox[4]],  # Top face
            [bbox[0], bbox[4]], [bbox[1], bbox[5]], [bbox[2], bbox[6]], [bbox[3], bbox[7]]   # Vertical lines
        ]

        for line in bbox_lines:
            p1, p2 = line
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            marker.points.append(Point(p2[0], p2[1], p2[2]))

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def track_obstacles(detected_obstacles, tracked_obstacles, max_distance=1.0):
    if not tracked_obstacles:
        for i, (mean, bbox) in enumerate(detected_obstacles):
            tracked_obstacles.append(TrackedObstacle(i, mean, bbox))
        return tracked_obstacles

    detected_positions = np.array([obs[0] for obs in detected_obstacles])
    tracked_positions = np.array([obs.position for obs in tracked_obstacles])

    if len(tracked_positions) == 0 or len(detected_positions) == 0:
        return tracked_obstacles

    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(detected_positions)
    distances, indices = nbrs.kneighbors(tracked_positions)

    assigned = set()
    for i, (dist, index) in enumerate(zip(distances.flatten(), indices.flatten())):
        if dist < max_distance and index not in assigned:
            tracked_obstacles[i].position = detected_obstacles[index][0]
            tracked_obstacles[i].bbox = detected_obstacles[index][1]
            tracked_obstacles[i].lost_frames = 0
            assigned.add(index)
        else:
            tracked_obstacles[i].lost_frames += 1

    # Add new detected obstacles as new tracked obstacles
    unassigned = set(range(len(detected_obstacles))) - assigned
    for index in unassigned:
        new_id = max([obs.id for obs in tracked_obstacles], default=-1) + 1
        tracked_obstacles.append(TrackedObstacle(new_id, detected_obstacles[index][0], detected_obstacles[index][1]))

    # Remove lost obstacles
    tracked_obstacles = [obs for obs in tracked_obstacles if obs.lost_frames < 5]

    return tracked_obstacles

def main():
    global marker_pub, listener
    rospy.init_node("obstacle_detection_node")
    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    rospy.Subscriber("/velodyne_points", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/ground_truth/state", Odometry, get_robot_pose)

    listener = tf.TransformListener()

    rospy.sleep(1)  # Give some time to receive the messages

    tracked_obstacles = []

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        detected_obstacles = get_obstacles_info()
        tracked_obstacles = track_obstacles(detected_obstacles, tracked_obstacles)
        publish_markers(tracked_obstacles)
        rate.sleep()

if __name__ == "__main__":
    main()

