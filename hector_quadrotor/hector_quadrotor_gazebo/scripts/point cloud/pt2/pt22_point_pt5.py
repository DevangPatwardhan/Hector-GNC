#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import tf
from filterpy.kalman import KalmanFilter

# Global variables to store the robot pose and point cloud message
robot_pose = np.array([0.0, 0.0, 0.0])
robot_pose_received = False
pointcloud_msg = None
listener = None
track_list = []

class Track:
    def __init__(self, id, position, bbox):
        self.id = id
        self.position = position
        self.bbox = bbox
        self.lifetime = 0
        self.state = 'mature'
        self.has_converged = True
        self.rel_vel = np.array([0.0, 0.0, 0.0])
        self.probabIMM = {'M1': 0.0, 'M2': 0.0, 'M3': 0.0}
        self.is_dynamic = True
        self.measurement_cps = []
        self.assoc_bb = bbox
        self.ambiguous = False
        # Initialize Kalman Filter for tracking
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.x = np.zeros(6)  # Initial state (position and velocity)
        self.kf.P *= 1000.  # Initial uncertainty
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])  # State transition matrix
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])  # Measurement matrix
        self.kf.R = np.eye(3) * 10  # Measurement noise
        self.kf.Q = np.eye(6)  # Process noise

def get_robot_pose(data):
    global robot_pose, robot_pose_received
    robot_pose[0] = data.pose.pose.position.x
    robot_pose[1] = data.pose.pose.position.y
    robot_pose[2] = data.pose.pose.position.z
    robot_pose_received = True
    return robot_pose[0], robot_pose[1], robot_pose[2]

def pointcloud_callback(msg):
    global pointcloud_msg
    pointcloud_msg = msg

def process_pointcloud(msg):
    global listener

    try:
        listener.waitForTransform('/world', msg.header.frame_id, rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('/world', msg.header.frame_id, rospy.Time(0))
        transform_matrix = listener.fromTranslationRotation(trans, rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("TF lookup failed")
        return {}

    points = np.array(list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))

    if len(points) == 0:
        rospy.logwarn("No points received")
        return {}

    # Transform points to the world frame
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
    obstacle_info = []

    for label in unique_labels:
        cluster_indices = np.where(labels == label)[0]
        cluster_points = pcd.select_by_index(cluster_indices)

        cluster_array = np.asarray(cluster_points.points)
        distances = np.linalg.norm(cluster_array - robot_pose, axis=1)
        closest_cluster_point = cluster_array[np.argmin(distances)]

        # Compute the bounding box
        bbox = cluster_points.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extents = bbox.get_extent()

        rospy.loginfo("Obstacle %d: Center (x, y, z) = %s, Horizontal size = %.3f, Vertical size = %.3f" % (
            label, center, extents[0], extents[2]))

        publish_box(marker_array, bbox, marker_id)
        publish_point(marker_array, center, marker_id)
        outside_point = get_center_outside(bbox, center)
        publish_outside_point(marker_array, outside_point, marker_id)
        obstacle_info.append({
            "distance": np.min(distances),
            "outside_point": outside_point.tolist(),
            "center": center.tolist(),
            "horizontal_size": extents[0],
            "vertical_size": extents[2]
        })
        marker_id += 1

    marker_pub.publish(marker_array)
    
    # Sort obstacles by distance
    obstacle_info.sort(key=lambda x: x["distance"])

    # Create the final dictionary
    sorted_obstacle_info = {f"obstacle{idx}": info for idx, info in enumerate(obstacle_info)}

    # Track the obstacles
    track_obstacles(sorted_obstacle_info)

    return sorted_obstacle_info

def get_center_outside(bbox, center, offset=0.5):
    # Calculate the direction vector from the robot to the center of the obstacle
    direction_vector = center - robot_pose
    direction_vector /= np.linalg.norm(direction_vector)

    # Adjust direction vector to ensure the outside point is always in front of the drone
    direction_vector[2] = 0  # Ensure the outside point remains at the same height as the obstacle center
    if np.linalg.norm(direction_vector) > 0:
        direction_vector /= np.linalg.norm(direction_vector)

    # Calculate the outside point
    outside_point = center - direction_vector * offset

    rospy.loginfo("Outside center point (x, y, z) = [%.3f, %.3f, %.3f]" % (outside_point[0], outside_point[1], outside_point[2]))
    return outside_point

def publish_box(marker_array, bbox, marker_id):
    marker = Marker()
    marker.header.frame_id = "world"
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

def publish_point(marker_array, position, marker_id):
    point_marker = Marker()
    point_marker.header.frame_id = "world"
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

def publish_outside_point(marker_array, position, marker_id):
    outside_marker = Marker()
    outside_marker.header.frame_id = "world"
    outside_marker.header.stamp = rospy.Time.now()
    outside_marker.ns = "obstacle_outside_points"
    outside_marker.id = marker_id + 2000  # Ensure unique IDs for outside points
    outside_marker.type = Marker.SPHERE
    outside_marker.action = Marker.ADD
    outside_marker.pose.position.x = position[0]
    outside_marker.pose.position.y = position[1]
    outside_marker.pose.position.z = position[2]
    outside_marker.pose.orientation.w = 1.0
    outside_marker.scale.x = 0.1  # Size of the outside point marker
    outside_marker.scale.y = 0.1
    outside_marker.scale.z = 0.1
    outside_marker.color.r = 1.0  # Red color
    outside_marker.color.g = 0.0
    outside_marker.color.b = 0.0
    outside_marker.color.a = 1.0
    marker_array.markers.append(outside_marker)

    rospy.loginfo("Outside point (x, y, z) = [%.3f, %.3f, %.3f]" % (position[0], position[1], position[2]))

def track_obstacles(obstacles):
    global track_list
    # Convert obstacle information into Track objects and update
    new_tracks = []

    for idx, (obstacle_id, obstacle) in enumerate(obstacles.items()):
        position = np.array(obstacle["center"])
        bbox = None  # Handle the bounding box if needed
        found_track = False

        for track in track_list:
            if track.id == idx:
                found_track = True
                # Update Kalman Filter with the new position
                track.kf.predict()
                track.kf.update(position)
                track.position = track.kf.x[:3]
                track.lifetime += 1
                break

        if not found_track:
            new_track = Track(idx, position, bbox)
            new_track.kf.update(position)
            new_track.position = new_track.kf.x[:3]
            new_tracks.append(new_track)

    track_list = new_tracks

def main():
    global listener, marker_pub

    rospy.init_node('obstacle_tracking_node')
    rospy.Subscriber('/ground_truth/state', Odometry, get_robot_pose)
    rospy.Subscriber('/velodyne_points', PointCloud2, pointcloud_callback)
    marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if pointcloud_msg is not None and robot_pose_received:
            process_pointcloud(pointcloud_msg)
        rate.sleep()

if __name__ == '__main__':
    main()

