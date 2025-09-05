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
next_track_id = 0

class Track:
    def __init__(self, id, position, bbox):
        self.id = id
        self.position = np.array(position)
        self.bbox = bbox
        self.lifetime = 0
        self.state = 'mature'
        self.has_converged = True
        self.is_dynamic = True
        self.measurement_cps = []
        self.assoc_bb = bbox
        self.ambiguous = False

        # Kalman Filter Initialization
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.x = np.array([position[0], 0, position[1], 0, position[2], 0])  # initial state (location and velocity)
        self.kf.F = np.array([[1, 1, 0, 0, 0, 0],  # state transition matrix
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 1, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 1],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],  # Measurement function
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 1, 0]])
        self.kf.P *= 1000.  # covariance matrix
        self.kf.R *= 5  # measurement noise
        self.kf.Q *= 0.01  # process noise

    def predict(self):
        self.kf.predict()

    def update(self, measurement):
        self.kf.update(measurement)
        self.position = self.kf.x[::2]

def get_robot_pose(data):
    global robot_pose, robot_pose_received
    robot_pose[0] = data.pose.pose.position.x
    robot_pose[1] = data.pose.pose.position.y
    robot_pose[2] = data.pose.pose.position.z
    robot_pose_received = True

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

        publish_box(marker_array, bbox, label)
        publish_point(marker_array, center, marker_id)
        outside_point = get_center_outside(bbox, center)
        publish_outside_point(marker_array, outside_point, marker_id)
        obstacle_info.append({
            "distance": np.min(distances),
            "closest_point": closest_cluster_point.tolist(),
            "center": center.tolist(),
            "bbox": bbox,
            "horizontal_size": extents[0],
            "vertical_size": extents[2]
        })
        marker_id += 1

    marker_array_pub.publish(marker_array)
    
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

def publish_box(marker_array, bbox, label):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "obstacles"
    marker.id = label  # Use label as the marker ID
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

    # Publish text marker for obstacle ID
    text_marker = Marker()
    text_marker.header.frame_id = "world"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "obstacle_texts"
    text_marker.id = label + 1000  # Ensure unique ID for text markers
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = center[0]
    text_marker.pose.position.y = center[1]
    text_marker.pose.position.z = center[2] + extents[2] / 2.0  # Position above the obstacle
    text_marker.scale.z = 0.3  # Text size
    text_marker.color.r = 1.0  # White color
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0  # Full opacity
    text_marker.text = f"Obstacle {label}"  # Format text as "Obstacle {label}"

    marker_array.markers.append(text_marker)

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
    point_marker.scale.x = 0.2
    point_marker.scale.y = 0.2
    point_marker.scale.z = 0.2
    point_marker.color.r = 1.0
    point_marker.color.g = 0.0
    point_marker.color.b = 0.0
    point_marker.color.a = 1.0
    marker_array.markers.append(point_marker)

def publish_outside_point(marker_array, position, marker_id):
    outside_point_marker = Marker()
    outside_point_marker.header.frame_id = "world"
    outside_point_marker.header.stamp = rospy.Time.now()
    outside_point_marker.ns = "obstacle_outside_points"
    outside_point_marker.id = marker_id + 2000  # Ensure unique IDs for outside points
    outside_point_marker.type = Marker.SPHERE
    outside_point_marker.action = Marker.ADD
    outside_point_marker.pose.position.x = position[0]
    outside_point_marker.pose.position.y = position[1]
    outside_point_marker.pose.position.z = position[2]
    outside_point_marker.scale.x = 0.2
    outside_point_marker.scale.y = 0.2
    outside_point_marker.scale.z = 0.2
    outside_point_marker.color.r = 0.0
    outside_point_marker.color.g = 0.0
    outside_point_marker.color.b = 1.0
    outside_point_marker.color.a = 1.0
    marker_array.markers.append(outside_point_marker)

def track_obstacles(obstacle_info):
    global track_list, next_track_id

    updated_track_list = []

    for obstacle in obstacle_info.values():
        min_dist = float('inf')
        closest_track = None

        for track in track_list:
            dist = np.linalg.norm(track.position - np.array(obstacle["center"]))
            if dist < min_dist:
                min_dist = dist
                closest_track = track

        if min_dist < 1.0:  # Threshold for associating a detection with a track
            closest_track.update(np.array(obstacle["center"]))
            closest_track.assoc_bb = obstacle["bbox"]
            updated_track_list.append(closest_track)
        else:
            new_track = Track(next_track_id, obstacle["center"], obstacle["bbox"])
            next_track_id += 1
            updated_track_list.append(new_track)

    track_list = updated_track_list

    # Publish track markers
    marker_array = MarkerArray()
    for track in track_list:
        bbox = track.assoc_bb
        if bbox is not None:
            #publish_box(marker_array, bbox, track.id + 3000) 
            publish_box(marker_array, bbox, track.id ) # Add the 'label' argument here
    marker_array_pub.publish(marker_array)

def main():
    global marker_array_pub, listener

    rospy.init_node('obstacle_detection_node', anonymous=True)
    pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, pointcloud_callback)
    odometry_sub = rospy.Subscriber('/ground_truth/state', Odometry, get_robot_pose)
    marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        if robot_pose_received and pointcloud_msg:
            process_pointcloud(pointcloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

