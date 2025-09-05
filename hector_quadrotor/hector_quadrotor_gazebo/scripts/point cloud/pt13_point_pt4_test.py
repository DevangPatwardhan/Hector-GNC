#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Quaternion
import math

def get_obstacle_info():
    
    listener = tf.TransformListener()
    robot_pose = np.array([0.0, 0.0, 0.0])
    obstacle_info = []

    marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)

    def get_robot_pose(data):
        nonlocal robot_pose
        robot_pose[0] = data.pose.pose.position.x
        robot_pose[1] = data.pose.pose.position.y
        robot_pose[2] = data.pose.pose.position.z

    def compute_outside_point(center, offset=0.5):
       # outside_point_x = (center[0] / 1.5)
       # outside_point_y = (center[1] / 1.5)
       # outside_point_z = (center[2] / 1.2)
        
        outside_point_x = (center[0] )
        outside_point_y = (center[1] )
        outside_point_z = (center[2] )
        return [outside_point_x, outside_point_y, outside_point_z]

    def publish_marker(marker_array, marker_type, position, scale, color, marker_id, ns=""):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker_array.markers.append(marker)

    def pointcloud_callback(msg):
        nonlocal listener, robot_pose, obstacle_info
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/velodyne', rospy.Time(0))
            transform_matrix = listener.fromTranslationRotation(trans, rot)
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

        marker_array = MarkerArray()
        obstacle_info = []

        for marker_id, label in enumerate(unique_labels):
            cluster_indices = np.where(labels == label)[0]
            cluster_points = pcd.select_by_index(cluster_indices)

            cluster_array = np.asarray(cluster_points.points)
            distances = np.linalg.norm(cluster_array - robot_pose, axis=1)
            closest_cluster_point = cluster_array[np.argmin(distances)]

            # Compute the bounding box
            bbox = cluster_points.get_axis_aligned_bounding_box()
            center = bbox.get_center()
           
            
            extents = bbox.get_extent()
            length = extents[0]
            width = extents[1]

            outside_point = compute_outside_point(center)
            modified_outside_point=[(center[0]/1.5),(center[1]/1.5),(center[2]/1.2)]
            obstacle_info.append((np.linalg.norm(center - robot_pose), modified_outside_point, length, width))
            modified_outside_point=[5,5,5]

            # Publish cuboid marker
            publish_marker(marker_array, Marker.CUBE, center, extents, (0.0, 1.0, 0.0, 0.5), marker_id, "obstacles")

            # Publish center point marker
            publish_marker(marker_array, Marker.SPHERE, center, (0.1, 0.1, 0.1), (0.0, 0.0, 1.0, 1.0), marker_id + 1000, "obstacle_centers")

            # Publish outside point marker
            publish_marker(marker_array, Marker.SPHERE, modified_outside_point, (0.1, 0.1, 0.1), (1.0, 0.0, 0.0, 1.0), marker_id + 2000, "obstacle_centers_outside")

        marker_pub.publish(marker_array)

    rospy.Subscriber("/velodyne_points", PointCloud2, pointcloud_callback)
    rospy.Subscriber("/ground_truth/state", Odometry, get_robot_pose)

    # Wait for messages to be processed
    rospy.sleep(2)

    # Return the list of obstacle info
    return obstacle_info

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(qx, qy, qz, qw)

def euler_from_vector(vector):
    x, y, z = vector.x, vector.y, -vector.z
    yaw = math.atan2(y, x)
    pitch = math.atan2(z, math.sqrt(x**2 + y**2))
    roll = 0
    return roll, pitch, yaw

def calculate_orientation_and_geometry(X, pix, lix):
    # Calculate direction vector
    direction_vector = Point(pix.x - X.x, pix.y - X.y, pix.z - X.z)
    
    # Calculate Euler angles and quaternion orientation
    roll, pitch, yaw = euler_from_vector(direction_vector)
    orientation = quaternion_from_euler(roll, pitch, yaw)
    
    # Calculate geometry
    rix = math.sqrt((pix.x - X.x)**2 + (pix.y - X.y)**2 + (pix.z - X.z)**2)
    half_angle = math.atan2(lix, rix)  # Compute vertex angle /2

    if half_angle > math.pi / 2:
        half_angle = math.pi - half_angle
    elif half_angle < -math.pi / 2:
        half_angle = -math.pi - half_angle
    else:
        half_angle = half_angle

    # This modification ensures that half_angle always falls within the range [−π/2,π/2], 
    # thus preventing math.cos(half_angle) from becoming negative. This keeps the computed height hc non-negative

    hc = rix * math.cos(half_angle)
    rcb = hc * math.tan(half_angle)
    # slant_height = math.sqrt(hc**2 + rcb**2)
    slant_height = rix
    hsc = slant_height - math.sqrt(slant_height**2 - rcb**2)
 
    rsc = math.sqrt(slant_height**2 - (slant_height - hsc)**2)
    return orientation, slant_height, hsc, rsc, rix, half_angle, rcb, hc

def create_cone_marker(frame_id, marker_id, X, orientation, hc, rcb):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cone"
    marker.id = marker_id
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    n_segments = 36
    delta_theta = 2 * math.pi / n_segments

    for i in range(n_segments):
        theta = i * delta_theta
        p1 = Point(0, 0, 0)
        p2 = Point(hc, rcb * math.cos(theta), rcb * math.sin(theta))
        p3 = Point(hc, rcb * math.cos(theta + delta_theta), rcb * math.sin(theta + delta_theta))
        marker.points.extend([p3, p2, p1])

    marker.pose.position = X
    marker.pose.orientation = orientation
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0, 1.0, 0, 0.5)
    return marker

def create_spherical_cap_marker(frame_id, marker_id, X, orientation, slant_height, hsc, rsc, rix):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "spherical_cap"
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    n_segments = 90
    delta_theta = 2 * math.pi / n_segments
    delta_phi = math.pi / 360

    for i in range(n_segments):
        theta1 = i * delta_theta
        theta2 = (i + 1) % n_segments * delta_theta
        for j in range(int(math.pi / delta_phi)):
            phi1 = j * delta_phi
            phi2 = (j + 1) * delta_phi

            if phi1 > math.asin(rsc / slant_height):
                break

            p1 = Point(slant_height * math.cos(phi1), slant_height * math.sin(phi1) * math.cos(theta1), slant_height * math.sin(phi1) * math.sin(theta1))
            p2 = Point(slant_height * math.cos(phi1), slant_height * math.sin(phi1) * math.cos(theta2), slant_height * math.sin(phi1) * math.sin(theta2))
            p3 = Point(slant_height * math.cos(phi2), slant_height * math.sin(phi2) * math.cos(theta1), slant_height * math.sin(phi2) * math.sin(theta1))
            p4 = Point(slant_height * math.cos(phi2), slant_height * math.sin(phi2) * math.sin(theta2), slant_height * math.sin(phi2) * math.cos(theta2))

            marker.points.extend([p1, p2, p3, p1, p3, p4])

    marker.pose.position = X
    marker.pose.orientation = orientation
    marker.scale.x = 0.01
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0, 0, 0.25)
    return marker

if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node", anonymous=True)
    obstacle_info = get_obstacle_info()
    for info in obstacle_info:
        distance, outside_point, length, width = info
        print("Obstacle at distance %.3f: Outside point (x, y, z) = [%.3f, %.3f, %.3f], Length = %.3f, Width = %.3f" %
              (distance, outside_point[0], outside_point[1], outside_point[2], length, width))
    
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    frame_id = "base_footprint"
    X = Point(0, 0, 3)
    pix = Point(1.704, 0.858, 3)
    lix = 1.0  
    orientation, slant_height, hsc, rsc, rix, half_angle, rcb, hc = calculate_orientation_and_geometry(X, pix, lix)

    print("Orientation:", orientation)
    print("Slant Height:", slant_height)
    print("Height of Cap:", hsc)
    print("Radius of Cap:", rsc)
    print("Height of cone:", hc)
    print("Radius of cone:", rcb)
    print("lix:", lix)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cone_marker = create_cone_marker(frame_id, 0, X, orientation, hc, rcb)
        cap_marker = create_spherical_cap_marker(frame_id, 1, X, orientation, slant_height, hsc, rsc, rix)
        pub.publish(cone_marker)
        pub.publish(cap_marker)
        rate.sleep()   
    
    rospy.spin()
