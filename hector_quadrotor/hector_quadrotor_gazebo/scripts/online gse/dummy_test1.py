#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point,Quaternion, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import heapq
import time
import random
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Quaternion,Twist
from math import pow, atan2, sqrt, pi, degrees


# Global variables to store the robot pose and point cloud message
robot_pose = np.array([0.0, 0.0, 0.0])
robot_pose_received = False
pointcloud_msg = None
listener = None

def get_robot_pose(data):
    global robot_pose, robot_pose_received
    robot_pose[0] = data.pose.pose.position.x
    robot_pose[1] = data.pose.pose.position.y
    robot_pose[2] = data.pose.pose.position.z
    robot_pose_received = True
    return robot_pose[0],robot_pose[1],robot_pose[2]

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
    outside_marker.color.r = 1.0
    outside_marker.color.g = 0.0
    outside_marker.color.b = 0.0  # Red color
    outside_marker.color.a = 1.0
    marker_array.markers.append(outside_marker)

    rospy.loginfo("Outside point (x, y, z) = [%.3f, %.3f, %.3f]" % (position[0], position[1], position[2]))

def get_obstacles_info():
    global marker_pub, pointcloud_msg, robot_pose_received

    if pointcloud_msg and robot_pose_received:
        return process_pointcloud(pointcloud_msg)
    else:
        rospy.logwarn("Pointcloud or robot pose not received yet")
        return {}
    
    
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



class MarkerPublisher:
    def __init__(self):
         # Added anonymous=True to ensure unique node names
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub= rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub1 = rospy.Publisher("visualization_marker2", Marker, queue_size=10)
        self.pub_workspace = rospy.Publisher('visualization_marker3', Marker, queue_size=10)
        self.pub_sphere1 = rospy.Publisher('shape1', Marker, queue_size=10)
        self.pub_sphere2 = rospy.Publisher('shape2', Marker, queue_size=10)
        self.pub_sphere3 = rospy.Publisher('shape3', Marker, queue_size=10)
        
        self.rate = rospy.Rate(10)  # Added a rate to control the publishing frequency
        
    def create_workspace_marker(self,min_x, max_x, min_y, max_y, min_z, max_z):
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line thickness
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)
    
        #  Define the vertices of the wall (12 line segments)
        vertices = [
            
           [min_x, min_y, min_z], [max_x, min_y, min_z],
           [max_x, min_y, min_z], [max_x, max_y, min_z],
           [max_x, max_y, min_z], [min_x, max_y, min_z],
           [min_x, max_y, min_z], [min_x, min_y, min_z],
        
           [min_x, min_y, max_z], [max_x, min_y, max_z],
           [max_x, min_y, max_z], [max_x, max_y, max_z], 
           [max_x, max_y, max_z], [min_x, max_y, max_z],
           [min_x, max_y, max_z], [min_x, min_y, max_z],
        
           [min_x, min_y, min_z], [min_x, min_y, max_z],
           [max_x, min_y, min_z], [max_x, min_y, max_z],
           [max_x, max_y, min_z], [max_x, max_y, max_z],
           [min_x, max_y, min_z], [min_x, max_y, max_z]
            ]
    
        # Set the vertices for the workspace Marker message
        for vertex in vertices:
            
            point = Point()
            point.x, point.y, point.z = vertex[0], vertex[1], vertex[2]
            marker.points.append(point)
    
            return marker
        
        
    def create_sphere_marker(self,center_x, center_y, center_z, radius, color):
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z
        marker.color = color
        return marker
    
    
    def generate_scene(self,min_x, max_x, min_y, max_y, min_z, max_z,
                   center_x1, center_y1, center_z1, radius1,
                   center_x2, center_y2, center_z2, radius2,
                   center_x3, center_y3, center_z3, radius3):
        
        
        
       
    
        #workspace_marker = self.create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z)
        #sphere_marker1 = self.create_sphere_marker(center_x1, center_y1, center_z1, radius1, ColorRGBA(1.0, 0.0, 0.0, 1.0))
        #sphere_marker2 = self.create_sphere_marker(center_x2, center_y2, center_z2, radius2, ColorRGBA(0.0, 1.0, 0.0, 1.0))
        #sphere_marker3 = self.create_sphere_marker(center_x3, center_y3, center_z3, radius3, ColorRGBA(0.0, 0.0, 1.0, 1.0))

        #self.pub_workspace.publish(workspace_marker)
        #self.pub_sphere1.publish(sphere_marker1)
        #self.pub_sphere2.publish(sphere_marker2)
        #self.pub_sphere3.publish(sphere_marker3)
        
        #self.rate.sleep()

        while not rospy.is_shutdown():
            
            workspace_marker = self.create_workspace_marker(min_x, max_x, min_y, max_y, min_z, max_z)
            sphere_marker1 = self.create_sphere_marker(center_x1, center_y1, center_z1, radius1, ColorRGBA(1.0, 0.0, 0.0, 1.0))
            sphere_marker2 = self.create_sphere_marker(center_x2, center_y2, center_z2, radius2, ColorRGBA(0.0, 1.0, 0.0, 1.0))
            sphere_marker3 = self.create_sphere_marker(center_x3, center_y3, center_z3, radius3, ColorRGBA(0.0, 0.0, 1.0, 1.0))

            self.pub_workspace.publish(workspace_marker)
            self.pub_sphere1.publish(sphere_marker1)
            self.pub_sphere2.publish(sphere_marker2)
            self.pub_sphere3.publish(sphere_marker3)
            self.rate.sleep()
        
        
    def publish_marker(self,point, point2 ,height, radius):
        
        # Calculate the vector components
        vector_x = point2.x - point.x
        vector_y = point2.y - point.y
        vector_z = point2.z - point.z

        # Calculate the magnitude of the vector (Euclidean distance)
        magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        #print("magnitude:",magnitude)

        # Calculate angles with x, y, z axes
        theta_x = math.atan2(vector_y, vector_z)
        theta_y = math.atan2(-vector_x, math.sqrt(vector_y**2 + vector_z**2))
        theta_z = math.atan2(math.sin(theta_x) * vector_x + math.cos(theta_x) * vector_y, vector_z)

        # Calculate quaternion from Euler angles
        cy = math.cos(theta_z * 0.5)
        sy = math.sin(theta_z * 0.5)
        cp = math.cos(theta_y * 0.5)
        sp = math.sin(theta_y * 0.5)
        cr = math.cos(theta_x * 0.5)
        sr = math.sin(theta_x * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        # Create a marker object
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "shapes"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "file:///home/ros/stl/r_0.5_h-1.stl"
        marker.action = Marker.ADD
        marker.pose.position = point  # Starting point
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = -w
        marker.scale.x = (radius * 2)  # Diameter equals twice the radius
        marker.scale.y = (radius * 2) 
        marker.scale.z = (magnitude)#height  # Set the scale according to the user-provided height
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        theta = math.atan(magnitude / radius)
        # Convert radians to degrees
        theta_degrees = math.degrees(theta)
        #print("Angle of the cone1: {:.2f} degrees".format(theta_degrees))
        

        return marker

    def publish_two_points(self, point1, point2):
        # Create a marker for the points
        points_marker = Marker()
        points_marker.header.frame_id = "base_link"  # Ensure this matches the fixed frame in RViz
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = "two_points"
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.scale.x = 0.1  # Point size
        points_marker.scale.y = 0.1
        points_marker.color.g = 1.0  # Green color
        points_marker.color.a = 1.0  # Alpha

        # Add the two points to the marker
        points_marker.points.append(point1)
        points_marker.points.append(point2)

        # Publish the points marker
        self.marker_pub.publish(points_marker)
        self.rate.sleep()  # Sleep to maintain the publishing rate

    def publish_vector_between_points(self, point1, point2):
        # Create a marker for the vector arrow
        vector_marker = Marker()
        vector_marker.header.frame_id = "base_link"  # Ensure this matches the fixed frame in RViz
        vector_marker.header.stamp = rospy.Time.now()
        vector_marker.ns = "vector_between_points"
        vector_marker.action = Marker.ADD
        vector_marker.pose.orientation.w = 1.0
        vector_marker.id = 1  # Changed ID to be unique for each marker
        vector_marker.type = Marker.ARROW
        vector_marker.scale.x = 0.05  # Arrow shaft diameter
        vector_marker.scale.y = 0.1  # Arrow head diameter
        vector_marker.scale.z = 0.2  # Arrow head length
        vector_marker.color.r = 1.0  # Red color
        vector_marker.color.a = 1.0  # Alpha

        # Set the start and end points of the vector
        vector_marker.points.append(point1)
        vector_marker.points.append(point2)

        # Publish the vector marker
        self.marker_pub.publish(vector_marker)
        self.rate.sleep()  # Sleep to maintain the publishing rate
        
    def publish_single_point(self,point3):
        
    
        marker_pub = rospy.Publisher("visualization_marker2", Marker, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        # Create a marker for the single point
        point_marker = Marker()
        point_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "single_point"
        point_marker.action = Marker.ADD
        point_marker.pose.orientation.w = 1.0
        point_marker.id = 0
        point_marker.type = Marker.POINTS
        point_marker.scale.x = 0.1  # Point size
        point_marker.scale.y = 0.1
        point_marker.color.g = 1.0  # Green color
        point_marker.color.a = 1.0  # Alpha

        # Add the point to the marker
        point_marker.points.append(point3)
 
        
        self.marker_pub.publish(point_marker)
        self.rate.sleep() 
        
        
    def publish_axes(self):
        
        # Create a marker for the axes
        axes_marker = Marker()
        axes_marker.header.frame_id = "base_link"
        axes_marker.header.stamp = rospy.Time.now()
        axes_marker.ns = "axes"
        axes_marker.action = Marker.ADD
        axes_marker.pose.orientation.w = 1.0
        axes_marker.id = 0
        axes_marker.type = Marker.LINE_LIST
        axes_marker.scale.x = 0.02  # Line width
        axes_marker.color.a = 1.0  # Alpha

        # Define colors for x, y, z axes
        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)    # Red for x-axis
        green = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green for y-axis
        blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)   # Blue for z-axis

        # Add x, y, z axes
        origin = Point()
        end_x = Point(x=1.0)
        end_y = Point(y=1.0)
        end_z = Point(z=1.0)

        axes_marker.points.extend([origin, end_x, origin, end_y, origin, end_z])

        # Assign colors to axes
        axes_marker.colors.extend([red, red, green, green, blue, blue])

        # Publish the axes marker
        
        self.marker_pub.publish(axes_marker)
        self.rate.sleep()
        
        
    def quaternion_from_euler(self,roll, pitch, yaw):
        
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(qx, qy, qz, qw)

    def euler_from_vector(self,vector):
        
        x, y, z = vector.x, vector.y, -vector.z
        yaw = math.atan2(y, x)
        pitch = math.atan2(z, math.sqrt(x**2 + y**2))
        roll = 0
        return roll, pitch, yaw
    
    def calculate_orientation_and_geometry(self,X, pix, lix):
        
        # Calculate direction vector
        direction_vector = Point(pix.x - X.x, pix.y - X.y, pix.z - X.z)
    
        # Calculate Euler angles and quaternion orientation
        roll, pitch, yaw = self.euler_from_vector(direction_vector)
        orientation = self.quaternion_from_euler(roll, pitch, yaw)
    
        # Calculate geometry
        rix =( math.sqrt((pix.x - X.x)**2 + (pix.y - X.y)**2 + (pix.z - X.z)**2) -lix)
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
    
    def create_cone_marker(self,frame_id, marker_id, X, orientation, hc, rcb):
        
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
    
    
    def create_spherical_cap_marker(self,frame_id, marker_id, X, orientation, slant_height, hsc, rsc, rix):
        
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
        
    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "world"  # Adjust this frame ID according to your setup
        marker.header.stamp = rospy.Time.now()
        marker.ns = "random_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # fully opaque
        marker.color.r = 1.0  # red
        marker.color.g = 0.0
        marker.color.b = 0.0

        
        self.marker_pub.publish(marker)
        self.rate.sleep()
        
        
class gse:
    
    
        
        
        
        
        
    def calculate_distance(self,point, point2):
        
        # Calculate Euclidean distance between points X and Pi_X
        # Calculate the vector components
        vector_x = point2.x - point.x
        vector_y = point2.y - point.y
        vector_z = point2.z - point.z
    
        magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        print("magnitude inside distance function:",magnitude)
    
    
        return magnitude
    
    
    def shape(self,p,x,xobs ):# p- random point , xnearest- nearest point , xbos obstacle list
        
    
        angle_difference=[]#dot product of ri
        ri=[]
        sat_fi_angle=[]
        sat_fi_distance=[] 
    
    
        for _ in range(len(xobs)):
            
        
            angle_difference.append(None)
            ri.append(None)
            sat_fi_angle.append(None)
            sat_fi_distance.append(None)
        
        
        (angle_difference,ri,sat_fi_angle,sat_fi_distance)=self.shape_obstacle_iterator(p,x,xobs )
        #print("fi:",fi)
        print("ri:",ri)
        ri_value = [int(value) for value in ri]    #ri  int
        sat_fi_int = [int(value) for value in sat_fi_angle]
        sat_fi_distance_int = [int(value) for value in sat_fi_distance]
        #ri_value = [1 if value else 0 for value in ri]
        #sat_fi_int = [1 if value else 0 for value in sat_fi_angle]
        #sat_fi_distance_int = [1 if value else 0 for value in sat_fi_distance]
        # Add corresponding integer values and store in a new list
        fi = [int(a) + int(b) for a, b in zip(sat_fi_int, sat_fi_distance_int)]#fi int
        print("fi:",fi)
        print("ri:",ri)
    
        if not all(ri):
            
            print("ALL RI IS 0 SO")
            return 0
    
        else:
            
         
            gi = []  # Initialize the list for g_i values
            for i in range(1, len(fi) + 1):
                
            
                # Calculate the summation part for each i
                summation = sum(ri_value [:i-1])
                # Calculate g_i and append to the list
                #gi.append(sat_ri_int[i-1] + summation - (i + 1))
                gi.append(fi [i-1] + summation - (i + 1))
                print("gi list",gi)
                result = 1  # Start with 1 because it's the identity value for multiplication
                for element in gi:
                    
                    result *= element
                    print("result",result)
            
                return result
        
        
    
    def shape_obstacle_iterator(self,p,x,xobs ):
            
        
    
        angle_difference=[]
        ri=[]
        sat_fi_angle=[]
        sat_fi_distance=[]
        rix=[]
        pix=[]
        lix=[]
        nix=[]
    
    
        for _ in range(len(xobs)):
            
        
            angle_difference.append(None)
            ri.append(None)
            sat_fi_angle.append(None)
            sat_fi_distance.append(None)
            
            
        for i in range(len(xobs)):
            
            rix.append(xobs[f"xobs{i}"][0])
            pix.append(xobs[f"xobs{i}"][1])
            lix.append(xobs[f"xobs{i}"][2])
            nix.append(xobs[f"xobs{i}"][3])
        
        print("shape iterator")
        print("rix:",rix)
        print("pix:",pix)
        print("lix:",lix)
        print("nix:",nix)
        
            
        
        
        
    
        #return ri,sat_ri,sat_fi,sat_fi_distance
    
        for i in range(len(xobs)):
            
            
        
            print("pix_list[i]",xobs[f"xobs{i}"])
            first_element = pix[i]
            print(first_element)
            print(first_element.x)
            print(first_element.y)
            print(first_element.z)
            pix_point_point = Point()
            pix_point_point.x = (first_element.x -0)
            pix_point_point.y = (first_element.y -0)
            pix_point_point.z = (first_element.z -0)
            #ri_X = self.calculate_distance(x, pix_point_point)
            #print("Minimum distance from point X to obstacle i (ri_X):", ri_X)

            #height = float(ri_X)
            height = float(rix[i])
            #radius = float(input("Enter the radius of the cone: "))
            radius = 0.5
            theta_i = math.atan(radius / height)
            print("angletheta_i:",theta_i)
        
        
            (angle_difference[i],ri[i],sat_fi_angle[i],sat_fi_distance[i])=self.shape_classifier(p,x,pix_point_point,nix[i],theta_i )
            frame_id = "base_link"
            shape_publisher = MarkerPublisher()
            #shape_publisher.marker_pub1.publish(x, pix_point_point, height, radius)
            shape_publisher.publish_two_points(x, pix_point_point)
            
            shape_publisher.publish_vector_between_points(x, pix_point_point)
            
            orientation, slant_height, hsc, rsc, rix_c, half_angle, rcb, hc = shape_publisher.calculate_orientation_and_geometry(x, pix_point_point, lix[i])
            cone_marker = shape_publisher.create_cone_marker(frame_id, 0, x, orientation, hc, rcb)
            cap_marker = shape_publisher.create_spherical_cap_marker(frame_id, 1, x, orientation, slant_height, hsc, rsc, rix_c)
            #marker = shape_publisher.publish_marker(x, pix_point_point, height, radius)
            shape_publisher.marker_pub1.publish(cone_marker)
            shape_publisher.marker_pub1.publish(cap_marker)
            time.sleep(0.4)
            # Code execution resumes after 5 seconds
            print("Delay complete.")
        
        print("angle_difference",angle_difference)
        print("ri",ri)
        print("sat_fi_angle",sat_fi_angle)
        print("sat_fi_distance",sat_fi_distance)
    
        return angle_difference,ri,sat_fi_angle,sat_fi_distance
    
    
    
    def shape_classifier(self,p,x,xobs,nix, theta_i):
    
        
   
        ri=0
        sat_fi_distance=0
        print("shape classifer")
    
        print("start points:",x.x,x.y,x.z)
        print("pix points",xobs.x,xobs.y,xobs.z)
    
        #nix=Vector3()
        #nix.x= (xobs.x - x.x)
        #nix.y= (xobs.y - x.y)
        #nix.z= (xobs.z - x.z)
        
       
    
        #vector for startpoint and random point
        xp=Vector3()#(P-X)
        xp.x=(p.x - x.x)
        xp.y=(p.y - x.y)
        xp.z=(p.z - x.z)
        print("v_i_x",xp.x,xp.y,xp.z)
    
        # Define the components of vectors A and B
        A_nix = [nix.x, nix.y,nix.z]
        B_xp = [xp.x, xp.y, xp.z]

        # Calculate the dot product of vectors A and B
        dot_product = sum(a * b for a, b in zip(A_nix, B_xp))
        print("dot_product",dot_product)

        # Calculate the magnitudes of vectors A and B
        magnitude_A_nix = (sum(a**2 for a in A_nix))**0.5
        magnitude_B_xp = (sum(b**2 for b in B_xp))**0.5
    
        print("magnitude_ni_X: ",magnitude_A_nix )
    
        print("magnitude_B_xp: ",magnitude_B_xp)
    
        # Calculate the angle in radians
        cos_theta = (dot_product / (magnitude_A_nix * magnitude_B_xp))
    
        cos_theta = max(min(cos_theta, 1.0), -1.0)
    
        theta_radians = math.acos(cos_theta)

        # Convert the angle to degrees
        theta_degrees = math.degrees(theta_radians)

        print("Angle between vectors A and B (in radians):", theta_radians)
        print("Angle between vectors A and B (in degrees):", theta_degrees)

        # Calculate ri
        angle_difference = (theta_radians - theta_i)
        #ri = np.arccos((dot_product) / ((magnitude_ni_X) * (magnitude_V_i_X))) - (theta_i)
        print("ri:",angle_difference )
    
  
    
    
        #ri = angle - theta_i

        # Determine if Pi_X lies within the angle theta_i
        #if ri <= theta_i:
        if angle_difference < 0:
            
            print("Point Pi_X lies within the angle theta_i")
            ri=True
        else:
            
            print("Point Pi_X lies outside the angle theta_i")
            ri=False
    
        if magnitude_A_nix > magnitude_B_xp:
            
            print("magnitude_ni_X > magnitude_B_xp: inside range")
            sat_fi_distance=True
        else:
            
            print("magnitude_ni_X < magnitude_B_xp: outside range")
            sat_fi_distance=False
    
        sat_fi_angle = not ri
        return angle_difference,ri,sat_fi_angle,sat_fi_distance
    
    
    def xobstacle(self,x,obstacles_info):
        
        
        print("indise obs------------------------------")
        
        for key, value in obstacles_info.items():
            
            print(f"{key}: {value}")
            
        print("outside obs------------------------------")
        
        num_keys = len(obstacles_info)

        # Extract 'outside_point' values and 'vertical_size' for each obstacle
        xobs = []
        offset = []
        for obstacles_data in obstacles_info.values():
            
            xobs.append(obstacles_data['outside_point'])
            offset.append(obstacles_data['vertical_size'])

        print(f"Number of obstacles: {num_keys}")
        print(f"Outside points: {xobs}")
        print(f"Vertical sizes: {offset}")
        
        
        
        
        
        xobs_dict = {}
        
        for i in range(len(xobs)):
            
            key = f"xobs{i}"
            first_element = xobs[i]
            pix=Point()
            pix.x=(first_element[0])
            pix.y=(first_element[1])
            pix.z=(first_element[2])
            #pix=scale_vector(pix, 0.5)
            rix=self.calculate_distance(x, pix)
            lix=0.5#dummy
            nix=Vector3()
            nix.x= (first_element[0] - x.x)
            nix.y= (first_element[1] - x.y)
            nix.z= (first_element[2] - x.z)
            #nix=scale_vector(nix, 0.5)
            
            #xobs_dict[key] = [rix, pix,lix,nix]
            #xobs_dict[key] = [rix, pix,offset[i],nix]
            xobs_dict[key] = [rix, pix,offset[i],nix]
            
            print("obstacle data:", xobs_dict)
            
            
            
        return xobs_dict
    
    
    
    
    def generate_random_point(self,robot_pose_x, robot_pose_y,robot_pose_z):
        # Sensor position
        sensor_x, sensor_y, sensor_z = robot_pose_x, robot_pose_y,robot_pose_z

        # Random distance
        r = random.uniform(0.1, 5)

        # Random angles
        theta = random.uniform(-math.pi, math.pi)
        phi = random.uniform(-math.pi / 12, math.pi / 12)

        # Convert spherical to Cartesian coordinates
        x = r * math.cos(phi) * math.cos(theta)
        y = r * math.cos(phi) * math.sin(theta)
        #z = r * math.sin(phi)
        z = 0

        # Translate to sensor's position
        x_translated = x + sensor_x
        y_translated = y + sensor_y
        z_translated = z + sensor_z
        #z_translated = robot_pose_z

        return x_translated, y_translated, z_translated
 
    
    
    
class Graph:
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex, x, y, z, color):
        self.vertices[vertex] = (x, y, z, color)

    def add_edge(self, start_vertex, end_vertex, cost):
        self.edges.append((start_vertex, end_vertex, cost))

    def get_neighbors(self, vertex):
        neighbors = []
        for edge in self.edges:
            if edge[0] == vertex:
                neighbors.append((edge[1], edge[2]))  # (neighbor, cost)
            elif edge[1] == vertex:
                neighbors.append((edge[0], edge[2]))  # (neighbor, cost)
        return neighbors
    def delete_vertex(self, vertex):
        if vertex in self.vertices:
            del self.vertices[vertex]
        self.edges = [edge for edge in self.edges if edge[0] != vertex and edge[1] != vertex]
        
        
    def clear(self):
        self.vertices.clear()
        self.edges.clear()

def is_connected(graph, start, goal):
    visited = set()
    stack = [start]

    while stack:
        vertex = stack.pop()
        if vertex == goal:
            return True
        if vertex not in visited:
            visited.add(vertex)
            neighbors = graph.get_neighbors(vertex)
            for neighbor, _ in neighbors:
                if neighbor not in visited:
                    stack.append(neighbor)
    return False

def dijkstra(graph, start, goal):
    queue = [(0, start, [])]  # (cost, current_vertex, path)
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)
        if current not in visited:
            visited.add(current)
            path = path + [current]
            if current == goal:
                return path
            for neighbor, neighbor_cost in graph.get_neighbors(current):
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + neighbor_cost, neighbor, path))
    return None

def publish_graph(g, start, goal, shortest_path):
    #rospy.init_node("graph_rviz")
    marker_pub_graph = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        update_markers(g, start, goal, shortest_path, marker_pub_graph)
        rate.sleep()

def update_markers(g, start, goal, shortest_path, marker_pub):
    markers = MarkerArray()

    # Create markers for points
    for idx, (vertex_name, (x, y, z, color)) in enumerate(g.vertices.items()):
        # Create a marker for each vertex
        point_marker = Marker()
        point_marker.header.frame_id = "base_link"
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = "graph"
        point_marker.id = idx
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position = Point(x=x, y=y, z=z)
        point_marker.scale = Vector3(0.2, 0.2, 0.2)
        point_marker.color = color  # Use color from the vertex
        markers.markers.append(point_marker)

        # Create a marker for the text above the point
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "graph"
        text_marker.id = idx + len(g.vertices)  # Offset by number of vertices
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(x=x, y=y, z=z + 0.5)  # Adjust z coordinate to position above the point
        text_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
        text_marker.scale = Vector3(0.1, 0.1, 0.1)
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White color
        text_marker.text = vertex_name
        markers.markers.append(text_marker)

    # Create a marker array for the lines
    lines_markers = MarkerArray()

    # Add the edges to the lines markers
    for idx, (start_vertex, end_vertex, _) in enumerate(g.edges):
        lines_marker = Marker()
        lines_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
        lines_marker.header.stamp = rospy.Time.now()
        lines_marker.ns = "graph"
        lines_marker.id = idx + 2 * len(g.vertices)  # Offset by number of vertices
        lines_marker.type = Marker.LINE_LIST
        lines_marker.action = Marker.ADD
        lines_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
        lines_marker.scale = Vector3(0.05, 0.05, 0.05)  # Line width
        lines_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue color for edges

        start_point = g.vertices[start_vertex][:3]
        end_point = g.vertices[end_vertex][:3]
        lines_marker.points.append(Point(x=start_point[0], y=start_point[1], z=start_point[2]))
        lines_marker.points.append(Point(x=end_point[0], y=end_point[1], z=end_point[2]))

        lines_markers.markers.append(lines_marker)

    # Highlight the shortest path
    path_marker = Marker()
    path_marker.header.frame_id = "base_link"
    path_marker.header.stamp = rospy.Time.now()
    path_marker.ns = "graph"
    path_marker.id = 2 * len(g.vertices) + len(g.edges)  # Offset by number of vertices and edges
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.pose.orientation = Quaternion()  # Initialize to identity quaternion
    path_marker.scale = Vector3(0.07, 0.07, 0.07)  # Line width
    path_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color for path

    if shortest_path:
        for vertex in shortest_path:
            x, y, z, _ = g.vertices[vertex]
            path_marker.points.append(Point(x=x, y=y, z=z))
        markers.markers.append(path_marker)

    marker_pub.publish(markers)
    marker_pub.publish(lines_markers)
    
    
def delete_vertex(graph, vertex_name, marker_pub):
    if vertex_name in graph.vertices:
        graph.delete_vertex(vertex_name)
        update_markers(graph, None, None, None, marker_pub)
        print(f"Vertex {vertex_name} and its related edges have been deleted.")
    else:
        print(f"Vertex {vertex_name} does not exist in the graph.")

def euclidean_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

def print_distances(graph):
    for vertex1, (x1, y1, z1, _) in graph.vertices.items():
        for vertex2, (x2, y2, z2, _) in graph.vertices.items():
            if vertex1 != vertex2:
                distance = euclidean_distance((x1, y1, z1), (x2, y2, z2))
                print(f"Distance between {vertex1} and {vertex2}: {distance:.2f}")

def print_graph(graph):
    print("Vertices:")
    for vertex, (x, y, z, color) in graph.vertices.items():
        print(f"Vertex: {vertex}, Position: ({x}, {y}, {z}), Color: {color}")
    print("\nEdges:")
    for start_vertex, end_vertex, cost in graph.edges:
        print(f"Edge: {start_vertex} -> {end_vertex}, Cost: {cost}")

def nearest(g, point):
    distances = []

    for vertex, (x, y, z, _) in g.vertices.items():
        distance = euclidean_distance((x, y, z), point)
        distances.append((distance, vertex, (x, y, z)))

    distances.sort()
    
    first_nearest_vertex, first_nearest_coords = distances[0][1], distances[0][2]
    second_nearest_vertex, second_nearest_coords = distances[1][1], distances[1][2]

    return first_nearest_vertex, first_nearest_coords, second_nearest_vertex, second_nearest_coords

def steer(first_nearest_vertex,second_nearest_vertex,x1nearest,x2nearest,xrand,graph, marker_pub,node_name,i):
    #steer(first_nearest_vertex,second_nearest_vertex,first_nearest_coords,second_nearest_coords,xrand,g, marker_pub,node_name,i)
    
    vertex_name =node_name
    x = xrand[0]
    y = xrand[1]
    z = xrand[2]
    r = 0
    g_value = 0
    b = 1
    a = 1
    point = (x, y, z)
    #first_nearest_vertex, first_nearest_coords, second_nearest_vertex, second_nearest_coords = nearest(graph, point)
    #print("First nearest point:", first_nearest_vertex, "with coordinates:", first_nearest_coords)
    #print("Second nearest point:", second_nearest_vertex, "with coordinates:", second_nearest_coords)
    
    color = ColorRGBA(r, g_value, b, a)
    graph.add_vertex(vertex_name, x, y, z, color)
    update_markers(graph, None, None, None, marker_pub)
    add_edge(graph, marker_pub, vertex_name,x1nearest,x2nearest,i,xrand,first_nearest_vertex,second_nearest_vertex)
    return vertex_name, first_nearest_vertex, x1nearest, second_nearest_vertex, x2nearest

#def add_edge(graph, marker_pub, vertex_name, first_nearest_vertex, second_nearest_vertex):
def add_edge(graph, marker_pub, vertex_name,x1nearest,x2nearest,i,xrand,first_nearest_vertex,second_nearest_vertex):
    if len(graph.vertices) < 2:
        print("At least two vertices are required to add an edge.")
        return
    
    
        
    first_nearest = vertex_name
    second_nearest = first_nearest_vertex
    
    edge_cost_to_first_vertex = euclidean_distance(xrand, xrand)
    edge_cost_to_second_vertex = euclidean_distance(xrand, x2nearest)
    print("inside edge x1nearest:",x1nearest)
    print("inside edge x2nearest:",x2nearest)
    #edge_cost_to_first_vertex = 1
    #edge_cost_to_second_vertex = 1
    
    
    graph.add_edge(vertex_name, first_nearest, edge_cost_to_first_vertex)
    graph.add_edge(vertex_name, second_nearest, edge_cost_to_second_vertex)
        
    update_markers(graph, None, None, None, marker_pub)
    
    
def add_edge_only(graph, marker_pub, vertex_name,x1nearest,i,xrand,first_nearest_vertex):
    if len(graph.vertices) < 2:
        print("At least two vertices are required to add an edge.")
        return
    
    
        
    first_nearest = vertex_name
    second_nearest = first_nearest_vertex
    
    edge_cost_to_first_vertex = euclidean_distance(xrand, xrand)
    edge_cost_to_second_vertex = euclidean_distance(xrand, x1nearest)
    print("inside edge x2nearest:",x1nearest)
    #edge_cost_to_first_vertex = 1
    #edge_cost_to_second_vertex = 1
    
    
    graph.add_edge(vertex_name, first_nearest, edge_cost_to_first_vertex)
    graph.add_edge(vertex_name, second_nearest, edge_cost_to_second_vertex)
        
    update_markers(graph, None, None, None, marker_pub)
        
 
        
        
    
def generatesample():
        
        # Generate a random point within the specified ranges
        random_x = random.uniform(-5, 5)
        random_y = random.uniform(-5, 5)
        random_z = random.uniform(0, 5)
        
        point=(random_x,  random_y,  random_z)
        return  point       

def get_all_vertex_positions(graph):
    vertex_positions = {}
    for vertex, (x, y, z, _) in graph.vertices.items():
        vertex_positions[vertex] = (x, y, z)
    return vertex_positions 

def get_vertex_position(graph, vertex_name):
    if vertex_name in graph.vertices:
        x, y, z, _ = graph.vertices[vertex_name]
        return (x, y, z)
    else:
        print(f"Vertex {vertex_name} not found in the graph.")
        return None


def nearest_intersected_shape(f,random_point,xobs,gse_sample,g,marker_pub,node_name,xrand):
    
    for index, (key, value) in enumerate(f.items()):
        
                    
        if index == len(f) - 1:
            
            break
                    
        print(f"Key: {key}, Value: {value}")
                    
        print("key:",key)
        print("value:",value)
                    #time.sleep(20)
                    
        value_point=Point()
        value_point.x=value[0]
        value_point.y=value[1]
        value_point.z=value[2]
                    
                    
        gi_temp= gse_sample.shape(random_point,value_point,xobs )
                    
        if gi_temp==0:
            
            #add_edge_only(graph, marker_pub, vertex_name,x1nearest,i,xrand,first_nearest_vertex,second_nearest_vertex)
            add_edge_only(g, marker_pub, node_name,value,i,xrand,key)
      
  
def calculate_spherical_sector_angles(X, pix, lix):
    # Calculate the distance between X and pix
    distance = math.sqrt((pix.x - X.x)**2 + (pix.y - X.y)**2 + (pix.z - X.z)**2)
    
    # Central azimuthal angle (phi_a)
    phi_a = math.atan2(pix.y - X.y, pix.x - X.x)
    
    # Azimuthal angle range (d_phi)
    d_phi = 2 * math.asin(lix / distance)
    
    # Central polar angle (theta_a)
    theta_a = math.acos((pix.z - X.z) / distance)
    
    # Polar angle range (d_theta)
    d_theta = 2 * math.asin(lix / distance)
    
    return phi_a, d_phi, theta_a, d_theta

def generate_random_point_in_spherical_sector(rix, phi_a, d_phi, theta_a, d_theta):
    r = rix * (random.uniform(0, 1)**(1/3))  # Ensure uniform distribution within the volume
    phi = phi_a + (random.uniform(-d_phi / 2, d_phi / 2))
    theta = theta_a + (random.uniform(-d_theta / 2, d_theta / 2))

    x = r * math.sin(theta) * math.cos(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)
    
    return Point(x, y, z)

def scale_vector(point, scalar):
    
    return Point(point.x * scalar, point.y * scalar, point.z * scalar) 


class MazebotGTG:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_pose_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.get_mazebot_pose)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.angular_velocity_scale = 0.5
        self.linear_velocity_scale = 0.5
        self.goal_reached_threshold = 0.1  

    def get_mazebot_pose(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        self.robot_pose.z = data.pose.pose.position.z
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_, _, self.robot_pose_yaw) = self.euler_from_quaternion(*quaternion)

    def return_pose(self):
    
        robot_x=self.robot_pose.x
        robot_y=self.robot_pose.y
        robot_z=self.robot_pose.z
        robot_yaw=self.robot_pose_yaw
        return robot_x,robot_y,robot_z,robot_yaw

    def goal_movement(self,goal_x,goal_y):
        self.goal_pose.x = goal_x
        self.goal_pose.y = goal_y

        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            K_linear = self.linear_velocity_scale

            # Calculate distance to goal
            self.distance_to_goal = sqrt((self.goal_pose.x - self.robot_pose.x)**2 + (self.goal_pose.y - self.robot_pose.y)**2)
            # Calculate angle to goal
            self.angle_to_goal = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x)
            
            current_yaw = self.robot_pose_yaw
            angle_difference = self.angle_to_goal - current_yaw

            # Normalize angle difference to be within -pi to pi
            if angle_difference > pi:
                angle_difference -= 2 * pi
            elif angle_difference < -pi:
                angle_difference += 2 * pi

            angular_velocity = self.angular_velocity_scale * angle_difference

            # First align the drone to the goal direction
            if abs(angle_difference) > 0.1:  # Some small threshold to start moving
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = angular_velocity
            else:
                # Move towards the goal
                self.vel_msg.linear.x = 0.5  # Constant linear velocity
                self.vel_msg.angular.z = 0  # Stop rotating

            self.velocity_publisher.publish(self.vel_msg)

            if self.distance_to_goal < self.goal_reached_threshold:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                rospy.loginfo("Reached the goal!")
                break

            rate.sleep()

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


        
def main():
    try:
        global marker_pub, listener
        rospy.init_node("obstacle_detection_node")
        marker_publisher = MarkerPublisher()
        mazebot_gtg = MazebotGTG()
        marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        rospy.Subscriber("/velodyne_points", PointCloud2, pointcloud_callback)
        #rospy.Subscriber("/ground_truth/state", Odometry, get_robot_pose)
        
        gse_sample = gse()
        g = Graph()

        #listener = tf.TransformListener()

        rospy.sleep(1)  # Give some time to receive the messages
        rate = rospy.Rate(30)
        # Get the start and goal vertices
        start_vertex = input("Enter start vertex name: ")
        x = float(input("Enter x coordinate for start vertex: "))
        y = float(input("Enter y coordinate for start vertex: "))
        z = float(input("Enter z coordinate for start vertex: "))
        r = 0
        g_value = 0
        b = 1
        a = 1
        start_color = ColorRGBA(r, g_value, b, a)
        g.add_vertex(start_vertex, x, y, z, start_color)

        goal_vertex = input("Enter goal vertex name: ")
        x1 = float(input("Enter x coordinate for goal vertex: "))
        y1 = float(input("Enter y coordinate for goal vertex: "))
        z1 = float(input("Enter z coordinate for goal vertex: "))
        r1 = 0
        g_value1 = 0
        b1= 1
        a1= 1
        goal_color = ColorRGBA(r1, g_value1, b1, a1)
        g.add_vertex(goal_vertex, x1, y1, z1, goal_color)
        
        
        print("\nGraph:")
        print_graph(g)
        middle_vertex = input("Enter goal vertex name: ")
        x2 = float(input("Enter x coordinate for goal vertex: "))
        y2 = float(input("Enter y coordinate for goal vertex: "))
        z2 = float(input("Enter z coordinate for goal vertex: "))
        r2 = 0
        g_value2 = 0
        b2= 1
        a2= 1
        goal_color = ColorRGBA(r2, g_value2, b2, a2)
        g.add_vertex(middle_vertex, x2, y2, z2, goal_color)
        print("\nGraph:")
        print_graph(g)
        vertex_to_delete = middle_vertex
        delete_vertex(g, vertex_to_delete, marker_pub)
        
        print("\nGraph:")
        print_graph(g)
        position = get_vertex_position(g, goal_vertex)
        if position:
            print(f"Position of vertex {goal_vertex}: {position}")
            
        
        
        
        # Example: Clear the entire graph
        g.clear()
        print("Cleared the graph.")
        print("\nUpdated Graph:")
        print_graph(g)
        
        
        

        
        
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    #workspace()
    main()
