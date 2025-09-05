#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math
import random

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

def create_point_marker(frame_id, marker_id, point):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "point"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position = point
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0, 0, 1.0, 1.0)
    return marker

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

def main():
    rospy.init_node("spherical_sector_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    frame_id = "base_link"
    X = Point(0, 0, 0)
    pix = Point(5, 5, 5)
    lix = 1.0  

    orientation, slant_height, hsc, rsc, rix, half_angle, rcb, hc = calculate_orientation_and_geometry(X, pix, lix)

    phi_a, d_phi, theta_a, d_theta = calculate_spherical_sector_angles(X, pix, lix)

    print("Orientation:", orientation)
    print("Slant Height:", slant_height)
    print("Height of Cap:", hsc)
    print("Radius of Cap:", rsc)
    print("Height of cone:", hc)
    print("Radius of cone:", rcb)
    print("lix:", lix)
    print(f"phi_a: {phi_a}, d_phi: {d_phi}, theta_a: {theta_a}, d_theta: {d_theta}")

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        cone_marker = create_cone_marker(frame_id, 0, X, orientation, hc, rcb)
        cap_marker = create_spherical_cap_marker(frame_id, 1, X, orientation, slant_height, hsc, rsc, rix)

        # Generate a random point in the spherical sector
        random_point = generate_random_point_in_spherical_sector(rix, phi_a, d_phi, theta_a, d_theta)
        point_marker = create_point_marker(frame_id, 2, random_point)

        pub.publish(cone_marker)
        pub.publish(cap_marker)
        pub.publish(point_marker)
        
        rate.sleep()

if __name__ == "__main__":
    main()
