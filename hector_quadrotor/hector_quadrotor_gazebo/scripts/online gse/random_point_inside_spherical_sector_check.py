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

"""
For a given sampled point X, first, the
minimum distance of X from the ith obstacle in X obs , denoted as rix, is obtained
let the point on obstacle i, which is at a minimum
distance rix from the sampled point be given as pix
Then, a plane
is constructed whose normal is the vector from point X to pix and
passing through point pix (for illustration, see Fig. 2). Let this
normal vector be denoted by nix . Next, the set of points of inter-
section of this plane and the lines joining the points on obstacle i and
the sampled point X are found and denoted as Qix . The maximum
distance of point pix from the points in Qix can be given as lix 
"""

"""
hc - height of the cone
hsc - height of the spherical cap
rcb - radius of the cone base
rsc - radius of the spherical cap
slant height - slant height of the cone
"""

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

def generate_random_point_in_spherical_sector(apex, rix, phi_a, d_phi, theta_a, d_theta):
    r = random.uniform(0, rix)
    phi = phi_a + random.uniform(-d_phi / 2, d_phi / 2)
    theta = theta_a + random.uniform(-d_theta / 2, d_theta / 2)

    # Spherical to Cartesian conversion
    x = r * math.sin(theta) * math.cos(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)

    # Translate point by the apex coordinates
    translated_x = x + apex.x
    translated_y = y + apex.y
    translated_z = z + apex.z

    return Point(translated_x, translated_y, 3)

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

def publish_single_point(point):
    
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
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

    # Set the position of the point
    single_point = Point()
    single_point.x = point.x
    single_point.y = point.y
    single_point.z = point.z

    # Add the point to the marker
    point_marker.points.append(single_point)

    
    marker_pub.publish(point_marker)
    
def calculate_distance(point, point2):
    
        
    # Calculate Euclidean distance between points X and Pi_X
    # Calculate the vector components
    vector_x = point2.x - point.x
    vector_y = point2.y - point.y
    vector_z = point2.z - point.z

    magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
    print("magnitude inside distance function:",magnitude)


    return magnitude

       
        
def euclidean_distance(point1, point2):
    return math.sqrt((point2.x - point1.x)**2 + 
                     (point2.y - point1.y)**2 + 
                     (point2.z - point1.z)**2)
    
def shape(p,x,xobs ):# p- random point , xnearest- nearest point , xbos obstacle list
    
        
    
    angle_difference=[]#dot product of ri
    ri=[]
    sat_fi_angle=[]
    sat_fi_distance=[] 


    for _ in range(len(xobs)):
        
    
        angle_difference.append(None)
        ri.append(None)
        sat_fi_angle.append(None)
        sat_fi_distance.append(None)
    
    
    (angle_difference,ri,sat_fi_angle,sat_fi_distance)=shape_obstacle_iterator(p,x,xobs )
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

    if all(ri):
        
        print("ALL RI IS 1 SO")
        return 0

    else:
        
        
        
        gi = []  # Initialize the list for g_i values
        for i in range(1, len(fi) + 1):
            
            # Calculate the summation part for each i
            summation = sum(ri_value[:i-1])
            # Calculate g_i and append to the list
            gi.append(fi[i-1] + summation - (i + 1))

        print("gi list", gi)

        result = 1  # Start with 1 because it's the identity value for multiplication
        for element in gi:
            
            result *= element
            print("result", result)

        # Ensure we return the final result correctly
        final_result = result
        print("Final result:", final_result)
        
        return final_result

def shape_obstacle_iterator(p,x,xobs):
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
        
        rix.append(xobs[f"xobs0"][0])
        pix.append(xobs[f"xobs0"][1])
        lix.append(xobs[f"xobs0"][2])
        nix.append(xobs[f"xobs0"][3])
    
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
        #radius = 0.5
        radius = float(lix[i])
        theta_i = math.atan((radius / height))
        print("angletheta_i:",theta_i)
        
        (angle_difference[i],ri[i],sat_fi_angle[i],sat_fi_distance[i])=shape_classifier(p,x,pix_point_point,nix[i],theta_i )
    
       
        
        
      
            
        
        #time.sleep(0.4)
        # Code execution resumes after 5 seconds
        print("Delay complete.")
    
    print("angle_difference",angle_difference)
    print("ri",ri)
    print("sat_fi_angle",sat_fi_angle)
    print("sat_fi_distance",sat_fi_distance)

    return angle_difference,ri,sat_fi_angle,sat_fi_distance

def shape_classifier(p,x,xobs,nix, theta_i):
    
    
        
   
    ri=0
    sat_fi_distance=0
    print("shape classifer")

    print("start points:",x.x,x.y,x.z)
    print("goal points",p.x,p.y,p.z)

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
    
    # Check for zero magnitudes to avoid division by zero
    if magnitude_A_nix == 0 or magnitude_B_xp == 0:
        
        print("One of the vectors is a zero vector; cannot compute cosine of the angle.")
        cos_theta = 0  # or any other appropriate value or handling
    else:
        cos_theta = dot_product / (magnitude_A_nix * magnitude_B_xp)


    # Calculate the angle in radians
    # cos_theta = (dot_product / (magnitude_A_nix * magnitude_B_xp))

    #cos_theta = max(min(cos_theta, 1.0), -1.0)

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
        ri=False
    else:
        
        print("Point Pi_X lies outside the angle theta_i")
        ri=True

    if magnitude_A_nix > magnitude_B_xp:
        
        print("magnitude_ni_X > magnitude_B_xp: inside range")
        sat_fi_distance=True
    else:
        
        print("magnitude_ni_X < magnitude_B_xp: outside range")
        sat_fi_distance=False

    sat_fi_angle = not ri
    return angle_difference,ri,sat_fi_angle,sat_fi_distance




def main():
    rospy.init_node("spherical_sector_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    

    #print("Orientation:", orientation)
    #print("Slant Height:", slant_height)
    #print("Height of Cap:", hsc)
    #print("Radius of Cap:", rsc)
    #print("Height of cone:", hc)
    #print("Radius of cone:", rcb)
    #print("lix:", lix)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        X_point=Point(-1,-1,1)
        pix_point=Point(1,1,5)
        rix = euclidean_distance(X_point, pix_point)
        print("rix:",rix)
        frame_id = "base_link"
        X = Point(X_point.x, X_point.y, X_point.z)
        pix = Point(pix_point.x, pix_point.y, pix_point.z)
        lix=3.0
        nix=Vector3()
        nix.x= (pix.x - X.x)
        nix.y= (pix.y - X.y)
        nix.z= (pix.z - X.z)
        xobs={"xobs0":[rix, pix,lix,nix]}
         
        phi_a, d_phi, theta_a, d_theta=calculate_spherical_sector_angles(X, pix, lix)
        print("phi_a, d_phi, theta_a, d_theta:",phi_a, d_phi, theta_a, d_theta)
        random_pont=generate_random_point_in_spherical_sector(X,rix, phi_a, d_phi, theta_a, d_theta)
        #random_pont=Point(9,9,1)
        print("random_pont:",random_pont)
        orientation, slant_height, hsc, rsc, rix, half_angle, rcb, hc = calculate_orientation_and_geometry(X, pix, lix)
        print("orientation:")
        cone_marker = create_cone_marker(frame_id, 0, X, orientation, hc, rcb)
        print("cone_marker :")
        cap_marker = create_spherical_cap_marker(frame_id, 1, X, orientation, slant_height, hsc, rsc, rix)
        print("cap_marker:")
        publish_single_point(random_pont)
        print("publish_single_point:")
        pub.publish(cone_marker)
        print("pub.publish")
        pub.publish(cap_marker)
        print("i")
        indicator1=shape(random_pont,X,xobs )
        print("gi value:",indicator1)
        #time.sleep(5)
        a=input("enter enter")
        rate.sleep()

if __name__ == "__main__":
    main()
