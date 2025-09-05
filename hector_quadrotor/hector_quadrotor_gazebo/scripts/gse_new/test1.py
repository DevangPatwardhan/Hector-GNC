#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import time


class ShapePublisher:
    def __init__(self):
        rospy.init_node('shape_publisher')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        
        self.marker_pub1 = rospy.Publisher("visualization_marker2", Marker, queue_size=10)
        self.rate1 = rospy.Rate(1) 

    def publish_marker(self,point, point2 ,height, radius):
        
        # Calculate the vector components
        vector_x = point2.x - point.x
        vector_y = point2.y - point.y
        vector_z = point2.z - point.z

        # Calculate the magnitude of the vector (Euclidean distance)
        magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        print("magnitude:",magnitude)

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
        print("Angle of the cone1: {:.2f} degrees".format(theta_degrees))

        return marker


    def publish_vector_between_points(self,point1, point2):
        
        # Create a marker for the vector arrow
        vector_marker = Marker()
        vector_marker.header.frame_id = "base_link"  # Replace with a valid frame ID
        vector_marker.header.stamp = rospy.Time.now()
        vector_marker.ns = "vector_between_points"
        vector_marker.action = Marker.ADD
        vector_marker.pose.orientation.w = 1.0
        vector_marker.id = 0
        vector_marker.type = Marker.ARROW
        vector_marker.scale.x = 0.05  # Arrow shaft diameter
        vector_marker.scale.y = 0.1  # Arrow head diameter
        vector_marker.scale.z = 0.2  # Arrow head length
        vector_marker.color.r = 1.0
        vector_marker.color.g = 0.0  # Red color
        vector_marker.color.b = 0.0
        vector_marker.color.a = 1.0  # Alpha

        # Calculate vector direction
        vector_direction = Point()
        vector_direction.x = point2.x - point1.x
        vector_direction.y = point2.y - point1.y
        vector_direction.z = point2.z - point1.z
        # Set arrow position to the midpoint between the two points
        vector_marker.pose.position.x = point1.x 
        vector_marker.pose.position.y = point1.y 
        vector_marker.pose.position.z = point1.z 
        # Set arrow orientation
        vector_marker.pose.orientation.x = 0.0
        vector_marker.pose.orientation.y = 0.0
        vector_marker.pose.orientation.z = 0.0
        vector_marker.pose.orientation.w = 1.0
        # Set arrow direction
        vector_marker.points.append(point1)
        vector_marker.points.append(point2)
        # Publish the vector marker
        self.marker_pub.publish(vector_marker)

    def publish_points(self, point, point2):
        # Create a marker for the points
        points_marker = Marker()
        points_marker.header.frame_id = "base_link"
        points_marker.header.stamp = rospy.Time.now()
        points_marker.ns = "points"
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.id = 0
        points_marker.type = Marker.LINE_STRIP
        points_marker.scale.x = 0.05  # Line width
        points_marker.color.r = 1.0  # Red color
        points_marker.color.a = 1.0  # Alpha

        # Set pink color
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 1.0

        # Add the points to the marker
        points_marker.points.append(point)
        points_marker.points.append(point2)

        # Publish the points marker
        self.marker_pub.publish(points_marker)

        # Create markers for the points
        point_marker1 = Marker()
        point_marker1.header.frame_id = "base_l can you convert this code with classink"
        point_marker1.header.stamp = rospy.Time.now()
        point_marker1.ns = "points"
        point_marker1.action = Marker.ADD
        point_marker1.pose.orientation.w = 1.0
        point_marker1.id = 1
        point_marker1.type = Marker.SPHERE
        point_marker1.scale.x = 0.1  # Sphere size
        point_marker1.scale.y = 0.1
        point_marker1.scale.z = 0.1
        point_marker1.color.r = 0.0  # Red color
        point_marker1.color.g = 1.0  # Green color
        point_marker1.color.b = 1.0  # Blue color
        point_marker1.color.a = 1.0  # Alpha
        point_marker1.pose.position = point

        point_marker2 = Marker()
        point_marker2.header.frame_id = "base_link"
        point_marker2.header.stamp = rospy.Time.now()
        point_marker2.ns = "points"
        point_marker2.action = Marker.ADD
        point_marker2.pose.orientation.w = 1.0
        point_marker2.id = 2
        point_marker2.type = Marker.SPHERE
        point_marker2.scale.x = 0.1  # Sphere size
        point_marker2.scale.y = 0.1
        point_marker2.scale.z = 0.1
        point_marker2.color.r = 0.0  # Red color
        point_marker2.color.g = 1.0  # Green color
        point_marker2.color.b = 1.0  # Blue color
        point_marker2.color.a = 1.0  # Alpha
        point_marker2.pose.position = point2

        # Publish the point markers
        self.marker_pub.publish(point_marker1)
        self.marker_pub.publish(point_marker2)

    

    def publish_single_point(self,point3):
        
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
 
        point_marker.points.append(point3)
 
        self.marker_pub1.publish(point_marker)

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

    def vector_between_points(self,point, point2):
        
    
        return [point2[0] - point[0], point2[1] - point[1], point2[2] - point[2]]

    def calculate_angles(self,point, point2):
        
        # Calculate the vector components
        vector_x = point2.x - point.x
        vector_y = point2.y - point.y
        vector_z = point2.z - point.z
        # Calculate the magnitude of the vector
        magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        # Calculate angles with x, y, z axes
        theta_x = math.degrees(math.acos(vector_x / magnitude))
        theta_y = math.degrees(math.acos(vector_y / magnitude))
        theta_z = math.degrees(math.acos(vector_z / magnitude))

        return theta_x, theta_y, theta_z

    def angle_between_vectors(self,X, Pi_X):
        
        dot_product = np.dot(X, Pi_X)
        cos_angle = dot_product / (np.linalg.norm(X) * np.linalg.norm(Pi_X))
        return np.arccos(np.clip(cos_angle, -1.0, 1.0))



    def calculate_distance(self,point, point2):
        
        # Calculate Euclidean distance between points X and Pi_X
        # Calculate the vector components
        vector_x = point2.x - point.x
        vector_y = point2.y - point.y
        vector_z = point2.z - point.z
    
        magnitude = math.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        print("magnitude inside distance function:",magnitude)
        return magnitude


    def construct_normal_vector(self,X, Pi_X):
        
        # Calculate the vector v from X to Pi_X
        v = Pi_X - X

        # Calculate the magnitude of v
        v_magnitude = np.linalg.norm(v)

        # Normalize the vector v to get the unit vector u
        u = v / v_magnitude

        # Construct the normal vector n_i;X by permuting components of the unit vector u
        n_i_X = np.array([u[1], u[2], -u[0]])

        return n_i_X

    def calculate_ri(self,X, P_o , ni_X, theta_i,O_o ):
        
        # Calculate V_i_X
        #fp=O_o - P_o
        #sp=O_o - X
        sat_ri=0
        sat_fi_distance=0
        fp=P_o - O_o
        sp=X - O_o
        print("fp",fp[0],fp[1],fp[2])
        print("sp",sp[0],sp[1],sp[2])
    
        V_i_X = fp - sp
        print("v_i_x",V_i_X[0],V_i_X[1],V_i_X[2])
    
        #V_i_X = P_o - X
        #print("v_i_x",V_i_X[0],V_i_X[1],V_i_X[2])
        # Calculate dot product1,1
        dot_product = np.dot(ni_X, V_i_X)
        print("dot_product",dot_product)
        # Calculate magnitudes
        magnitude_ni_X = np.linalg.norm(ni_X)
        print("magnitude_ni_X",magnitude_ni_X)
        magnitude_V_i_X = np.linalg.norm(V_i_X)
        print("magnitude_V_i_X",magnitude_V_i_X)
        # Calculate the angle
        angle = np.arccos(np.clip(dot_product / (magnitude_ni_X * magnitude_V_i_X), -1.0, 1.0))
        #angle = np.arccos((dot_product) / ((magnitude_ni_X) * (magnitude_V_i_X)))
        print("angle:", angle )
        # Calculate ri
        ri = np.arccos(np.clip(dot_product / (magnitude_ni_X * magnitude_V_i_X), -1.0, 1.0)) - theta_i
        #ri = np.arccos((dot_product) / ((magnitude_ni_X) * (magnitude_V_i_X))) - (theta_i)
        print("ri:",ri )
        # Determine if Pi_X lies within the angle theta_i
        #if ri <= theta_i:
        if ri < 0:
            
            print("Point Pi_X lies within the angle theta_i")
            sat_ri=True
        else:
            
            print("Point Pi_X lies outside the angle theta_i")
            sat_ri=False
    
        if magnitude_ni_X > magnitude_V_i_X:
            
            print("magnitude_ni_X > magnitude_V_i_X: inside range")
            sat_fi_distance=True
        else:
            
            print("magnitude_ni_X < magnitude_V_i_X: outside range")
            sat_fi_distance=False
    
        sat_fi = not sat_ri
        return ri,sat_ri,sat_fi,sat_fi_distance
    
    def run(self):
        point_coords = [float(coord) for coord in
                        input("Enter coordinates of point 1 (comma separated x, y, z): ").split(',')]
        point2_coords = [float(coord) for coord in
                         input("Enter coordinates of point 2 (comma separated x, y, z): ").split(',')]
        P_o = [float(coord) for coord in
               input("Enter coordinates of point point on obstacle i to X (comma separated x, y, z): ").split(',')]

        X = np.array(point_coords)
        Pi_X = np.array(point2_coords)
        Point_p = np.array(P_o)
        origin = np.array([0.0, 0.0, 0.0])

        point = Point()
        point.x = point_coords[0]
        point.y = point_coords[1]
        point.z = point_coords[2]
        
        #origin = Point()
        #origin.x = 0
        #origin.y = 0
        #origin.z = 0

        point2 = Point()
        point2.x = point2_coords[0]
        point2.y = point2_coords[1]
        point2.z = point2_coords[2]

        point3 = Point()
        point3.x = P_o[0]
        point3.y = P_o[1]
        point3.z = P_o[2]

        ri_X = self.calculate_distance(point, point2)
        ri_x_vector = self.vector_between_points(X, Pi_X)
        height = float(ri_X)
        radius = float(input("Enter the radius of the cone: "))
        theta_i = math.atan(radius / height)

        (ri, sat_ri, sat_fi, sat_fi_distance) = self.calculate_ri(X, Point_p, ri_x_vector, theta_i, origin)
        print("ri:", ri)
        print("sat_ri:", sat_ri)
        print("sat_fi:", sat_fi)
        print("sat_fi_distance:", sat_fi_distance)

        while not rospy.is_shutdown():
            self.publish_axes()
            self.publish_single_point(point3)
            self.publish_points(point, point2)
            self.publish_vector_between_points(point, point2)
            marker = self.publish_marker(point, point2, height, radius)
            self.marker_pub.publish(marker)
            self.rate.sleep()

if __name__ == "__main__":
    #shape_publisher = ShapePublisher()
    #shape_publisher.run()
    nix=Vector3()
    nix.x= 1
    nix.y= 2
    nix.z= 3
     
    nix1=Vector3()
    nix1.x= 3
    nix1.y= 2
    nix1.z= 1
    
    my_dict = {
    'xobs0': [nix, 3.0],
    'xobs1': [nix1, 5.0, 5.0, 5.0]
          }
    
    xobs0_first_value = my_dict['xobs0'][0]
    xobs1_first_value = my_dict['xobs1'][0]
    
    print(xobs0_first_value.z)
    
    # Creating a single list
    combined_list = [xobs0_first_value, xobs1_first_value]

    print(f"Combined list: {combined_list}")
    rix=[]
    
    #for i in range(len(my_dict)):
        
        #rix.append(None)
    
    for i in range(len(my_dict)):
        
        rix.append(my_dict[f"xobs{i}"][0])
        
    print("rix:",rix)
