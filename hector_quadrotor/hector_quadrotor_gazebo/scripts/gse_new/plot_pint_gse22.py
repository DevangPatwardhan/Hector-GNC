#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import time

class MarkerPublisher:
    def __init__(self):
        rospy.init_node("two_points_and_vector_rviz", anonymous=True)  # Added anonymous=True to ensure unique node names
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.marker_pub1 = rospy.Publisher("visualization_marker2", Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # Added a rate to control the publishing frequency
        
        
        
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
    
        if all(ri):
            
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
            radius = float(input("Enter the radius of the cone: "))
            theta_i = math.atan(radius / height)
            print("angletheta_i:",theta_i)
        
        
            (angle_difference[i],ri[i],sat_fi_angle[i],sat_fi_distance[i])=self.shape_classifier(p,x,pix_point_point,nix[i],theta_i )
            
            shape_publisher = MarkerPublisher()
            #shape_publisher.marker_pub1.publish(x, pix_point_point, height, radius)
            shape_publisher.publish_two_points(x, pix_point_point)
            
            shape_publisher.publish_vector_between_points(x, pix_point_point)
            
            marker = shape_publisher.publish_marker(x, pix_point_point, height, radius)
            shape_publisher.marker_pub1.publish(marker)
            time.sleep(3)
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
    
    
    def xobstacle(self,x):
        
        xobs=[(-3,-3,3),(4,3,4)]
        xobs_dict = {}
        
        for i in range(len(xobs)):
            
            key = f"xobs{i}"
            first_element = xobs[i]
            pix=Point()
            pix.x=(first_element[0]-0)
            pix.y=(first_element[1]-0)
            pix.z=(first_element[2]-0)
            rix=self.calculate_distance(x, pix)
            lix=0.5#dummy
            nix=Vector3()
            nix.x= (first_element[0] - x.x)
            nix.y= (first_element[1] - x.y)
            nix.z= (first_element[2] - x.z)
            
            xobs_dict[key] = [rix, pix,lix,nix]
            
            print("obstacle data:", xobs_dict)
            
            
            
        return xobs_dict
            
            
            
            
        
        
        
        
        
        
        


def main():
    try:
        marker_publisher = MarkerPublisher()
        #point1 = Point(x=0.0, y=0.0, z=0.0)  # Example point, replace with actual values
        #point2 = Point(x=10.0, y=10, z=10)
        #point3 = Point(x=2.0, y=10, z=10)# Example point, replace with actual values
        
        xnearest = [float(coord) for coord in
                    input("Enter coordinates of point 1 (comma separated x, y, z): ").split(',')]
        
        xrand = [float(coord) for coord in
               input("Enter coordinates of point point on obstacle i to X (comma separated x, y, z): ").split(',')]
        
        x = Point()
        x.x = (xnearest[0]-0)
        x.y = (xnearest[1]-0)
        x.z = (xnearest[2]-0)
        random_point = Point()
        random_point.x = (xrand[0]-0)
        random_point.y = (xrand[1]-0)
        random_point.z = (xrand[2]-0)
        
        #xobs=[(-3,-3,3),(5,5,5)]
        
        ##xobs_point1.x = (-3 -0)
        #xobs_point1.y = (-3 -0)
        #xobs_point1.z = (3 -0)
        ##xobs_point2.x = (5 -0)
        #xobs_point2.y = (5 -0)
        #xobs_point2.z = (5 -0)
        
        marker_publisher.publish_axes()
        marker_publisher.publish_single_point(random_point)
        
        gse_sample = gse() #xobs_dict[key] = [rix, pix,lix,nix]
        xobs=gse_sample.xobstacle(x)
        gi_value= gse_sample.shape(random_point,x,xobs ) #shape(self,p,x,xobs )
        print("gi value:",gi_value)
        
        
        
        #marker_publisher.publish_two_points(x, xobs_point1)
        #marker_publisher.publish_two_points(x, xobs_point2)
        #marker_publisher.publish_vector_between_points(x, xobs_point1)
        #marker_publisher.publish_vector_between_points(x, xobs_point2)
        marker_publisher.rate.sleep()


        while not rospy.is_shutdown():
            
            #marker_publisher.publish_axes()
            #marker_publisher.publish_single_point(random_point)
            #marker_publisher.publish_two_points(x, xobs_point1)
            #marker_publisher.publish_two_points(x, xobs_point2)
            #marker_publisher.publish_vector_between_points(x, xobs_point1)
            #marker_publisher.publish_vector_between_points(x, xobs_point2)
            
            
            
        
            marker_publisher.rate.sleep()  # Sleep to maintain the loop rate
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
