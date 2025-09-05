#!/usr/bin/env python3

import moveit_commander
import rospy
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh
from geometry_msgs.msg import PoseStamped

# Initialize the moveit_commander
moveit_commander.roscpp_initialize([])
rospy.init_node('hector_moveit_collision_checking')

# Instantiate a `RobotCommander` object
robot = moveit_commander.RobotCommander()

# Instantiate a `PlanningSceneInterface` object
scene = moveit_commander.PlanningSceneInterface()

# Define a collision object (e.g., a box)
collision_object = CollisionObject()
collision_object.id = "box"
collision_object.header.frame_id = robot.get_planning_frame()

# Define the shape and size of the box
box = SolidPrimitive()
box.type = box.BOX
box.dimensions = [0.1, 0.1, 0.1]

# Define the pose of the box (position + orientation)
box_pose = PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.5
box_pose.pose.position.z = 0.5
box_pose.pose.orientation.w = 1.0

# Add the box to the collision object
collision_object.primitives.append(box)
collision_object.primitive_poses.append(box_pose.pose)

# Add the collision object to the planning scene
scene.add_collision_objects([collision_object])

# Now you can use the planning scene to check for collisions
collision_detected = scene.check_collision(collision_object)

print("Collision detected:", collision_detected)

