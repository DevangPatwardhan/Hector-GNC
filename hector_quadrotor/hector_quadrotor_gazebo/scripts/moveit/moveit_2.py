#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import sys

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rrt_star_path_planner', anonymous=True)

    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object
    group_name = "YOUR_ROBOT_ARM_GROUP_NAME"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set the planner to RRT*
    move_group.set_planner_id("RRTstar")

    # Get the name of the end-effector link
    end_effector_link = move_group.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "YOUR_REFERENCE_FRAME"
    move_group.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the chances of a solution
    move_group.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.1)

    # Get the start and goal coordinates from the user
    start_x, start_y, start_z = map(float, raw_input("Enter start point x y z: ").split())
    goal_x, goal_y, goal_z = map(float, raw_input("Enter goal point x y z: ").split())

    # Set the start state to the current state
    move_group.set_start_state_to_current_state()

    # Create a pose for the start and goal
    start_pose = geometry_msgs.msg.Pose()
    start_pose.position.x = start_x
    start_pose.position.y = start_y
    start_pose.position.z = start_z

    goal_pose = geometry_msgs.msg.Pose()
    goal_pose.position.x = goal_x
    goal_pose.position.y = goal_y
    goal_pose.position.z = goal_z

    # Set the start and goal pose
    move_group.set_pose_target(start_pose)
    move_group.go(wait=True)
    move_group.set_pose_target(goal_pose)

    # Plan and execute the path
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Visualize the path in RViz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Publish the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

if __name__ == '__main__':
    main()
