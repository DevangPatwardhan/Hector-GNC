#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
def generate_trajectory(start_pose, goal_pose):
    # Initialize moveit_commander
    rospy.init_node('rrt_planner')
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    rospy.sleep(1)

    # Set start and goal poses
    start = PoseStamped()
    start.header.frame_id = "base_link"
    start.pose = start_pose

    goal = PoseStamped()
    goal.header.frame_id = "base_link"
    goal.pose = goal_pose

    # Use OMPL RRT planner
    rospy.wait_for_service('/compute_ompl_plan')
    compute_ompl_plan = rospy.ServiceProxy('/compute_ompl_plan', GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = "manipulator"
    req.ik_request.pose_stamped = goal
    res = compute_ompl_plan(req)

    # Extract trajectory
    if res.error_code.val == res.error_code.SUCCESS:
        trajectory = res.solution.joint_trajectory
        return trajectory
    else:
        rospy.logerr("Failed to find trajectory: {}".format(res.error_code))

def main():
    start_pose = Pose()
    start_pose.position.x = 0.0
    start_pose.position.y = 0.0
    start_pose.position.z = 0.0
    start_pose.orientation.w = 1.0

    goal_pose = Pose()
    goal_pose.position.x = 1.0
    goal_pose.position.y = 1.0
    goal_pose.position.z = 1.0
    goal_pose.orientation.w = 1.0

    trajectory = generate_trajectory(start_pose, goal_pose)
    if trajectory:
        rospy.loginfo("Trajectory generated successfully.")
        rospy.loginfo("Trajectory: {}".format(trajectory))
    else:
        rospy.logerr("Failed to generate trajectory.")

if __name__ == "__main__":
    main()
