#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def main():
    rospy.init_node("ur3_joint_controller_node")
    action_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for Server")
    action_client.wait_for_server()
    rospy.loginfo("Connected to Action Server")
    list_points = []
    pt_1 = JointTrajectoryPoint()
    pt_1.positions = [1.0]
    pt_1.time_from_start = rospy.Duration(1)
    list_points.append(pt_1)
    pt_2 = JointTrajectoryPoint()
    pt_2.positions = [-1.0]
    pt_2.time_from_start = rospy.Duration(2)
    list_points.append(pt_2)
    traj = FollowJointTrajectoryGoal()
    traj.trajectory.joint_names = ['wrist_2_joint']
    traj.trajectory.points = list_points
    action_client.send_goal(traj)
    rospy.loginfo("Waiting for result")
    action_client.wait_for_result()
    rospy.loginfo("%s", action_client.get_result())
    rospy.loginfo("Execution finish !")

if __name__ == "__main__":
    main()
