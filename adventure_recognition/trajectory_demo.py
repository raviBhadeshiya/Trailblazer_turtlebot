#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class phantomXArm():

	def __init__(self):

		self.reset = rospy.get_param('~reset', False)

		self.sync = rospy.get_param('~sync', True)

		self.arm_joints = ['arm_shoulder_pan_joint',
					'arm_shoulder_lift_joint',
					'arm_elbow_flex_joint',
					'arm_wrist_flex_joint',
					'gripper_link_joint']

		rospy.loginfo('Waiting for right arm trajectory controller...')
		self.arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.arm_client.wait_for_server()
		rospy.loginfo('...connected.')
		self.arm_goal =  [0, 0, 0, 0, 0]

	def setGoal(self,goal=[0, 0, 0, 0, 0]):

		self.arm_goal = goal
		self.arm_trajectory = JointTrajectory()
		self.arm_trajectory.joint_names = self.arm_joints
		self.arm_trajectory.points.append(JointTrajectoryPoint())
		self.arm_trajectory.points[0].positions = self.arm_goal
		self.arm_trajectory.points[0].velocities = [0.0 for i in self.arm_joints]
		self.arm_trajectory.points[0].accelerations = [0.0 for i in self.arm_joints]
		self.arm_trajectory.points[0].time_from_start = rospy.Duration(3.0) 

	def armMoveToGoal(self):

		rospy.loginfo('Moving the arm to goal position...')

		arm_goal = FollowJointTrajectoryGoal()

		arm_goal.trajectory = self.arm_trajectory

		arm_goal.goal_time_tolerance = rospy.Duration(0.0)

		self.arm_client.send_goal(arm_goal)
		
		if not self.sync:

			self.arm_client.wait_for_result(rospy.Duration(60))

		rospy.loginfo('...done')
