#!/usr/bin/env python 

import rospy
from wheeled_robot_kinematics.srv import *
from wheeled_robot_kinematics.msg import *
from project_2.kinematics import *
from geometry_msgs.msg import *
 
def forwardSrv(req):
	axle_length=rospy.get_param("/axle_length")
	wheel_radius=rospy.get_param("/wheel_radius")
	max_speed=rospy.get_param("/max_speed")
	
	rd=(axle_length, wheel_radius, max_speed)
	(x,y,theta)=forward((req.pose.x,req.pose.y,req.pose.theta),(req.action.left_velocity,req.action.right_velocity,req.action.time),rd)
	
	resp=DiffDriveFKResponse()
	resp.end_pose=Pose2D(x,y,theta)
	#end_pose = apply(Pose2D, end_pose1)
	return resp#end_pose

def inverseSrv(req):
	axle_length=rospy.get_param("/axle_length")
	wheel_radius=rospy.get_param("/wheel_radius")
	max_speed=rospy.get_param("/max_speed")
	
	rd=(axle_length, wheel_radius, max_speed)
	res=inverse((req.pose.x,req.pose.y,req.pose.theta),(req.end_pose.x,req.end_pose.y,req.end_pose.theta),rd)
	resp = DiffDriveIKResponse()
	
	for entry in res:
		(vl, vr, t) = entry
		resp.actions.append( DiffDriveAction(vl, vr, t))
	
	return resp


if __name__ == "__main__":
	rospy.init_node('kinematics')
	s1 = rospy.Service('~forward', DiffDriveFK, forwardSrv)
	s2 = rospy.Service('~inverse', DiffDriveIK, inverseSrv)
	print "Service Running Now..." 
	rospy.spin()