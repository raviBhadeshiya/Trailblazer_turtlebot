#!/usr/bin/env python
import rospy, pcl_ros, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry

import random as random

import cv2, math, pcl
import numpy as np

pub = rospy.Publisher('/slam_debug', MarkerArray)
MIN_Inliers=75

var_pLines=[]


def get_line(p1, v1, id_, color=(0,0,1)):
	marker = Marker()
	marker.header.frame_id = "camera_depth_frame"
	marker.header.stamp = rospy.Time()
	marker.lifetime = rospy.Duration(1)
	marker.ns = "slam_debug_ns"
	marker.id = id_
	marker.type = Marker.LINE_STRIP
	marker.action = Marker.ADD
	marker.pose.position.x = 0
	marker.pose.position.y = 0
	marker.pose.position.z = 0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 0.0
	marker.scale.x = 0.01
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.a = 1.0
	marker.color.r = color[0]
	marker.color.g = color[1]
	marker.color.b = color[2]
	marker.points.append(Point(p1[0] + 100 * v1[0], p1[1] + 100 * v1[1], 0))
	marker.points.append(Point(p1[0] - 100 * v1[0], p1[1] - 100 * v1[1], 0))
	return marker


def removeInlinear(pcl_points,data):
	npData=np.asarray(data,dtype=np.float32)
	npData=set(tuple(x) for x in npData)
	pcl_points_set=set(tuple(x) for x in np.array(pcl_points,dtype=np.float32))
	pcl_points_set=pcl_points_set.difference(pcl_points_set.intersection(npData))
	pcl_points_set=list(pcl_points_set)
	return pcl_points_set


def fitLine(p):
	## create a segmenter object
	seg = p.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_LINE)
	seg.set_method_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold (0.003)
	## apply RANSAC
	indices, model = seg.segment()
	return indices,model


def angleBetweenLines(line1,line2):
	(x1,y1),(x2,y2)=line1
	(px1,py1),(px2,py2)=line2

	angel=abs(math.atan((y2-y1)/float(x2-x1))-math.atan((py2-py1)/float(px2-px1)))
	# angel=abs(math.atan2((y2-y1),(x2-x1))-math.atan2((py2-py1),(px2-px1)))
	return angel

def isParallel(line1,line2,thre=5.0):

	angle=angleBetweenLines(line1,line2)
	
	if angle <= thre * (math.pi/180):
			return True

	return False


def parallelDist(line1,line2):
	(x1,y1),(x2,y2)=line1
	(px1,py1),(px2,py2)=line2
	
	slope=((y2-y1)/float(x2-x1)) + ((py2-py1)/float(px2-px1))
	slope/=2.0


	c=y1-slope*x1
	pc=py1-slope*px1

	return abs(c-pc)/math.hypot(slope,1)


def lineMath(line,varPLines):
	result=None
	minimum=float('Inf')
	for index in varPLines:
		if isParallel(line,index):
			dist=parallelDist(line,index)
			if dist < minimum:
				minimum=dist
				result=index
	return result

def computeOdom(lines,varPlines):

	t,r=0.0,0.0

	translation=[]
	rotation=[]

	for index in lines:
		result=lineMath(index,varPlines)
		if result != None:
			translation.append(parallelDist(index,result))
			rotation.append(angleBetweenLines(index,result))
	
	if len(translation)!=0 and len(rotation) != 0:
		t=sum(translation)/len(translation)
		r=sum(rotation)/len(rotation)

	return t,r

def laser_callback(scan):
	marker_array = MarkerArray()
	var_lines=[]
	# Convert the laserscan to coordinates
	angle = scan.angle_min
	points = []    
	for r in scan.ranges:
		theta = angle
		angle += scan.angle_increment
		if (r > scan.range_max) or (r < scan.range_min):
			continue
		if (math.isnan(r)):
			continue

		points.append([r * math.sin(theta), r * math.cos(theta)]) 
	
	## convert points to pcl type
	points = np.array(points, dtype=np.float32)
	pcl_points = np.concatenate((points, np.zeros((len(points), 1))), axis=1)    
	p = pcl.PointCloud(np.array(pcl_points, dtype=np.float32))
	
	if p.size > MIN_Inliers:
		#fit the first
		indices, model = fitLine(p)
		
		# print "Found", len(indices), "inliers", model
		marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), 0))
		
		var_lines.append(((model[1], model[0]), (model[4], model[3])))

		pub.publish(marker_array)

		i=1
		pcl_points_next=removeInlinear(pcl_points,p.extract(indices))
		
		while len(pcl_points_next)/3 > MIN_Inliers:
		#fit next line
			p = pcl.PointCloud(np.array(pcl_points_next, dtype=np.float32))
			indices, model = fitLine(p)
			# print "Found", len(indices), "inliers", model

			marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), i))
			
			var_lines.append(((model[1], model[0]), (model[4], model[3])))

			pub.publish(marker_array)

			pcl_points_next=removeInlinear(pcl_points_next,p.extract(indices))

			i+=1

	global var_pLines
	
	if len(var_pLines)!=0:
		t,r=computeOdom(var_lines,var_pLines)
		print str(t)+"----------"+str(r*(180/math.pi))

	var_pLines=var_lines

			
def main():
	rospy.init_node('adventure_slam', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, laser_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
