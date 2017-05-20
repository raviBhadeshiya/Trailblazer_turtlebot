#!/usr/bin/env python

import rospy
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_inverse
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
import math
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
import threading
import copy
import rospkg
from trajectory_demo import *
from actionlib_msgs.msg import *
import os
from matplotlib import pyplot as plt

#-------------------------------------------------------------------------------
# Object search class
#-------------------------------------------------------------------------------
class ObjectSearch:
  def __init__(self):
		# Initialize node
	rospy.init_node('adventure_recognition')

	self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
	rospy.loginfo("Wait for the move base server..")
	self.move_base.wait_for_server(rospy.Duration(5))
	rospy.loginfo("Done..")
	# Navigation
	
	self.goalStatesText = ['PENDING',
					  'ACTIVE',
					  'PREEMPTED',
					  'SUCCEEDED',
					  'ABORTED',
					  'REJECTED',
					  'PREEMPTING',
					  'RECALLING',
					  'RECALLED',
					  'LOST']

	# Vision
	self.image = []
	self.processImage = False #Keep it true to store the  image
	self.lock = threading.Lock()

	rospack = rospkg.RosPack()
	self.debugImageDir = rospack.get_path('adventure_recognition') + "/images/debug/"
	self.trainImageDir = rospack.get_path('adventure_recognition') + "/images/train/"
	# self.trainImageNames = ['bottle_0.jpg', 'bottle_1.jpg', 'bottle_1.jpg', 'temp.jpg']
	self.trainImageNames = ['temp.jpg']

	#Flags
	self.moveFlag=False
	self.imageFlag=False
	self.armFlag=False
	self.index =0
	self.time=rospy.get_time()
	self.areaThreshold=1000
	self.dst=[]

	sift = cv2.xfeatures2d.SIFT_create()
	
	self.trainingImages = []


	for string in self.trainImageNames:
		img = cv2.imread(os.path.join(self.trainImageDir,string))
		if img is not None:
			self.trainingImages.append(img)


	# Image subscriber and cv_bridge
	# self.imageTopic = "/camera/rgb/image_raw/compressed"
	self.imageTopic = "/usb_cam/image_raw/compressed"
	self.imageSub = rospy.Subscriber(self.imageTopic,CompressedImage,self.imageCallback)    
	self.debugImageId  = 0
	self.velPub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	self.linearVel = Twist()
	self.angularVel = Twist()

	# Generate goal poses
	self.goalPoses = []


	target=[2.7, -1.6] 
	p=Pose(Point(target[0],target[1],0),Quaternion(0,0,0,1))
	self.goalPoses.append(p)

	target=[0.4,0.0] 
	p=Pose(Point(target[0],target[1],0),Quaternion(0,0,0,1))
	self.goalPoses.append(p)

	target=[0.7,-1.6] 
	p=Pose(Point(target[0],target[1],0),Quaternion(0,0,0,1))
	self.goalPoses.append(p)

  #-------------------------------------------------------------------------------
  # Draw matches between a training image and test image
  #  img1,img2 - RGB images
  #  kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
  #            detection algorithms
  #  matches - A list of matches of corresponding keypoints through any
  #            OpenCV keypoint matching algorithm
  def drawMatches(self, img1, kp1, img2, kp2, matches):

	# Create a new output image that concatenates the two images together
	# (a.k.a) a montage
	rows1 = img1.shape[0]
	cols1 = img1.shape[1]
	rows2 = img2.shape[0]
	cols2 = img2.shape[1]

	out = np.zeros((max([rows1,rows2]),cols1+cols2), dtype='uint8')

	# Place the first image to the left
	out[:rows1,:cols1] = img1

	# Place the next image to the right of it
	out[:rows2,cols1:] = img2

	# For each pair of points we have between both images
	# draw circles, then connect a line between them
	for mat in matches:

	   # Get the matching keypoints for each of the images
	   img1_idx = mat.queryIdx
	   img2_idx = mat.trainIdx

	   # x - columns
	   # y - rows
	   (x1,y1) = kp1[img1_idx].pt
	   (x2,y2) = kp2[img2_idx].pt

	   # Draw a small circle at both co-ordinates
	   # radius 4
	   # colour blue
	   # thickness = 1
	   cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
	   cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

	   # Draw a line in between the two points
	   # thickness = 1
	   # colour blue
	   cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)

	# Also return the image if you'd like a copy
	return out

  #-----------------------------------------------------------------------------
  # Image callback
  def imageCallback(self, data):

	np_arr = np.fromstring(data.data, np.uint8)

	self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV 3.0

	if not self.moveFlag and not self.imageFlag and not self.armFlag:
		self.moveFlag=True
		self.robotMoveToGoal(self.goalPoses[self.index])

	if self.moveFlag and not self.imageFlag and not self.armFlag:
		if rospy.get_time() - self.time > 2:
			if self.computeSiftFeatures(self.trainingImages[0], self.cv_image):
				self.imageFlag = True

				h,w=self.dst.item(3)-self.dst.item(1), self.dst.item(4)-self.dst.item(2)
				x,y = int(self.dst.item(0)+(w/2)),int(self.dst.item(1)+h/2)
				area = h*w
				
				temp1,sizeY,temp2=self.cv_image.shape

				if abs(y-sizeY/2) < 10:
					t=y-sizeY/abs(t-sizeY)
					self.angularVel.angular.z=0.1*t
					self.velPub.publish(self.angularVel)

				elif area < 1500:
					self.linearVel.linear.x=0.1
					self.velPub.publish(self.linearVel)

			cv2.imshow("Image live feed", self.cv_image)
			cv2.waitKey(1)
			self.time=rospy.get_time()

	if self.moveFlag and  self.imageFlag and not self.armFlag:
		cv2.destroyAllWindows()
		cv2.waitKey(1)

		traj=phantomXArm() #Target location for phantomX arm
		traj.setGoal(goal=[-0.5,-0.5,-0.5,-0.5,-0.5])
		traj.armMoveToGoal()
		rospy.sleep(1)
		traj.setGoal()
		traj.armMoveToGoal()
		rospy.sleep(1)
		print "ARm out"
		self.imageFlag= False
		self.moveFlag=False

		if self.index < 2:
			self.index +=1
		else:
			self.index=0
			self.armFlag = True

  #-------------------------------------------------------------------------------
  # Capture image, display it and save it to the debug folder
  def capture_image(self):

	self.processImage = True
	while (self.processImage):
		rospy.sleep(0.01)

  def computeSiftFeatures(self, img1, _img2):
	
	MIN_MATCH_COUNT = 10

	img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
	img2 = cv2.cvtColor(_img2, cv2.COLOR_BGR2GRAY)

	sift = cv2.xfeatures2d.SIFT_create()

	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	good = []
	for m,n in matches:
	    if m.distance < 0.7*n.distance:
		   good.append(m)

	if len(good)>MIN_MATCH_COUNT:
	    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

	    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
	    if mask != None:
		    matchesMask = mask.ravel().tolist()

		    h,w = img1.shape
		    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
		    self.dst = cv2.perspectiveTransform(pts,M)

		    cv2.polylines(self.cv_image,[np.int32(self.dst)],True,255,3, cv2.LINE_AA)
	else:
	    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
	    matchesMask = None
	    self.cv_image=_img2
	    return False

	return True


  def robotMoveToGoal(self,waypoint):

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp=rospy.Time.now()
	goal.target_pose.pose = waypoint

	self.move_base.send_goal(goal)
	timeOut = self.move_base.wait_for_result(rospy.Duration(60))
	state = self.move_base.get_state()

	if timeOut and state == GoalStatus.SUCCEEDED:
		return True
	else:
		self.move_base.cancel_goal()
		return False

  def run(self):
		while True:
			rospy.sleep(0.1)

#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

  objectSearch = ObjectSearch()
  objectSearch.run()
