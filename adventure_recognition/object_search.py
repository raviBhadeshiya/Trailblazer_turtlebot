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

	# Initialize node
	rospy.init_node('adventure_recognition')
	# rospy.init_node('trajectory_demo')


	sift = cv2.xfeatures2d.SIFT_create()
	
	self.trainingImages = []
	# self.kp = []
	# self.des = []

	for string in self.trainImageNames:
		img = cv2.imread(os.path.join(self.trainImageDir,string))
		if img is not None:
			self.trainingImages.append(img)


	# Image subscriber and cv_bridge
	# self.imageTopic = "/camera/rgb/image_raw/compressed"
	self.imageTopic = "/usb_cam/image_raw/compressed"
	self.imageSub = rospy.Subscriber(self.imageTopic,CompressedImage,self.imageCallback)    
	self.debugImageId  = 0

	# Generate goal poses
	self.goalPoses = []

	#Subcribe to move_base action server
	self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
	rospy.loginfo("Wait for the move base server..")
	self.move_base.wait_for_server(rospy.Duration(5))
	rospy.loginfo("Done..")

			# kp, des = sift.detectAndCompute(img,None)
			# self.kp.append(kp), self.des.append(des)
			# i+=1

	# self.flannIndexKDtree = 0
	# self.index_params = dict(algorithm = self.flannIndexKDtree, trees = 5)
	# self.search_params = dict(checks = 50)

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

  	

	# Capture image
	np_arr = np.fromstring(data.data, np.uint8)

	self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV 3.0
	# cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # OpenCV 2.4


	# Store it if required
	self.lock.acquire()

	if self.processImage:
		self.image = copy.deepcopy(self.cv_image)
	 	self.processImage = False


	# for img in self.trainingImages:
	# 	# print "inside for loop"
	# 	if self.computeSiftFeatures(img, self.cv_image):
	# 		print "inside for loop"
	# 		break

	# Show image
	# cv2.imshow("Image live feed", self.cv_image)
	cv2.waitKey(0)

	self.lock.release()
	# print "Call back out!"

  #-------------------------------------------------------------------------------
  # Capture image, display it and save it to the debug folder
  def capture_image(self):

	# First get the image to be processed
	self.processImage = True
	while (self.processImage):
		print "Checking.."
		rospy.sleep(0.01)

	#Display image
	# self.lock.acquire()
	# cv2.imshow("Captured image", self.image)
	# self.lock.release()

	# # Save image
	# cv2.imwrite(self.debugImageDir+"image_" + str(self.debugImageId) + ".png", self.image)
	# self.debugImageId += 1


  def computeSiftFeatures(self, img1, img2):
  	MIN_MATCH_COUNT = 10
	# img1 = cv2.imread('images/train/temp.jpg',0)          # queryImage
	# img2 = cv2.imread('images/train/cpp.jpg',0) # trainImage

	# Initiate SIFT detector
	# sift = cv2.SIFT()

	#img1=training image
	#img2=query image

	img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
	img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

	sift = cv2.xfeatures2d.SIFT_create()

	# find the keypoints and descriptors with SIFT
	kp1, des1 = sift.detectAndCompute(img1,None)
	kp2, des2 = sift.detectAndCompute(img2,None)

	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)

	flann = cv2.FlannBasedMatcher(index_params, search_params)

	matches = flann.knnMatch(des1,des2,k=2)

	# store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
	    if m.distance < 0.7*n.distance:
	        good.append(m)

	if len(good)>MIN_MATCH_COUNT:
	    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

	    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
	    matchesMask = mask.ravel().tolist()

	    h,w = img1.shape
	    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
	    dst = cv2.perspectiveTransform(pts,M)

	    self.cv_image=cv2.polylines(self.cv_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
	    # cv2.imshow("img2",img2)
	else:
	    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
	    matchesMask = None
	    self.cv_image=img2
	    return False

	# if matchesMask != None:    
	# 	draw_params = dict(matchColor = (0,255,0), # draw matches in green color
	# 	                   singlePointColor = None,
	# 	                   matchesMask = matchesMask, # draw only inliers
	# 	                   flags = 2)

		# self.cv_image = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
		# self.drawMatches(img1, kp1, img2, kp2, good)


		# plt.imshow(img3, 'gray'),plt.show()
	return True


  def robotMoveToGoal(self,waypoint):

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp=rospy.Time.now()
	goal.target_pose.pose = waypoint

	self.move_base.send_goal(goal)
	timeOut = self.move_base.wait_for_result(rospy.Duration(60))
	state = self.move_base.get_state()
	# print state

	if timeOut and state == GoalStatus.SUCCEEDED:
		return True
	else:
		self.move_base.cancel_goal()
		return False

  def run(self):
		target=[2.7, -1.6] #Target position for robot to move
		traj=phantomXArm() #Target location for phantomX arm
		p=Pose(Point(target[0],target[1],0),Quaternion(0,0,0,1))
		traj.setGoal(goal=[0.5,0.5,0.5,0.5,0.5])
		self.robotMoveToGoal(p)
		traj.armMoveToGoal()
		
		while True:
			self.capture_image()
			
			rospy.sleep(0.1)

			for img in self.trainingImages:
				if self.computeSiftFeatures(img, self.cv_image):
					print "inside for loop"
					break	
			cv2.imshow("Image live feed", self.cv_image)
			cv2.waitKey(3)
	# while True:
	# 	rospy.sleep(0.1)

#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------
if __name__ == '__main__':

  objectSearch = ObjectSearch()
  objectSearch.run()
