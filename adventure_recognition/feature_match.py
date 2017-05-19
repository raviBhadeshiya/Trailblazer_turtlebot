import numpy as np
import cv2
from matplotlib import pyplot as plt

MIN_MATCH_COUNT = 10

img1 = cv2.imread('images/train/temp.jpg',0)          # queryImage
img2 = cv2.imread('images/train/cpp.jpg',0) # trainImage

# Initiate SIFT detector
# sift = cv2.SIFT()

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

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
    matchesMask = mask.ravel().tolist()
    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    cv2.imshow("img2",img2)
    # cv2.waitkey(1)

else:
    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

plt.imshow(img3, 'gray'),plt.show()



 #  def computeSiftFeatures(self):

 #  	self.capture_image()
  	
 #  	print "Captured!"
 #  	sift = cv2.xfeatures2d.SIFT_create()
 #  	self.image=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
 #  	kp, des = sift.detectAndCompute(self.image,None)
 #  	flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

 #  	for i in range(len(self.trainingImages)):

 #  		matches = flann.knnMatch(self.des[i],des,k=2)
	# 	good = []
	# 	for m,n in matches:
	# 	    if m.distance < 0.7*n.distance:
	# 	        good.append(m)

	# 	if len(good)>self.minMatchCount:
	# 	    print i
	# 	    dst_pts = np.float32([ kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
	# 	    src_pts = np.float32([ self.kp[i][m.trainIdx].pt for m in good ]).reshape(-1,1,2)

	# 	    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
	# 	    matchesMask = mask.ravel().tolist()

	# 	    h,w = self.image.shape

	# 	    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
	# 	    dst = cv2.perspectiveTransform(pts,M)

	# 	    img2 = cv2.polylines(self.image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
	# 	    print "Matched!! - %d/%d" % (len(good),self.minMatchCount)
	# 	    break
	# 	else:
	# 	    print "Not enough matches are found - %d/%d" % (len(good),self.minMatchCount)
	# 	    matchesMask = None

	# if matchesMask != None:
	# 	draw_params = dict(matchColor = (0,255,0), 
	# 		singlePointColor = None,
	#        matchesMask = matchesMask, flags = 2)

	# 	# img3 = cv2.drawMatches(self.image,kp,img2,self.kp[i],good,None,**draw_params)
	# 	img3 = cv2.drawMatches(self.trainingImages[i],kp,img2,self.kp[i],good,None,**draw_params)
	# 	plt.imshow(img3, 'gray'),plt.show()


  #-----------------------------------------------------------------------------
  # Run!