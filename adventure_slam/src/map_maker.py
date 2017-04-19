from geometry import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from math import sin, cos, degrees
import numpy as np
import rospy
# ------------------------------------------------------------------------------
# MapMaker class
# ------------------------------------------------------------------------------
class MapMaker:
	def __init__(self, origin_x, origin_y, resolution, size_x, size_y):
		self.origin_x = origin_x
		self.origin_y = origin_y
		self.resolution = resolution
		self.size_x = size_x
		self.size_y = size_y
		self.grid = OccupancyGrid()
		self.grid.header.frame_id = 'odom'
		self.grid.info.resolution = resolution
		self.grid.info.width = size_x
		self.grid.info.height = size_y
		self.grid.info.origin.position.x = origin_x
		self.grid.info.origin.position.y = origin_y
		self.grid.info.origin.orientation.w = 1.0
		# self.grid.info.origin.orientation.x = -1.0
		self.grid.data = [-1] * (size_x * size_y)
		self.numScansReceived = 0
		self.mapData=[0.0] * (size_x * size_y)
		self.pose=(0,0,0)

	# ----------------------------------------------------------------------------
	# Convert from world coordinates to grid coordinates. This is convenience 
	# wrapper around the to_grid function from the first part of the assignment.
	# Usage:
	#   (x_grid, y_grid) = self.to_grid(x_world, y_world)
	def to_grid(self, x, y):
		return to_grid(x, y, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

	# ----------------------------------------------------------------------------
	# Convert from grid coordinates to world coordinates. This is convenience 
	# wrapper around the to_world function from the first part of the assignment.
	# Usage:
	#   (x_world, y_world) = self.to_world(x_grid, y_grid)
	def to_world(self, gx, gy):
		return to_world(gx, gy, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

	# ----------------------------------------------------------------------------
	# Process odometry message. You code should go here.
	def process_odom(self, msg):
		self.curr_time = msg.header.stamp
		pose=msg.pose.pose
		self.pose = pose.position.x, pose.position.y, euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]

	# ----------------------------------------------------------------------------
	# Process laserscan message. You code should go here.
	def process_scan(self, msg):
		self.range=msg.ranges
		rays=list()
		free_cells=list()
		occupied_cells=list()
		dist=iter(msg.ranges)
		for angle in np.arange(msg.angle_max,msg.angle_min,-msg.angle_increment):
			d=dist.next()
			x=d*cos(self.pose[2]+angle)+self.pose[0]
			y=d*sin(self.pose[2]+angle)+self.pose[1]
			rays.append((x,y))
			if self.to_grid(x,y)!=None and self.to_grid(self.pose[0],self.pose[1]) != None:
				gx,gy=self.to_grid(x,y)
				px,py=self.to_grid(self.pose[0],self.pose[1])
				fPoints=bresenham(px,py,gx,gy)
				#free cell
				for i in fPoints:
					# free_cells.append(i)
					self.grid.data[to_index(i[0],i[1],self.size_x)]=0
					# self.mapData[to_index(i[0],i[1],self.size_x)]-=0.7 #log odd negative
				#occupied cell
				occupied_cells.append((gx,gy))
				# if d < msg.range_max:
				# 	pr=1.6*2
				# else:
				# 	pr=0.4*2
				self.grid.data[to_index(gx,gy,self.size_x)]=100
				# self.mapData[to_index(gx,gy,self.size_x)]+=pr #log odd positive

		# for idx, val in enumerate(self.mapData):
		# 	if val <= -2.1: #cutoff prob
		# 		self.grid.data[idx]=0
		# 	elif val >=1.0: #cutoff prob
		# 		self.grid.data[idx]=100
		self.numScansReceived+=1
		None


