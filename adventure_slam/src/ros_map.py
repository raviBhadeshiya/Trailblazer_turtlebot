#!/usr/bin/python

from map_maker import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import rosbag
import rospy
import tf

rospy.init_node('map_maker', anonymous=True)

tfListener = tf.TransformListener()
while True:
	try:
		# (position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
		(position, orientation) = tfListener.lookupTransform("/odom_visual", "/base_footprint", rospy.Time(0))
		break
	except:
		pass

rospy.loginfo("TF connection sucessful. Mapping node is running!")

def getPos():
	# (position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
	(position, orientation) = tfListener.lookupTransform("/odom_visual", "/base_footprint", rospy.Time(0))
	orientation = tf.transformations.euler_from_quaternion(orientation)
	(x, y, t) = position
	(aX, aY, theta) = orientation
	return x,y,theta

size=rospy.get_param('~size')
origin_x= rospy.get_param('~origin_x')
origin_y= rospy.get_param('~origin_y')
resolution= rospy.get_param('~resolution')

m = MapMaker(origin_x,origin_y,resolution,size,size)

pub = rospy.Publisher('/map', OccupancyGrid,queue_size=40)

def mapCallback(msg):
	m.pose=getPos()
	m.process_scan(msg)
	pub.publish(m.grid)

def main():
	rospy.Subscriber("/scan", LaserScan, mapCallback)
	rospy.loginfo("Node Running.")
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		
		pass
