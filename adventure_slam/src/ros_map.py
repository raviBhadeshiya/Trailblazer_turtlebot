#!/usr/bin/python

from map_maker import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import rosbag
import rospy
import tf

rospy.init_node('map_maker', anonymous=True)
# mapBag=rosbag.Bag('test.bag','w')

tfListener = tf.TransformListener()
while True:
	try:
		(position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
		break
	except:
		rospy.loginfo("Connecting to TF...")

rospy.loginfo("TF connection sucessful.")

def getPos():
	(position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
	orientation = tf.transformations.euler_from_quaternion(orientation)
	(x, y, t) = position
	(aX, aY, theta) = orientation
	# print x,y,theta
	return x,y,theta

# size=100
# origin_x,origin_y,_=getPos()
# origin_x,origin_y=-50,-50
# resolution= 0.5

size=1000
# origin_x,origin_y,_=getPos()
origin_x= -50
origin_y= -50
resolution= 0.1

m = MapMaker(origin_x,origin_y,resolution,size,size)

pub = rospy.Publisher('/map', OccupancyGrid,queue_size=40)

def mapCallback(msg):
	m.pose=getPos()
	# mapBag.write('Odometry', float(m.pose))
	# mapBag.write('LaserScan', msg.ranges)
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
		# mapBag.close()
		pass
