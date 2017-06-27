#!/usr/bin/env python

import rospy
import argparse
from std_msgs.msg import String
import geometry_msgs.msg
def polygonOpen():

  #-----------------------------------------------------------------------------
  # Initialize node
  #-----------------------------------------------------------------------------

  rospy.init_node('open_polygon', anonymous=False)

  # This code is required to make sure this node gets simulation time correctly
  simulation = False
  if rospy.has_param('/use_sim_time'):
    if rospy.get_param("/use_sim_time") == True:
      simulation = True

  if simulation:
    rospy.loginfo("Using simulated time.")
    rospy.loginfo("Waiting for the first valid time measurement...")
    t = rospy.Time.now()
    while t == rospy.Time.from_sec(0):
      t = rospy.Time.now()
    rospy.loginfo("Done!")

  else:
    rospy.loginfo("Using real time.")

  #-----------------------------------------------------------------------------
  # Parse command line
  #-----------------------------------------------------------------------------

  parser = argparse.ArgumentParser(description='Polygon Drive Openloop Control')
  parser.add_argument('-d',      default=0.2, type=float)
  parser.add_argument('-n',      default=6, type=int)

  args = parser.parse_args()
  sideLength = args.d
  numSides = args.n

  rospy.loginfo("Polygon parameters:")
  rospy.loginfo("  number of sides: " + str (numSides))
  rospy.loginfo("  side length: " + str(sideLength))

  #-----------------------------------------------------------------------------
  # Drive (your code should go here)
  #-----------------------------------------------------------------------------
  cmdPublisher = rospy.Publisher('/cmd_vel_mux/input/navi',geometry_msgs.msg.Twist, queue_size=10);
  linerVelocity,angularVelocity=0.5,0.5;

  trasnslate_msg=geometry_msgs.msg.Twist();
  rotation_msg=geometry_msgs.msg.Twist();

  trasnslate_msg.linear.x=linerVelocity;
  trasnslate_time=sideLength/linerVelocity;
  rospy.loginfo('Traslate Time')
  rospy.loginfo(trasnslate_time)
  
  rotation_msg.angular.z=angularVelocity;
  rotation_time=(6.28/numSides)/angularVelocity
  rospy.loginfo('Rotation Time')
  rospy.loginfo(rotation_time)

  for i in range(numSides):
    #Translate
    r = rospy.Rate(10);
    t = rospy.Time.now();
    while rospy.Time.now() - t < rospy.Duration(trasnslate_time):
      cmdPublisher.publish( trasnslate_msg );
      r.sleep();
    #Rotate
    r = rospy.Rate(10);
    t = rospy.Time.now();
    while rospy.Time.now() - t < rospy.Duration(rotation_time):
      cmdPublisher.publish( rotation_msg );
      r.sleep();

#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  try:
    polygonOpen()
  except rospy.ROSInterruptException:
    pass