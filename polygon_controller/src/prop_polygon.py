#!/usr/bin/env python
import rospy
import argparse
from std_msgs.msg import String
import geometry_msgs.msg
import tf
import math as math
import numpy as np
#Param
linerMaxVelocity = 0.5
angularMaxVelocity = 0.5

# Gazebo
K_RHO = 0.8
K_ALPHA = 1.6
K_BETA = -0.075

#Turtlebot
# K_RHO = 0.55
# K_ALPHA = 0.7
# K_BETA = -0.072

def polygonProp():

  #-----------------------------------------------------------------------------
  # Initialize node
  #-----------------------------------------------------------------------------

  rospy.init_node('prop_polygon', anonymous=False)

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

  parser = argparse.ArgumentParser(description='Polygon Drive Proportional Control')
  parser.add_argument('-d',      default=2, type=float)
  parser.add_argument('-n',      default=6, type=int)

  args = parser.parse_args()
  sideLength = args.d
  numSides = args.n

  rospy.loginfo("Polygon parameters:")
  rospy.loginfo("  number of sides: " + str (numSides))
  rospy.loginfo("  side length: " + str(sideLength))

  #-----------------------------------------------------------------------------
  # Drive 
  #-----------------------------------------------------------------------------

  # Initialize transform listener
  tfListener = tf.TransformListener()
  def getPos():
    while True:
      try:
        (position, orientation) = tfListener.lookupTransform(
            "/odom", "/base_footprint", rospy.Time(0))
        orientation = tf.transformations.euler_from_quaternion(orientation)
        #rospy.loginfo('Tf is working..');
        (x, y, t) = position
        (aX, aY, theta) = orientation
        break
      except:
        pass
        #rospy.loginfo('tf is not working..');
    return [x,y,theta]

  print "Starting postion.."
  print getPos()

  cmdPublisher = rospy.Publisher('/cmd_vel_mux/input/navi', geometry_msgs.msg.Twist, queue_size=10)

  msg = geometry_msgs.msg.Twist()

  def sendMsg(cmd_msg, time):
    r = rospy.Rate(10)
    t = rospy.Time.now()
    while rospy.Time.now() - t < rospy.Duration(time):
      cmdPublisher.publish(cmd_msg)  # check this
      r.sleep()

  def waypoints(numSides,sideLength):
    [x,y,theta]= getPos()
    polygonAngle=(6.28/numSides)
    pos=[x,y,theta]
    poly_wp = []
    for i in range(numSides):
      x_pose = pos[0] + sideLength*math.cos(pos[2])
      y_pose = pos[1] + sideLength*math.sin(pos[2])
      if pos[2] > 3.14:
        pos[2] = pos[2] - 6.28
      poly_wp.append([x_pose, y_pose, pos[2]])
      pos[2] = pos[2] + polygonAngle
      pos = [x_pose, y_pose, pos[2]]
    return poly_wp

  wp = waypoints(numSides,sideLength)

  def controller(xT,yT,x,y,theta):
    rho = math.sqrt( math.pow((xT-x),2) + math.pow((yT-y),2) )
    theta_direction=math.atan2((yT-y),(xT-x))
    alpha = theta_direction - theta
    beta = - theta_direction
    return rho,alpha,beta

  def force(rho,alpha,beta):
      linear = K_RHO * rho
      alpha = ((alpha + math.pi) %(2*math.pi)) - math.pi
      beta = ((beta + math.pi)%(2 * math.pi)) - math.pi
      angular = (K_ALPHA*alpha) + (K_BETA*beta)
      return linear ,angular

  for i in range(0,numSides):
    (xT, yT, thetaT) = wp[i]
    xT=round(xT)
    yT=round(yT)
    if thetaT > math.pi:
      thetaT = thetaT - 2*math.pi
    
    while True:
      (x, y, theta) = getPos()
      rho,alpha,beta=controller(xT,yT,x,y,theta)
      if (abs(((thetaT - theta + math.pi) % (2 * math.pi)) - math.pi) < 0.5) and abs(rho) < 0.5:
        break
      msg.linear.x,msg.angular.z = force(rho,alpha,beta)
      
      msg.linear.x = msg.linear.x if msg.linear.x < linerMaxVelocity else linerMaxVelocity
      if msg.angular.z >= 0:
        msg.angular.z = msg.angular.z if msg.angular.z < angularMaxVelocity else angularMaxVelocity
      else:
        msg.angular.z = msg.angular.z if msg.angular.z > -angularMaxVelocity else -angularMaxVelocity

      sendMsg(msg,0.01)


    rospy.loginfo('Waypoint '+str(i)+" Done")
    print getPos()


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  try:
    polygonProp()
  except rospy.ROSInterruptException:
    pass