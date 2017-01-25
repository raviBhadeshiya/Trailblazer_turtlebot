import math as math 
import numpy as np

def forward(p, a, rd):
  (x,y,theta) = p
  (vl,vr,t) = a
  (axle_length, wheel_radius, max_speed) = rd
  uW=(vl+vr)/2
  uV=vr-vl
  ti=0.00001
  td=0
  while (td<t):
  #theta_dot = (wheel_radius/axle_length) * uV
  #Theta changing over time, integrating over course of time ; theta will be average of final and initial.
    x_dot= wheel_radius * uW * math.cos(theta)
  #x_dot= wheel_radius * uW * math.cos((2*theta+theta_dot)/2) #math.cos(theta)
    y_dot = wheel_radius * uW * math.sin(theta)
  #y_dot = wheel_radius * uW * math.sin((2*theta+theta_dot)/2) #math.sin(theta)
    theta_dot = (wheel_radius/axle_length) * uV 
    x+=x_dot * ti
    y+=y_dot * ti
    theta += theta_dot * ti
    td+=ti

  return (x, y, theta)

def angle_diff(a, b):
  d = a - b
  return (d + math.pi) % (2 * math.pi) - math.pi

def matches(p0, p1):
  (x0,y0,theta0) = p0
  (x1,y1,theta1) = p1
  if math.hypot(x0 - x1, y0 - y1) > .05:
    return False
  return abs( angle_diff(theta0, theta1)) < .05


def inverse(p0, p1, rd):
  (x0,y0,theta0) = p0
  (x1,y1,theta1) = p1
  (axle_length, wheel_radius, max_speed) = rd
  action = []
  #recursive base case
  if(matches(p0,p1)): return action

  #Distance between start and stop point
  dist=0.52*math.hypot(x0-x1,y0-y1)

  #Target location angel condition
  if(x0!=x1 or y0 != y1):
    if(x1-x0 == 0):
      theta=math.pi/2 - theta0 #divison zero error
    else:
      theta = math.atan((y1-y0)/(x1-x0))-theta0
  else:
    theta=0.5*(theta1-theta0) #Rotation on base 

  #invese Calculation
  #Assign max speed to wheel and calculate time, with that time calculate other wheel speed
  if (theta<=0):
    left_velocity=max_speed
    time=(2*dist-theta)/(left_velocity*wheel_radius)
    right_velocity=(2*dist+theta)/(time*wheel_radius)
  else:
    right_velocity=max_speed
    time=(2*dist+theta)/(right_velocity*wheel_radius)
    left_velocity=(2*dist-theta)/(time*wheel_radius)
  time = time * axle_length
  #cordination correction
  if(x1<0):
    left_velocity,right_velocity=-right_velocity,-left_velocity
  
  #Minor time correction
  limit=0.5
  for time_i in np.arange(time-limit,time+limit,0.1):
    pt=forward(p0,(left_velocity,right_velocity,time_i),rd)
    print "Time:",time_i," pose:",pt
    if( matches(pt,p1)):
      action.append((left_velocity,right_velocity,time_i))
      print "Done......"
      return action #return solution got it
  
  #if solution not get find it by recursivly...
  act=(left_velocity,right_velocity,time)
  action.append(act)
  pt=forward(p0,act,rd)
  if(not matches(pt,p1)):
    res=inverse(pt,p1,rd)
    for i in res:
      action.append(i)  
  return action