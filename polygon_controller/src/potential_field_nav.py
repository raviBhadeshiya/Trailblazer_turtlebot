#!/usr/bin/env python

import numpy as np               # Linear algebra
import matplotlib.pyplot as plt  # Plotting
from math import hypot,atan2,cos,sin
#-------------------------------------------------------------------------------
def GoToGoal(x_g, y_g):
  #-----------------------------------------------------------------------------
  # Initialization (you don't have to modify anything here)
  #-----------------------------------------------------------------------------
  # Define a map
  nX = 100
  nY = 100
  X, Y = np.meshgrid(np.linspace(0,nX,nX), np.linspace(0,nY,nY))#num=100...no need to map index!
  # Define start position
  x_0 = 20
  y_0 = 20
  #-----------------------------------------------------------------------------
  # Calculate potential field for each cell of the map
  #-----------------------------------------------------------------------------
  def attractiveMatrix(u,v,x,y,rho=np.inf):
    u=u-x
    v=v-y   
    #compute dist form goal and create mask 
    dist = (np.square(u)+np.square(v))
    mask=dist < rho
    mask.astype(float)
    #goal =0 and increasing form awy to goal
    return  np.multiply(mask, dist)

  def repulsiveMatrix(u,v,x,y,rho):
    r=5#radius   #square to save computation 
    force=np.zeros((len(u),len(v)))
    # for xx in np.arange(len(u)):
    #   for yy in np.arange(len(v)):
    for xx in np.arange(x-r-rho-1,x+r+rho+1):
      for yy in np.arange(y-r-rho-1,y+r+rho+1):
        distTo=hypot(xx-x,yy-y)
        if (distTo<r) and (distTo<r):
          force[xx][yy]=1000  #assign high value for obstacle
        elif (distTo >= r) and (distTo <r+rho):
          force[xx][yy]=1000*(1/distTo - 1/rho)**2
    #mask dist matrix just inv the value thus obstacle will be at high 
    # dist=dist.max()-dist
    return force

  force=10.*attractiveMatrix(X,Y,x_g,y_g);

  obstacle=[[50, 20],[80, 35]]

  for i in obstacle:
    rho=7;
    force=np.add(force,50.*repulsiveMatrix(X,Y,i[1],i[0],rho));
  ##force gradient map
  U,V=np.gradient(-force);
  #-----------------------------------------------------------------------------
  # Find the robot path using the potential field
  #-----------------------------------------------------------------------------
  def pathPlanning(start,goal,gy,gx):#gradient decent approch
      path=[]
      i=1
      while i < 1000:#max Iteration
          dy=gy[round(start[0]),round(start[1])];
          dx=gx[round(start[0]),round(start[1])];
          
          #unit force vector
          dx=dx/hypot(dx,dy);
          dy=dy/hypot(dx,dy);
          
          start[0]+=0.5*dx
          start[1]+=0.5*dy
          
          path.append((start[1],start[0]))         
          i=i+1;
          if hypot(start[0]-goal[0],start[1]-goal[1]) < 2.0:
              break;
      # print "Found path in ",i," steps"
      return path

  path=[]
  path=pathPlanning([x_0,y_0],[x_g,y_g],V,U)
  #-----------------------------------------------------------------------------
  # Plot results (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Plot potential field. Every nth cell will be plotted
  nth = 1
  fig=plt.figure()
  Q = plt.quiver(X[::nth, ::nth], Y[::nth, ::nth], V[::nth, ::nth], U[::nth, ::nth],
        pivot='mid', units='width')
  plt.axis([-5, 105, -5, 105])
  plt.title('Robot path')
  plt.xlabel('Y')
  plt.xlabel('X')

  # Plot Path
  path_x = []
  path_y = []
  for i in range(len(path)-1):
    path_x.append(path[i][0])
    path_y.append(path[i][1])
  plt.plot(path_x, path_y, 'r', linewidth=4)

  # Plot Start and goal positions
  plt.plot([x_0], [y_0], 'bo', markersize=10)
  plt.plot([x_g], [y_g], 'go', markersize=10)
  ob1=plt.Circle((50, 20), 5, color='#e9ff00', alpha=0.8)
  ob2=plt.Circle((80, 35), 5, color='#e9ff00', alpha=0.8)
  ax = fig.add_subplot(111)
  ax.add_artist(ob1)
  ax.add_artist(ob2)

  # Show plot
  print "Close fig for next trejectory"
  plt.show()

#-------------------------------------------------------------------------------
if __name__ == '__main__':
  x_g = 94
  y_g = 33
  print "PathPlanning -- goalX=94 and goalY=33"
  GoToGoal(x_g, y_g)
  x_g = 94
  y_g = 40
  print "PathPlanning -- goalX=94 and goalY=40"
  GoToGoal(x_g, y_g)