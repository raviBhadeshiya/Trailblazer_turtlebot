#!/usr/bin/python

from project_2.kinematics import *
import argparse

parser = argparse.ArgumentParser(description='Inverse Kinematics Tester')
parser.add_argument('-x0',      default=0.0, type=float)
parser.add_argument('-y0',      default=0.0, type=float)
parser.add_argument('-theta0',  default=0.0, type=float)
parser.add_argument('-x1',      default=0.0, type=float)
parser.add_argument('-y1',      default=0.0, type=float)
parser.add_argument('-theta1',  default=0.0, type=float)
parser.add_argument('-axle',    default=1.0, type=float)
parser.add_argument('-wheelr',  default=1.0, type=float)
parser.add_argument('-speed',   default=1.0, type=float)

args = parser.parse_args()

pose0 = args.x0, args.y0, args.theta0
pose1 = args.x1, args.y1, args.theta1
robot = args.axle, args.wheelr, args.speed

plan = inverse(pose0, pose1, robot)
print "Plan: "
print plan
print 

p = pose0

for a in plan:
    print "Pose: (%3.2f %3.2f %3.2f)"%p, "Action: (%3.2f %3.2f %3.2f)"%a
    p = forward(p, a, robot)
print "Final pose: (%3.2f %3.2f %3.2f)"%p
print "Goal pose : (%3.2f %3.2f %3.2f)"%pose1

