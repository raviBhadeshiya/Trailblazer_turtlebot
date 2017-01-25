#!/usr/bin/python

from project_2.kinematics import *
import argparse

parser = argparse.ArgumentParser(description='Forward Kinematics Tester')
parser.add_argument('-x',      default=0.0, type=float)
parser.add_argument('-y',      default=0.0, type=float)
parser.add_argument('-theta',  default=0.0, type=float)
parser.add_argument('-vleft',  default=0.0, type=float)
parser.add_argument('-vright', default=0.0, type=float)
parser.add_argument('-time',   default=1.0, type=float)
parser.add_argument('-axle',   default=1.0, type=float)
parser.add_argument('-wheelr', default=1.0, type=float)

args = parser.parse_args()

pose = args.x, args.y, args.theta
action = args.vleft, args.vright, args.time
robot = args.axle, args.wheelr, 0.0 #speed limit not used

print forward( pose, action, robot )