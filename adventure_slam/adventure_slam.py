#!/usr/bin/env python
import rospy, pcl_ros, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry

import cv2, math, pcl
import numpy as np

pub = rospy.Publisher('/slam_debug', MarkerArray)


def get_line(p1, v1, id_, color=(0,0,1)):
    marker = Marker()
    marker.header.frame_id = "camera_depth_frame"
    marker.header.stamp = rospy.Time()
    marker.lifetime = rospy.Duration(1)
    marker.ns = "slam_debug_ns"
    marker.id = id_
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0
    marker.scale.x = 0.01
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.points.append(Point(p1[0] + 100 * v1[0], p1[1] + 100 * v1[1], 0))
    marker.points.append(Point(p1[0] - 100 * v1[0], p1[1] - 100 * v1[1], 0))
    return marker


def laser_callback(scan):
    marker_array = MarkerArray()

    # Convert the laserscan to coordinates
    angle = scan.angle_min
    points = []    
    for r in scan.ranges:
        theta = angle
        angle += scan.angle_increment
        if (r > scan.range_max) or (r < scan.range_min):
            continue
        if (math.isnan(r)):
            continue

        points.append([r * math.sin(theta), r * math.cos(theta)]) 

    # Fit the line
    ## convert points to pcl type
    points = np.array(points, dtype=np.float32)
    pcl_points = np.concatenate((points, np.zeros((len(points), 1))), axis=1)    
    p = pcl.PointCloud(np.array(pcl_points, dtype=np.float32))
    
    ## create a segmenter object
    seg = p.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_LINE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold (0.003)
    
    ## apply RANSAC
    indices, model = seg.segment()
    print "Found", len(indices), "inliers", model
    
    # OpenCV line fitter - least squares
    # line = cv2.fitLine(points, 2, 0, 0.01, 0.01)
    # Publish detected lines so we can see them in Rviz
    # marker_array.markers.append(get_line((line[3], line[2]), (line[1], line[0]), 1, (0,1,0)))
    # pub.publish(marker_array)

    marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), 0))
    pub.publish(marker_array)
      

def main():
    rospy.init_node('adventure_slam', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
