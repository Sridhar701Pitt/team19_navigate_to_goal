#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from visualization_msgs.msg import Marker 

###################################
## VARIABLE DECLARATION AND SETUP
###################################

max_dist_threshold = 1.0            # Segment out obstacles that are detected beyond 2 m as they create unnecessary computations for the planner

angles_array = np.arange(0,360)                 # Create the array with angles at each index for conversion from polar to cartesian (obstacle vectors)
angles_array = angles_array * np.pi / 180.0                    # Convert values to radian
cos_angles_array = np.cos(angles_array)         # Required to get x components of the obstacle vectors
sin_angles_array = np.sin(angles_array)         # Required to get y components of the obstacle vectors

###################################
## Function Declaration
###################################
def obstacle_arrow_data(obstacle_vec_x, osbtacle_vec_y):

    obstacle_marker = Marker()
    obstacle_marker.action = Marker.ADD
    obstacle_marker.header.frame_id = '/base_scan'
    obstacle_marker.header.stamp = rospy.Time.now()
    obstacle_marker.ns = 'points_arrows'
    obstacle_marker.id = 11311
    obstacle_marker.type = Marker.ARROW
    obstacle_marker.pose.orientation.y = 0
    obstacle_marker.pose.orientation.w = 1
    obstacle_marker.scale = Vector3(0.1, 0.1, 0.1)
    obstacle_marker.color.r = 0.2
    obstacle_marker.color.g = 0.5
    obstacle_marker.color.b = 1.0
    obstacle_marker.color.a = 0.3

    obstacle_marker.points = [ Point(0, 0, 0), Point(obstacle_vec_x, osbtacle_vec_y, 0) ]
    return obstacle_marker


def compute_object_arrow(laser_scan_object):
    scan_object_ranges = np.asarray(laser_scan_object.ranges)
    scan_dist_threshold = np.where(scan_object_ranges > max_dist_threshold, 0, scan_object_ranges)
    
    obstacle_vec_x = -1 * np.dot(scan_dist_threshold, cos_angles_array)
    obstacle_vec_y = -1 * np.dot(scan_dist_threshold, sin_angles_array)

    print("obstacle_x : ", obstacle_vec_x, "     obstacle y : ", obstacle_vec_y)
    
    # obstacle_vec_theta = obstacle_vec_y / obstacle_vec_x
    
    obstacle_arrow = obstacle_arrow_data(obstacle_vec_x, obstacle_vec_y)

    #obstacle_vector_tip = Point(obstacle_vec_x, obstacle_vec_y, 0)
    #print(obstacle_vector_tip)
    #pub.publish(obstacle_vector_tip)
    
    print(obstacle_arrow)
    pub.publish(obstacle_arrow)



def Init():
	global pub

	rospy.init_node('get_object_range', anonymous=True)
	
	# Subscribe to LIDAR scan topic /scan
	rospy.Subscriber("/scan", LaserScan, compute_object_arrow, queue_size=1)

	# Publish angle and distance
	pub = rospy.Publisher("/obstacle_vector", Marker, queue_size=10)

	rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass