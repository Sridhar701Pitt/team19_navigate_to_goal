#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from visualization_msgs.msg import Marker 

###################################
## VARIABLE DECLARATION AND SETUP
###################################

max_dist_threshold = 1.0                        # Segment out obstacles that are detected beyond 2 m as they create unnecessary computations for the planner

angles_array = np.arange(0,360)                 # Create the array with angles at each index for conversion from polar to cartesian (obstacle vectors)
angles_array = angles_array * np.pi / 180.0                    # Convert values to radian
cos_angles_array = np.cos(angles_array)         # Required to get x components of the obstacle vectors
sin_angles_array = np.sin(angles_array)         # Required to get y components of the obstacle vectors

l = 0.15
theta = 0

###################################
## Function Declaration
###################################
def calc_optimal_vel(obstacle_vector):

    global current_obstacle_vector

    current_obstacle_vector = obstacle_vector.points
    

def get_odom_data(odom_data):

    global current_pose
    
    current_pose = odom_data.pose.pose


def get_checkpoints()

    global coordinates 

    coordinates = np.empty((3,2))

    with open('./wayPoints.txt', 'r') as infile:
        data = infile.readlines()
        for i, waypoint in enumerate(data):
            coordinates[i] = waypoint.split()

    print(coordinates) 


def go_to_goal(current_goal_state)
    global goal_state 

    current_goal = checkpoints[current_goal_state]

    current_goal_vector = current_goal - current_pose

    resultant_vector = current_goal_vector + current_obstacle_vector

    # transformation to v,w
    l_mat = np.array([[1, 0],
             [0, 1/l]])
    rot_mat = np.array([[np.cos(-theta), -np.sin(-theta)],
               [np.sin(-theta), np.cos(theta)]])

    vw_vector = np.matmul(np.matmul(l_mat, rot_mat), vel_total)
    rospy.loginfo('vw_vector is ${0}'.format(vw_vector))

    #robot maximum velocity limits
    robot_max_linearx = 0.22
    robot_max_rotz = 2.84

    #If the calculated velocities exceed the maximum velocity limits, normalize and scale the vw_vector to the limit.
    if np.abs(vw_vector[0]) > robot_max_linearx:
        temp = vw_vector / np.linalg.norm(vw_vector)
        vw_vector = (robot_max_linearx/np.abs(temp[0])) * temp

    if np.abs(vw_vector[1]) > robot_max_rotz:
        temp = vw_vector / np.linalg.norm(vw_vector)
        vw_vector = (robot_max_rotz/np.abs(temp[1])) * temp

    twist = Twist()
    twist.linear.x = vw_vector[0]
    twist.angular.z = vw_vector[1]

    print(twist)
    pub.publish(twist)

    if np.abs(current_goal_vector) < 0.5:
        goal_state += 1
        pub.publish(Twist())
        rospy.sleep(10)
        print(goal_state)


def Init():
    global pub, goal_state

    get_checkpoints()
    
    goal_state = 0
    
    rospy.init_node('go_to_goal', anonymous=True)

    # Publish angle and distance
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	
    # Subscribe to LIDAR scan topic /scan
    rospy.Subscriber("/obstacle_vector", Marker, calc_optimal_vel, queue_size=1)
    
    # Subscribe to /odom
    rospy.Subscriber("/odom", Odometry, get_odom_data, queue_size=1)

    while goal_state < 4:
        go_to_goal(checkpoints[goal_state])

    rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass