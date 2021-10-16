#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point, Vector3, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from visualization_msgs.msg import Marker 

###################################
## VARIABLE DECLARATION AND SETUP
###################################

l = 0.15
theta = 0

current_pose = np.empty((1,2))
current_obstacle_vector = np.empty((1,2))

###################################
## Function Declaration
###################################
def goal_arrow_data(goal_vector)):

    goal_vector = Marker()
    goal_vector.action = Marker.ADD
    goal_vector.header.frame_id = '/base_goal_vector'
    goal_vector.header.stamp = rospy.Time.now()
    goal_vector.ns = 'points_arrows'
    goal_vector.id = 131
    goal_vector.type = Marker.ARROW
    goal_vector.pose.orientation.y = 0
    goal_vector.pose.orientation.w = 1
    goal_vector.scale = Vector3(0.1, 0.1, 0.1)
    goal_vector.color.r = 0.2
    goal_vector.color.g = 0.5
    goal_vector.color.b = 1.0
    goal_vector.color.a = 0.3

    goal_vector.points = [ Point(0, 0, 0), Point(goal_vector[0][0], goal_vector[0][1], 0) ]
    
    pub_goal_vector.publish(goal_vector)


def resultant_arrow_data(resultant_vector)):

    resultant_vector = Marker()
    resultant_vector.action = Marker.ADD
    resultant_vector.header.frame_id = '/base_resultant_vector'
    resultant_vector.header.stamp = rospy.Time.now()
    resultant_vector.ns = 'points_arrows'
    resultant_vector.id = 111
    resultant_vector.type = Marker.ARROW
    resultant_vector.pose.orientation.y = 0
    resultant_vector.pose.orientation.w = 1
    resultant_vector.scale = Vector3(0.1, 0.1, 0.1)
    resultant_vector.color.r = 0.2
    resultant_vector.color.g = 0.5
    resultant_vector.color.b = 1.0
    resultant_vector.color.a = 0.3

    resultant_vector.points = [ Point(0, 0, 0), Point(resultant_vector[0][0], resultant_vector[0][1], 0) ]
    
    pub_resultant_vector.publish(resultant_vector)


def calc_optimal_vel(obstacle_vector):

    global current_obstacle_vector

    current_obstacle_vector[0][0] = obstacle_vector.points[1].x
    current_obstacle_vector[0][1] = obstacle_vector.points[1].y

    

def get_odom_data(odom_data):

    global current_pose
    
    current_pose[0][0] = odom_data.pose.pose.position.x
    current_pose[0][1] = odom_data.pose.pose.position.y

    print("Current Odometry Pose: ", current_pose) 


def get_checkpoints():

    global checkpoints 

    checkpoints = np.empty((3,2))

    with open('./wayPoints.txt', 'r') as infile:
        data = infile.readlines()
        for i, waypoint in enumerate(data):
            checkpoints[i] = waypoint.split()

    print("Checkpoints: ", checkpoints) 


def go_to_goal(current_goal_state):
    global goal_state 

    current_goal = checkpoints[goal_state]

    current_goal_vector = current_goal - current_pose

    goal_arrrow_data(current_goal_vector)

    resultant_vector = current_goal_vector + current_obstacle_vector

    resultant_arrow_data(resultant_vector)

    # transformation to v,w
    l_mat = np.array([[1, 0],
             [0, 1/l]])
    rot_mat = np.array([[np.cos(-theta), -np.sin(-theta)],
               [np.sin(-theta), np.cos(theta)]])

    vw_vector = np.matmul(np.matmul(l_mat, rot_mat), np.transpose(resultant_vector))
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

    # print(twist)
    pub.publish("Published Twist Message: ", twist)

    if np.linalg.norm(current_goal_vector) < 0.5:
        goal_state += 1
        pub.publish(Twist())
        rospy.sleep(10)
        print("Current Goal State: ", goal_state)


def Init():
    global pub, goal_state

    get_checkpoints()
    
    goal_state = 0
    
    rospy.init_node('go_to_goal', anonymous=True)

    # Publish angle and distance
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Publish angle and distance
    pub_goal_vector = rospy.Publisher("/goal_vector", Marker, queue_size=10)

    # Publish angle and distance
    pub_resultant_vector = rospy.Publisher("/resultant_vector", Marker, queue_size=10)
	
    # Subscribe to LIDAR scan topic /scan
    rospy.Subscriber("/obstacle_vector", Marker, calc_optimal_vel, queue_size=1)
    
    # Subscribe to /odom
    rospy.Subscriber("/odom", Odometry, get_odom_data, queue_size=1)

    while goal_state < 4 and not rospy.is_shutdown():
        go_to_goal(checkpoints[goal_state])
        rospy.sleep(0.25)

    rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass