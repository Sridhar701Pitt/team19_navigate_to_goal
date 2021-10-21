#!/usr/bin/env python
#Convert object location and LIDAR scan to range and angular values of the detected object

import rospy
from geometry_msgs.msg import Point, Vector3, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray

###################################
## VARIABLE DECLARATION AND SETUP
###################################

marker_array = MarkerArray()
visualize_Markers = True
marker_run_once = True

l = 0.05
globalAng = 0

current_pose = np.empty((1,2))
obstacle_vector_local = np.empty((1,2))

k_goal_factor = 2.0
run_once = True
Init_ang = None
Init_pos = lambda: None

goal_proximity_allow = 0.05   # stops within this distance from each goal point

#robot maximum velocity limits
robot_max_linearx = 0.15
robot_max_rotz = 1.8

max_obstacle_norm = 4.0

###################################
## Function Declaration
###################################
def define_vis_Markers(my_vector, rgb_vec, marker_count):
    global marker_array

    #Define the marker and assign values
    my_marker = Marker()
    my_marker.action = Marker.ADD
    my_marker.header.frame_id = '/marker_link'
    my_marker.header.stamp = rospy.Time.now()
    my_marker.ns = 'points_arrows'
    my_marker.type = Marker.ARROW
    my_marker.pose.orientation.y = 0
    my_marker.pose.orientation.w = 1
    my_marker.scale = Vector3(0.1, 0.1, 0.1)
    my_marker.color.r = rgb_vec[0]
    my_marker.color.g = rgb_vec[1]
    my_marker.color.b = rgb_vec[2]
    my_marker.color.a = 0.7
    my_marker.id = marker_count
    my_marker.points = [ Point(0, 0, 0), Point(my_vector[0][0], my_vector[0][1], 0) ]
    # Append this created marker to the array
    marker_array.markers.append(my_marker)

def update_vis_Markers(my_vector, marker_count):
    global marker_array

    #Update the marker values
    marker_array.markers[marker_count].points = [ Point(0, 0, 0), Point(my_vector[0][0], my_vector[0][1], 0) ]

def get_obstacle_vector(obs_vector):
    global obstacle_vector_local

    # Get the obstacle vector
    obstacle_vector_local[0] = [obs_vector.points[1].x, obs_vector.points[1].y]

def update_Odometry(Odom):
    global run_once
    global current_pose
    global globalAng
    global Init_ang, Init_pos
    
    position = Odom.pose.pose.position
    
    #Orientation uses the quaternion parametrization.
    #To get the angular position along the z-axis, the following equation is required.
    q = Odom.pose.pose.orientation
    orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    if run_once:
        #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
        run_once = False
        Init_ang = orientation

        globalAng = Init_ang
        Mrot = np.matrix([[np.cos(Init_ang), np.sin(Init_ang)],[-np.sin(Init_ang), np.cos(Init_ang)]])        

        Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
        Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
        Init_pos.z = position.z

    Mrot = np.matrix([[np.cos(Init_ang), np.sin(Init_ang)],[-np.sin(Init_ang), np.cos(Init_ang)]])        

    #We subtract the initial values
    # globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - Init_pos.x
    # globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - Init_pos.y
    current_pose[0][0] = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - Init_pos.x
    current_pose[0][1] = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - Init_pos.y
    globalAng = orientation - Init_ang

    # current_pose[0][0] = globalPos.x
    # current_pose[0][1] = globalPos.y

def load_checkpoints():
    global checkpoints 

    checkpoints = np.empty((3,2))
    with open('./wayPoints.txt', 'r') as infile:
        data = infile.readlines()
        for i, waypoint in enumerate(data):
            checkpoints[i] = waypoint.split()


def go_to_goal(goal_point):
    global marker_run_once

    lookahead_pose = np.array([[current_pose[0][0] + l*np.cos(globalAng), current_pose[0][1]+l*np.sin(globalAng)]])
    # Make a constant field for the goal vector (constant magnitude attractor)
    current_goal_vector_unnorm = goal_point - lookahead_pose
    current_goal_vector = k_goal_factor * (current_goal_vector_unnorm / np.linalg.norm(current_goal_vector_unnorm))

    # Transform obstacle vector into global coordinates (from local)
    rot = np.array([[np.cos(globalAng), -np.sin(globalAng)], [np.sin(globalAng), np.cos(globalAng)]])
    obstacle_vector = np.transpose(np.matmul(rot, np.transpose(obstacle_vector_local)))

    obstacle_vector_t = [obstacle_vector[0][0],obstacle_vector[0][1]]
    current_goal_vector_t = [current_goal_vector[0][0],current_goal_vector[0][1]]

    if np.sign(np.dot(obstacle_vector_t, current_goal_vector_t)) > 0.0:
        #diminish the effect of obstacle vector
        obstacle_vector = obstacle_vector/2.0
    else:
        #Add some angular shift          
        direction = np.sign(np.cross(np.array([obstacle_vector[0][0],obstacle_vector[0][1],0.0]),np.array([current_goal_vector[0][0],current_goal_vector[0][1],0.0]))[2])
        heuristic_angle = 90*1*(max_obstacle_norm - np.linalg.norm(obstacle_vector))/max_obstacle_norm

        rotation_offset = np.deg2rad(heuristic_angle)
        rot = np.array([[np.cos(rotation_offset), -np.sin(rotation_offset)], [np.sin(rotation_offset), np.cos(rotation_offset)]])
        obstacle_vector = np.transpose(np.matmul(rot, np.transpose(obstacle_vector)))


    # Find the resultant vector of the above two vectors
    resultant_vector = current_goal_vector + obstacle_vector


    # transformation to v,w, convert this vector to cmd_vel values
    l_mat = np.array([[1, 0],
            [0, 1/l]])
    rot_mat = np.array([[np.cos(-globalAng), -np.sin(-globalAng)],
            [np.sin(-globalAng), np.cos(-globalAng)]])

    vw_vector = np.matmul(np.matmul(l_mat, rot_mat), np.transpose(resultant_vector))
    

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

    rospy.loginfo('vw_vector is {0}'.format(vw_vector))
    pub.publish(twist)

    if visualize_Markers:
        # create and publish all vectors for visualization
        if marker_run_once:
            define_vis_Markers(current_goal_vector, [0.0, 1.0, 0.0], 0)
            define_vis_Markers(obstacle_vector, [0.0, 0.0, 1.0], 1)
            define_vis_Markers(resultant_vector, [1.0, 0.0, 0.0], 2)
            marker_run_once = False
        else:
            update_vis_Markers(current_goal_vector, 0)
            update_vis_Markers(obstacle_vector, 1)
            update_vis_Markers(resultant_vector, 2)

        pub_marker_array.publish(marker_array)

def Init():
    global pub, pub_marker_array

    # Initilize the node
    rospy.init_node('go_to_goal', anonymous=True)

    ## Initialize all Publishers
    # Publish angular and linear velocities to /cmd_vel
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Publish vector MarkersArray for visualization
    pub_marker_array = rospy.Publisher("/potential_vectors", MarkerArray, queue_size=10)

    # Sleep a bit to manually add visualization vectors to rviz
    rospy.sleep(1)
	
    ## Initialize all Subscribers
    # Subscribe to obstacle vector topic from get_Object_Range node
    rospy.Subscriber("/obstacle_vector", Marker, get_obstacle_vector, queue_size=1)
    
    # Subscribe to /odom from turtlebot3_bringup turtlebot3_robot.py
    rospy.Subscriber("/odom", Odometry, update_Odometry, queue_size=1)

    # Load all goal coordinates into a global variable: "checkpoints"
    load_checkpoints()

    # Checkpoint index to be reached
    goal_index = 0
    while goal_index < 3 and not rospy.is_shutdown():
        if np.linalg.norm(checkpoints[goal_index] - current_pose) < goal_proximity_allow:
            goal_index += 1
            pub.publish(Twist())
            rospy.sleep(10)
            continue

        go_to_goal(checkpoints[goal_index])
        rospy.sleep(0.25)

    print(Yayyy!!!!)
    rospy.sleep(5.0)


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass