#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Vector3, Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
from visualization_msgs.msg import Marker 


###################################
## DWA PARAMETERS
###################################

HZ = 20
ROBOT_FRAME = "base_link"

TARGET_VELOCITY = 0.8       # Should depend on required velocity between final position of robot (in a simulated trajectory) and goal ---> large distance, high vel | small distance, low vel

MAX_VELOCITY = 1.0
MIN_VELOCITY = 0.0
MAX_YAWRATE = 0.8
MAX_ACCELERATION = 1.0
MAX_D_YAWRATE= 2.0          # Maximum value for angular accelaration

MAX_DIST= 10.0

VELOCITY_RESOLUTION = 0.1
YAWRATE_RESOLUTION = 0.1

PREDICT_TIME = 3.0
DT = 1.0 / HZ

TO_GOAL_COST_GAIN = 1.0
SPEED_COST_GAIN = 1.0
OBSTACLE_COST_GAIN = 1.0

USE_SCAN_AS_INPUT = True

GOAL_THRESHOLD = 0.1
TURN_DIRECTION_THRESHOLD = 1.0

MIN_OBSTACLE_DIST = 0.1 


###################################
## Custom Data Type Definitions
###################################

class Window:

    def __init__(self, min_v = 0.0, max_v = 0.0, min_y = 0.0, max_y = 0.0):
        self.min_velocity = min_v
        self.max_velocity = max_v
        self.min_yawrate = min_y
        self.max_yawrate = max_y


class State:

    def __init__(self, x, y, yaw, velocity, yawrate):

        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.yawrate = yawrate


###################################
## Call Backs and Helper Functions
###################################

# Loads goal coordinates from the text file 
def get_checkpoints():

    global checkpoints, goal_subscribed

    checkpoints = np.empty((3,2))

    with open('./wayPoints.txt', 'r') as infile:
        data = infile.readlines()
        for i, waypoint in enumerate(data):
            checkpoints[i] = waypoint.split()
        
        goal_subscribed = True

    print("Checkpoints: ", checkpoints)

    return checkpoints 


def scan_callback(scan_msg):

    global scan, scan_updated
    
    scan = scan_msg
    scan_updated = True


def odom_callback(odom_data):

    global current_velocity, current_position, odom_updated
    
    current_velocity = odom_data.twist
    cuurent_position = odom_data.pose.pose.position
    odom_updated = True


# Converts laser scan observations from (r,theta) to (x,y) obstacle locations
def scan_to_obs():

    obstacles_list = list()

    angle = scan.angle_min
    
    for r in scan.ranges:
        x = r * math.cos(angle)
        y = r * math.cos(angle)

        angle += scan.angle_increment
    
    return obstacles_list


###################################
##  DWA Planner Functions
###################################

def calc_dynamic_window(current_velocity):

    window = Window(MIN_VELOCITY, MAX_VELOCITY, -1.0 * MAX_YAWRATE, MAX_YAWRATE)

    window.min_velocity = max((current_velocity.linear.x - MAX_ACCELERATION * DT), MIN_VELOCITY)
    window.max_velocity = min((current_velocity.linear.x + MAX_ACCELERATION * DT), MAX_VELOCITY)
    window.min_yawrate = max((current_velocity.angular.z - MAX_D_YAWRATE * DT), -MAX_YAWRATE)
    window.max_yawrate = min((current_velocity.angular.z + MAX_D_YAWRATE * DT),  MAX_YAWRATE)
    
    return window


def motion(state, velocity, yawrate):
    
    state.yaw += yawrate * DT

    state.x += velocity * math.cos(state.yaw) * DT
    state.y += velocity * math.sin(state.yaw) * DT

    state.velocity = velocity
    state.yawrate = yawrate


def calc_to_goal_cost(trajectory, goal):

    last_position = np.array([trajectory[-1].x, trajectory[-1].y])

    cost = np.linalg.norm(last_position, goal)

    return cost


def calc_speed_cost(trajectory, target_velocity):

    cost = abs(target_velocity - abs(trajectory[-1].velocity))

    return cost


def calc_obstacle_cost(trajectory, obstacles_list):

    cost = 0.0
    min_dist = 1e3

    for state in trajectory:
        for obstacle_coord in obstacles_list:

            dist = math.sqrt((state.x - obstacle_coord[0] ** 2) + (state.y - obstacle_coord[1]))

            if dist <= MIN_OBSTACLE_DIST:
                cost = 1e6
                return cost
            
            min_dist = min(min_dist, dist)
    
    cost = min_dist

    return cost


def dwa_planning(dynamic_window, goal, obstacles_list):

    min_cost = 1e6
    min_obs_cost = min_cost
    min_goal_cost = min_cost
    min_speed_cost = min_cost

    trajectories = list()
    best_trajectory = list()

    for v in range(dynamic_window.min_velocity, dynamic_window.max_velocity, VELOCITY_RESOLUTION):
        for y in range(dynamic_window.min_yawrate, dynamic_window.max_yawrate, YAWRATE_RESOLUTION):

            state = State(current_position.x, current_position.y, 0.0, current_velocity.linear.x, current_velocity.angular.z) # update with current position

            trajectory = list()

            for t in range(0, PREDICT_TIME, DT):
                motion(state, v, y)
                trajectory.append(state)
            
            trajectories.append(trajectory)

            to_goal_cost = calc_to_goal_cost(trajectory, goal)
            speed_cost = calc_speed_cost(trajectory, TARGET_VELOCITY)
            obstacle_cost = calc_obstacle_cost(trajectory, obs_list)

            # Add smoothing to the cost function and normalise the values to [0,1]
            # Add a term which minimises the amount of deviation from the previous v, w ---> for smoother trajectories and reduce jerks
            current_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost

            if current_cost <= min_cost:
                min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost
                min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost
                min_speed_cost = SPEED_COST_GAIN * speed_cost
                
                min_cost = current_cost
                best_trajectory.append(trajectory)

    print("\n Cost: ", min_cost)
    print("\n - Goal cost: ", min_goal_cost)
    print("\n - Obs cost: ", min_obs_cost)
    print("\n - Speed cost: ", min_speed_cost)
    print("\n num of trajectories: ", len(trajectories))
    
    if min_cost == -1:
        trajectory = State(0,0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z)
        best_trajectory.append(trajectory)
    
    return best_trajectory[-1]


def visualise_trajectory(trajectory, r, g, b, pub):

    vis_trajectory = Marker()

    vis_trajectory.action = Marker.ADD
    vis_trajectory.header.frame_id = '/base_scan'
    vis_trajectory.header.stamp = rospy.Time.now()
    vis_trajectory.ns = 'points_arrows'
    vis_trajectory.id = 131
    vis_trajectory.type = Marker.LINE_STRIP
    vis_trajectory.color.r = r
    vis_trajectory.color.g = g
    vis_trajectory.color.b = b
    vis_trajectory.color.a = 0.8
    vis_trajectory.scale = 1

    traj_pose = Pose()
    traj_pose.orientation.w = 1
    vis_trajectory.pose = traj_pose

    traj_point = Point()
    for pose in trajectory:
        traj_point.x = traj_pose.x
        traj_point.y = traj_pose.y
        vis_trajectory.points.append(traj_point)
    
    pub.publisher(vis_trajectory)


###################################
##  Main Function
###################################

def Init():

    goals = get_checkpoints()
        
    rospy.init_node('go_to_goal_DWA', anonymous=True)

    # Publish angle and distance
    velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Visualise the best trajectory by publishing Marker message
    best_trajectory_pub = rospy.Publisher("best_trajectory", Marker, queue_size = 1)

    # Subscribe to LIDAR scan topic /scan
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)

    # Subscribe to /odom
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)
    
    optimal_vel = Twist()

    while not rospy.is_shutdown():
        
        if scan_updated == True and goal_subscribed == True and odom_updated == True:
            dynamic_window = calc_dynamic_window(current_velocity)

            goal = goal[0]
            print("\n Goal Coordinates: ", goal[0], ",", goal[1])

            if linalg.norm(goal) > GOAL_THRESHOLD:
                obstacles_list = scan_to_obs()
                scan_updated = False
            
                best_trajectory = dwa_planning(dynamic_window, goal, obstacles_list)
                
                optimal_vel.linear.x = best_trajectory[0].velocity
                optimal_vel.angular.z = best_trajectory[0].yawrate

                visualise_trajectory(best_trajectory, 1, 0, 0, best_trajectory_pub)

            else:
                optimal_vel.linear.x = 0.0
                optimal_vel.angular.z = 0.0

            print("\n Optimal Velocity: ", optimal_vel.linear.x, "m/s", optimal_vel.angular.z, "rad/s")
            velocity_pub.publish(optimal_vel)

            odom_updated = False

        else:
            
            if scan_updated == False:
                print("\n Scan not updated")
            
            if goal_subscribed == False:
                print("\n Goal not received")
            
            if odom_updated == False:
                print("\n Odom not updated")

    rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass