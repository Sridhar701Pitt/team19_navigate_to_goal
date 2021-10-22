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
MIN_VELOCITY = -0.5
MAX_YAWRATE = 2.0
MAX_ACCELERATION = 1.0
MAX_D_YAWRATE= 2.0          # Maximum value for angular accelaration

#MAX_DIST= 10.0
MAX_DIST_THRESHOLD = 1.0

VELOCITY_RESOLUTION = 0.05
YAWRATE_RESOLUTION = 0.05

PREDICT_TIME = 3.0
DT = 1.0 / HZ

TO_GOAL_COST_GAIN = 1.0
SPEED_COST_GAIN = 1.0
OBSTACLE_COST_GAIN = 1.0

USE_SCAN_AS_INPUT = True

GOAL_THRESHOLD = 0.1
TURN_DIRECTION_THRESHOLD = 1.0

MIN_OBSTACLE_DIST = 0.1 

run_once = True
globalAng = 0
Init_ang = None
Init_pos = lambda: None
current_position = lambda: None


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

def update_Odometry(Odom):
    global run_once
    global current_position
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
    current_position.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - Init_pos.x
    current_position.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - Init_pos.y
    globalAng = orientation - Init_ang

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

    global current_velocity, current_position, odom_updated, odom_x_y_pos
    
    current_velocity = odom_data.twist.twist
    # current_position = odom_data.pose.pose.position

    update_Odometry(odom_data)

    odom_x_y_pos = np.array([current_position.x, current_position.y])
    
    odom_updated = True


# Converts laser scan observations from (r,theta) to (x,y) obstacle locations
def scan_to_obs():

    obstacles_list = list()

    angle = scan.angle_min

    scan_ranges = np.asarray(scan.ranges)

    scan_ranges = np.where(scan_ranges > MAX_DIST_THRESHOLD, 0, scan_ranges)
    
    for r in scan_ranges:
        if r != 0:
            x = r * math.cos(angle + globalAng) + current_position.x
            y = r * math.sin(angle + globalAng) + current_position.y

            angle += scan.angle_increment

            obstacles_coords = [x, y]
            obstacles_list.append(obstacles_coords)

    # print("Obstacles_list: ",obstacles_list)
    
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

    next_state = State(0.0,0.0,0.0,0.0,0.0)
    
    next_state.yaw = state.yaw + yawrate * DT

    next_state.x = state.x + velocity * math.cos(state.yaw) * DT
    next_state.y = state.y + velocity * math.sin(state.yaw) * DT

    next_state.velocity = velocity
    next_state.yawrate = yawrate

    return next_state



def calc_to_goal_cost(trajectory, goal):

    # last_position = np.array([trajectory.x, trajectory[-1].y])

    #costs = list()

    min_cost = 1e6

    for state in trajectory:
        position = np.array([state.x, state.y])
        # costs = costs.append(np.linalg.norm(position - goal))

        cost = np.linalg.norm(position - goal)

        min_cost = min(min_cost, cost)

    return min_cost


def calc_speed_cost(trajectory, target_velocity):

    cost = abs(target_velocity - abs(trajectory[-1].velocity))

    return cost


def calc_obstacle_cost(trajectory, obstacles_list):

    cost = 0.0
    min_dist = 1e3

    for state in trajectory:
        for obstacle_coord in obstacles_list:

            dist = math.sqrt((state.x - obstacle_coord[0])**2 + (state.y - obstacle_coord[1])**2)

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
    best_trajectories = list()

    for v in np.arange(dynamic_window.min_velocity, dynamic_window.max_velocity + VELOCITY_RESOLUTION, VELOCITY_RESOLUTION):
        for y in np.arange(dynamic_window.min_yawrate, dynamic_window.max_yawrate + YAWRATE_RESOLUTION, YAWRATE_RESOLUTION):

            # print("Velocity Considered: ", v , y)

            state = State(current_position.x, current_position.y, globalAng, current_velocity.linear.x, current_velocity.angular.z) # update with current position

            trajectory = list()

            for t in np.arange(0, PREDICT_TIME, DT):
                state = motion(state, v, y)
                trajectory.append(state)
            
            # trajectories.append(trajectory)

            to_goal_cost = calc_to_goal_cost(trajectory, goal)
            # speed_cost = calc_speed_cost(trajectory, TARGET_VELOCITY)
            obstacle_cost = calc_obstacle_cost(trajectory, obstacles_list)

            # Add smoothing to the cost function and normalise the values to [0,1]
            # Add a term which minimises the amount of deviation from the previous v, w ---> for smoother trajectories and reduce jerks
            # current_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost
            current_cost = TO_GOAL_COST_GAIN * to_goal_cost + OBSTACLE_COST_GAIN * obstacle_cost

            #print("to_goal_cost: ",to_goal_cost)
            # print("speed_cost: ",speed_cost)
            # print("obstacle_cost: ",obstacle_cost)
            # print("current_Cost: ",current_cost)
            if current_cost <= min_cost:
                min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost
                min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost
                #min_speed_cost = SPEED_COST_GAIN * speed_cost
                
                min_cost = current_cost
                best_trajectories.append(trajectory)

                # print("Velocity Considered: ", v , y)
                # print("current_Cost:", current_cost)

    # print("Current Position: ", current_position.x, current_position.y)


    # print("Cost: ", min_cost)
    # print("- Goal cost: ", min_goal_cost)
    # print("- Obs cost: ", min_obs_cost)
    # print("- Speed cost: ", min_speed_cost)
    # print("num of trajectories: ", len(trajectories))
    
    if min_cost == 1e6:
        trajectory = [State(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.angular.z)]
        best_trajectories.append(trajectory)
    
    return best_trajectories[-1]


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
        traj_point.x = traj_pose.position.x
        traj_point.y = traj_pose.position.y
        vis_trajectory.points.append(traj_point)
    
    pub.publish(vis_trajectory)


###################################
##  Main Function
###################################

def Init():

    global scan_updated, odom_updated
    scan_updated = False
    odom_updated = False

    next_checkpoint = 0

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

            goal = goals[next_checkpoint]
            print("\n Goal Coordinates: ", goal[0], ",", goal[1])

            if np.linalg.norm(goal - odom_x_y_pos) > GOAL_THRESHOLD:

                print(np.linalg.norm(goal - odom_x_y_pos))
                obstacles_list = scan_to_obs()
                scan_updated = False
            
                best_trajectory = dwa_planning(dynamic_window, goal, obstacles_list)
                
                optimal_vel.linear.x = best_trajectory[0].velocity
                optimal_vel.angular.z = best_trajectory[0].yawrate
                print("Optimal_vel",optimal_vel.linear.x,optimal_vel.angular.z)

                # visualise_trajectory(best_trajectory, 1, 0, 0, best_trajectory_pub)

            else:
                optimal_vel.linear.x = 0.0
                optimal_vel.angular.z = 0.0



                next_checkpoint += 1
                velocity_pub.publish(optimal_vel)

                rospy.sleep(5 * next_checkpoint)

            print("\n Optimal Velocity: ", optimal_vel.linear.x, "m/s", optimal_vel.angular.z, "rad/s")
            velocity_pub.publish(optimal_vel)

            odom_updated = False

        else:
            '''
            if scan_updated == False:
                print("\n Scan not updated")
            
            if goal_subscribed == False:
                print("\n Goal not received")
            
            if odom_updated == False:
                print("\n Odom not updated")'''

    rospy.spin()


if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass