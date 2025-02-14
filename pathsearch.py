#!/usr/bin/env python3
""" Module for finding the optimal path using A-star"""

from collections import deque
from typing import Deque, Tuple, Dict
from numba import jit, njit
import numpy as np
import heapq as hq
import math
import time
import config
import obstacles

def find_path(start, goal, actions, a_star_weight=1):
    """ Calculates the optimal path from a starting point to a goal point.
    a_star_weight can be adjusted to use weighted-A_start method if needed.
    Returns a results dictoray containing runtime, visited nodes, parent nodes,
    step_info and runtime.
   """
    
    open_list = []  # to be used for heapq for nodes in the open-list
    visited_node_map = {} # key = index in layout grid, val = node rounded to nearest 0.5
    parent_node_map = {} # (key=node, val=parent)
    lowest_cost_map ={} #(key=node, val=c2c), used to ensure only lowest cost nodes in open-list hq
    step_info_map = {} # key = node, val= dict(linear_velocity, angular_velocity, distance_covered

    start_x, start_y, start_theta = start
    goal_x, goal_y = goal

    a_star_strt_time = time.time()  # used to calculate how long the code runs.
    print('\nstarting A* search, please wait. The entire search could take 12 to 40 seconds.\n')

    # Starting cost-to-come and parent node
    cost2come_start = 0.0                                                                               
    cost2go_start = _euclidean_distance(start_x, start_y, goal_x, goal_y)
    vel_start, theta_dot_start, dist_start  = 0, 0, 0
    # creating tuple with cost to come and coordinate values (x,y, theta) 
    total_cost_start = cost2come_start + a_star_weight*cost2go_start
    n1 = (total_cost_start, cost2come_start,(start_x, start_y, start_theta))  

    #Push elements to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, n1)
    # Update maps with starting point info
    lowest_cost_map[(start_x, start_y, start_theta)] = cost2come_start
    parent_node_map[(start_x, start_y, start_theta)] = None
    step_info_map[(start_x, start_y, start_theta)] = {'velocity':vel_start, 
                                                      'turning_angular_spd':theta_dot_start, 
                                                      'distance': dist_start}

    # a_star while loop to generate new nodes
    while len(open_list) > 0:
        active_node = hq.heappop(open_list)  # remove node with highest priority from heapq
        _, curnt_c2c, curnt_x, curnt_y, curnt_theta = active_node[0], active_node[1], active_node[2][0], active_node[2][1], active_node[2][2]
        # Look to see if the node from the open_list has already been found (i.e is in the lowest_cost_map),
        # if it's cost-to-come is > then the previous one, then ignore it. 
        if curnt_c2c > lowest_cost_map.get((curnt_x, curnt_y, curnt_theta), math.inf):
            continue
        # Determine what the node's index would be if a matrix was used per the project pdf.
        visited_index = calculate_grid_index(curnt_x, curnt_y, curnt_theta)
        visited_node_map[visited_index]= (curnt_x, curnt_y, curnt_theta) # updated visited map
        # Check if node is within a radius of goal, then backstracks if yes. 
        if ((curnt_x - goal_x)**2 + (curnt_y - goal_y)**2) <= config.GOAL_REACHED_RADIUS**2:
            print(f"Goal point {(goal_x*10, (config.WINDOW_HEIGHT-goal_y)*10)} (in mm) reached!")
            # Backtracking
            current_node = (curnt_x, curnt_y, curnt_theta)
            # Keep backtracking until start node reached
            final_path, step_info = _reconstruct_path(parent_node_map, step_info_map, current_node)

            print('Path found!')

            return {
            'path': final_path,
            'step_info': step_info,
            'visited_nodes': list(visited_node_map.values()),
            'runtime': time.time() - a_star_strt_time,
            'success': True,
            'message': "Path found!"
                    } 

        # Explore neighbors with 8 possible actions
        for act in actions:
            nx, ny, n_theta, n_c2c, n_dist, n_vel, n_theta_dot = _move(curnt_x, curnt_y, curnt_theta, curnt_c2c, act)
            # Ignore node if it's in the obstacle space.
            if (nx,ny) in obstacles.OBSTACLE_POINTS:
                continue

            # Checks that the new node has not already been visited
            if calculate_grid_index(nx, ny, n_theta) not in visited_node_map:          
                # only add node to open list and other dictionaries, if the new node is not in the lowest_cost_dictionary or 
                # if the new node's c2c is lower than the one in the lowest_cost dictionary. 
                if (nx, ny, n_theta) not in lowest_cost_map or n_c2c < lowest_cost_map[(nx, ny, n_theta)]:
                    n_c2g = _euclidean_distance(nx, ny, goal_x, goal_y)
                    n_tc = n_c2c + a_star_weight * n_c2g
                    lowest_cost_map[(nx, ny, n_theta)] = n_c2c
                    parent_node_map[(nx, ny, n_theta)] = (curnt_x, curnt_y, curnt_theta)
                    step_info_map[(nx, ny, n_theta)] = {'velocity': n_vel, 
                                                      'turning_angular_spd': n_theta_dot, 
                                                      'distance': n_dist}
                    # node_action_map[(nx, ny, n_theta)] = (act[0], act[1])
                    hq.heappush(open_list, (n_tc, n_c2c, (nx, ny, n_theta)))
    # Stop if the algorithm can't find a solution
    return {
    'path': None,
    'path_info': None,
    'visited_nodes': list(visited_node_map.values()),
    'runtime': time.time() - a_star_strt_time,
    'success': False,
    'message': "No Path found!"
            } 

def _reconstruct_path(parent_map: dict, step_info_map: dict, current: Tuple) -> Deque:
    """ 
    backtracks to create a path from the current node back to the initial node. 
    Returns the final path.
    
    Args:
        current (tuple): x,y, theta coordinate of final point
        parent_node_map (dict): nodes and their parent nodes
        step_info_map (dict): nodes and linear velocity, turning radius & distance

    Returns:
        path (deque): deque containing x,y of points on the final path.
        step_info (deque): deque containing dictionaries with linear velocity, turning speed & distance 
    """
    path = deque([current]) # create deque & add node to it.
    step_info = deque([step_info_map[current]]) # create deque & add node to it.
    while current in parent_map:
        current = parent_map[current]
        if current is None: # stop when starting node is found
            break
        path.appendleft(current)
        step_info.appendleft(step_info_map[current])
    return path, step_info

@njit(fastmath=True) # same as @jit(nopython=True, fastmath=True)
def calculate_grid_index(x, y, theta):
    """ calculates the grid index closest to the point """

    x_idx = round(x / config.THRESHOLD_X)
    y_idx = round(y / config.THRESHOLD_Y)
    theta_idx = round(theta / config.THRESHOLD_THETA)
    return x_idx, y_idx, theta_idx

def _euclidean_distance(point1_x, point1_y, point2_x, point2_y):
    """ calculates the euclidean distance between two points.Used for c2g calculation """
    return round(math.sqrt((point2_y - point1_y)**2 + (point2_x - point1_x)**2), 2)

@njit 
def round_near_point5(number):
    """ rounds number to the nearest 0.5 decimal. If the number is close to 0.5, it rounds it to that, 
    otherwise it rounds it to the closest whole number"""
    return round(number*2)/2
    
@jit(nopython=True, fastmath=True) # produces highly optimized machine code that doesn't rely on python compile.
def _move(Xi,Yi,Thetai,cost_to_come,wheel_rpms):
    """ Calculates the velocity, anguar speed, distance covered and new c2c.
     Using Euler numerical integration method. """
    ang_spd_l, ang_spd_r = wheel_rpms
    t = 0
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = np.radians(Thetai) 
    ang_spd_l = ang_spd_l*2*np.pi/60 # convert rpm to rad/s
    ang_spd_r = ang_spd_r*2*np.pi/60 # convert rpm to rad/s
    # Determin linear velocity and steering angular velocity
    x_dot = 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.cos(Thetan) 
    y_dot = 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.sin(Thetan)
    theta_dot = (config.WHEEL_RADIUS/config.WHEEL_SEPARATION) * (ang_spd_r - ang_spd_l)
    vel = np.sqrt(x_dot**2 + y_dot**2)
    distance=0
    # While loop for numerical integration
    while t<1:
        t = t + dt
        Xn += 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.cos(Thetan) * dt
        Yn += 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.sin(Thetan) * dt
        # Thetan += (r / L) * (ang_spd_r - ang_spd_l) * dt
        Thetan += (config.WHEEL_RADIUS/config.WHEEL_SEPARATION) * (ang_spd_r - ang_spd_l) * dt
        distance += np.sqrt((0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.cos(Thetan) * dt)**2 \
                      + (0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * np.sin(Thetan) * dt)**2)
    new_c2c = cost_to_come + distance  
    Thetan = np.degrees(Thetan) % 360 # convert to degrees then normalize to 360 deg

    return round_near_point5(Xn), round_near_point5(Yn), Thetan, round(new_c2c,1), round(distance,2), round(vel,4), round(theta_dot,4)