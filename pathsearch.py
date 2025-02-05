from collections import deque
from typing import Deque, Tuple, Dict
import heapq as hq
import math
import time
import config
import obstacles

def find_path(start, goal, wheel_rpm, actions, a_star_weight=1):
    """ Calculates the optimal path from a starting point to a goal point entered by the user.
    It takes the a_star weight factor, and a number to prompt or not to input the start point & left and right wheel rpm
    strt_and_rpm_input_sw = 1-> ask for start & rpm input, 0 = use defaults.
    gazebo_coord_sw =1 -> convert user input in gazebo coord to pygame. gazebo_coord_sw =0 -> convert from coord on bottom left of layout to pygame coord"""
    
    open_list = []  # to be used for heapq for nodes in the open-list
    visited_matrix_index_set = set() #set used to store the index of each visited node if they were added to a visted-node-matrix (see proj3 part 1 pdf page 14). replaces the closed-list from proj2
    parent_node_map = {} # dictionary (key=node, val=parent) for all the nodes visited mapped to their parents to be used for backtracking
    lowest_cost_map ={} # dictionary (key=node, val=c2c) to keep track of the nodes and their cost, used to ensure only lowest cost in open-list
    step_info_map = {} # dictionary (key = node, val= (linear_velocity, angular_velocity, distance_covered) to keep track of the nodes, traveled distance, linear & angular velocities
    threshold_x, threshold_y, threshold_theta = 0.5, 0.5, 30

    start_x, start_y, start_theta = start
    goal_x, goal_y = goal
    RPM_L, RPM_R = wheel_rpm

    a_star_strt_time = time.time()  # used to calculate how long the code runs.
    print('starting A* search, please wait \n')

    # Starting cost-to-come and parent node
    cost2come_start = 0.0                                                                               
    cost_2_go_start = _euclidean_distance(start_x, start_y, goal_x, goal_y)
    vel_start, theta_dot_start, dist_start  = 0, 0, 0
    # creating tuple with cost to come and coordinate values (x,y, theta) 
    total_cost_start = cost2come_start + a_star_weight*cost_2_go_start
    n1 = (total_cost_start, cost2come_start,(start_x, start_y, start_theta))  

    #Push elements to heap queue which also simultaneously heapifies the queue
    hq.heappush(open_list, n1)
    # Update lowest cost and parent node maps with starting point info
    lowest_cost_map[(start_x, start_y, start_theta)] = cost2come_start
    parent_node_map[(start_x, start_y, start_theta)] = None
    step_info_map[(start_x, start_y, start_theta)] = {'velocity':vel_start, 
                                                      'turning_angular_spd':theta_dot_start, 
                                                      'distance': dist_start}

    # a_star while loop to generate new nodes
    while len(open_list) > 0:
        active_node = hq.heappop(open_list)  # remove one node from heapq
        _, curnt_c2c, curnt_x, curnt_y, curnt_theta = active_node[0], active_node[1], active_node[2][0], active_node[2][1], active_node[2][2]
        # Look to see if the node from the open_list has already been found (i.e is in the lowest_cost_map), if it's cost-to-come is > then the one previous one, then ignore it. 
        if lowest_cost_map.get((curnt_x, curnt_y, curnt_theta)) is not None and curnt_c2c > lowest_cost_map.get((curnt_x, curnt_y, curnt_theta)):
            continue
        # rounds the x, y, theta to the nearest 0.5, then determine what its index would be if a matrix was used per page 14 of the project pdf, add it to the visited node set. 
        visited_matrix_index_set.add((_round_near_point5(curnt_x)/threshold_x, _round_near_point5(curnt_y)/threshold_y, curnt_theta/threshold_theta))

        # Check if we've reached the goal,if yes, backtrack to find path. If node is within 1.5 units of goal, then it's close enough. 
        if ((curnt_x - goal_x)**2 + (curnt_y - goal_y)**2) <= 1.5**2: #and (goal_theta-15 <= curnt_theta <= goal_theta+15):
            print(f"Goal point {(goal_x*10, (config.WINDOW_HEIGHT-goal_y)*10)} (in mm) reached!")
            # Backtracking
            current_node = (curnt_x, curnt_y, curnt_theta)
            # Keep backtracking until start node reached
            final_path, step_info = _reconstruct_path(parent_node_map, step_info_map, current_node)

            print('Path found!')
            print(f'size of path deque: {len(final_path)}')  # only for trouble-shotting
            # print(f'size of step_info deque: {len(step_info)}')  # only for trouble-shotting
            # print(f'size of parent_node dict: {len(parent_node_map)} \n') # only for trouble-shotting
            # print(f'size of step_info_map dict: {len(step_info_map)} \n') # only for trouble-shotting
            return {
            'path': final_path,
            'step_info': step_info,
            'visited_nodes': list(parent_node_map.keys()),
            'runtime': time.time() - a_star_strt_time,
            'success': True,
            'message': "Path found!"
                    } 

        # Explore neighbors with 5 possible actions
        for act in actions:
        # for nx, ny, n_theta, n_c2c in (move_0_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_30_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_60_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_neg30_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c), move_neg60_deg(curnt_x, curnt_y, curnt_theta, curnt_c2c)):
            nx, ny, n_theta, n_c2c, n_dist, n_vel, n_theta_dot = _move(curnt_x, curnt_y, curnt_theta, curnt_c2c, act)
            # Ignore node if it's in the obstacle space.
            if (nx,ny) in obstacles.OBSTACLE_POINTS:
                continue
            # Checks that the new node has not already been visited
            if (_round_near_point5(nx)/threshold_x, _round_near_point5(ny)/threshold_y, n_theta/threshold_theta) not in visited_matrix_index_set:
                # only add node to open list and other dictionaries, if the new node is not in the lowest_cost_dictionary or if the new node's c2c is lower than the one in the lowest_cost dictionary. 
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
    'visited_nodes': list(parent_node_map.keys()),
    'runtime': time.time() - a_star_strt_time,
    'success': False,
    'message': "No Path found!"
            } 

def _reconstruct_path(parent_map: dict, step_info_map: dict, current: Tuple) -> Deque:
    """ 
    backtracks to create a path from the current node back to the initial node. 
    Returns the final path.
    
    Args:
        current (tuple): x,y coordinate of final point
        parent_node_map (dict): nodes and their parent nodes

    Returns:
        path (deque): deque containing x,y of points on the final path.
        step_info (deque): deque containing dictionaries with linear velocity, turning speed & distance 
    """
    path = deque([current])
    step_info = deque([step_info_map[current]])
    while current in parent_map:
        current = parent_map[current]
        if current is None:
            break
        path.appendleft(current)
        step_info.appendleft(step_info_map[current])
    return path, step_info

def _normalize_angle(angle):
    """ normalizes all angles to be with respect to 360 degrees"""
    return angle % 360

def _euclidean_distance(point1_x, point1_y, point2_x, point2_y):
    """ calculates the euclidean distance between two points.Used for c2g calculation """
    return round(math.sqrt((point2_y - point1_y)**2 + (point2_x - point1_x)**2), 2)

def _round_near_point5(number):
    """ rounds number to the nearest 0.5 decimal. If the number is close to 0.5, it rounds it to that, otherwise it rounds it to the closest whole number"""
    return round(number*2)/2

def _move(Xi,Yi,Thetai,cost_to_come,wheel_rpms):
    """ Calculates the velocity, anguar speed, distance covered and new c2c. """
    ang_spd_l, ang_spd_r = wheel_rpms
    t = 0
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = math.radians(Thetai)
    ang_spd_l = ang_spd_l*2*math.pi/60 # convert rpm to rad/s
    ang_spd_r = ang_spd_r*2*math.pi/60 # convert rpm to rad/s

    x_dot = 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.cos(Thetan) 
    y_dot = 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.sin(Thetan)
    theta_dot = (config.WHEEL_RADIUS/config.WHEEL_SEPARATION) * (ang_spd_r - ang_spd_l)
    vel = math.sqrt(x_dot**2 + y_dot**2)
    distance=0
    while t<1:
        t = t + dt
        Xn += 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.cos(Thetan) * dt
        Yn += 0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.sin(Thetan) * dt
        # Thetan += (r / L) * (ang_spd_r - ang_spd_l) * dt
        Thetan += (config.WHEEL_RADIUS/config.WHEEL_SEPARATION) * (ang_spd_r - ang_spd_l) * dt
        distance += math.sqrt((0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.cos(Thetan) * dt)**2 \
                      + (0.5*config.WHEEL_RADIUS * (ang_spd_l + ang_spd_r) * math.sin(Thetan) * dt)**2)
    new_c2c = cost_to_come + distance  
    Thetan = math.degrees(Thetan)
    return round(Xn), round(Yn), round(_normalize_angle(Thetan)), round(new_c2c,1), round(distance,2), round(vel,4), round(theta_dot,4)