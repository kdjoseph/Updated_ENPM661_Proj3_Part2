#!/usr/bin/env python3
import pygame, sys, math, time
import heapq as hq
from collections import deque
import config
import obstacles
import pathsearch
import visualization

def start_goal_rpm_inputs(gazebo_coord_sw=0, strt_and_rpm_input_sw=1):
    start_pt_trigger = 1
    goal_pt_trigger =1
    wheel_rpm_trigger =1
    if strt_and_rpm_input_sw ==1:
        while start_pt_trigger ==1:
            try:
                startb_x = float(input('Enter starting point x-cordinate in mm: '))/10
                startb_y = float(input('Enter starting point y-cordinate in mm: '))/10
                startb_theta = float(input('Enter starting theta in degrees: '))
                # worst case start wrt to coord at bottom-left corner: start (6,6), goal (1194, 162) or goal (1194, 338)
            except:
                print('you did not enter a number, please enter only numbers')
                continue
            start_theta = startb_theta % 360
            if gazebo_coord_sw ==0:
                start_x = startb_x
                start_y = config.WINDOW_HEIGHT - startb_y # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
            elif gazebo_coord_sw ==1: 
                start_x = startb_x + 500/10 # convert gazebo-x into pygame-x 
                start_y = 1000/10 - startb_y  # convert gazebo-y into pygame-y

            if (start_x, start_y) in obstacles.OBSTACLE_POINTS:
                print('The chosen start point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            else:
                start_pt_trigger = 0

    while goal_pt_trigger == 1:
        try:
            goalb_x, goalb_y = float(input('Enter goal point x- cordinate in mm: '))/10, float(input('Enter goal point y-cordinate in mm: '))/10
        except:
            print('you did not enter a number, please enter only numbers')
            continue
        if gazebo_coord_sw == 0:
            goal_x = goalb_x
            goal_y = config.WINDOW_HEIGHT - goalb_y  # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
        elif gazebo_coord_sw ==1:
            goal_x = goalb_x + 500/10 # convert gazebo-x into pygame-x
            goal_y = -goalb_y +1000/10 # convert gazebo-y into pygame-y
        if (goal_x, goal_y) in obstacles.OBSTACLE_POINTS:
            print('The chosen goal point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            continue
        if (start_x,start_y) == (goal_x, goal_y):
            print('you chose the same starting and goal points, choose different starting and goal points')
        else:
            goal_pt_trigger = 0

    if strt_and_rpm_input_sw ==1:
        while wheel_rpm_trigger == 1:
            try:
                RPM_L, RPM_R = float(input('Enter an rpm less than 76 for the left-wheel: ')), float(input('Enter an rpm less than 76 for the right-wheel: '))
            except:
                print('you did not enter a number, please enter only numbers')
                continue
            if (RPM_L <0 or RPM_L >75) or (RPM_R <0 or RPM_R >75):
                print('The value you entered is outside the acceptable range (0 to 75), try again')
                continue
            wheel_rpm_trigger = 0
    return (start_x, start_y, start_theta), (goal_x, goal_y), (RPM_L, RPM_R)

def main():
# def main():
    """ Main function to start and animate the algorithm search. Then start the ROS publisher node"""
    start, goal, wheel_rpm = start_goal_rpm_inputs()
    goal_x, goal_y = goal
    RPM_L, RPM_R = wheel_rpm
    actions = ((0,RPM_L), (RPM_L,0), (RPM_L,RPM_L), (0,RPM_R),\
            (RPM_R,0), (RPM_R,RPM_R), (RPM_L,RPM_R), (RPM_R,RPM_L))
    
    # a_star_duration = A_star(1, 1, 0)
    result = pathsearch.find_path(start, goal, wheel_rpm, actions)

    print(f"A* Algorithm Execution Time: {result['runtime']} seconds")

    # if optimal path is found, create animation
    if result['message'] == "Path found!":
        animation_strt_time = time.time()
        # animation_run_flag = True
        # Create Pygame window
        pygame.init()
        WINDOW = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
        pygame.display.set_caption("Robot Path Animation with A* ")
        visualization.draw_environment(WINDOW)
        pygame.draw.circle(WINDOW, config.PATH_COLOR, (int(goal_x), int(goal_y)), int(round(1.5)))
        
        nodes_list = result['visited_nodes'][:-1] # slicing to avoid drawing robot motion for the last node, since after it the goal is found
        if 0<len(nodes_list)<15000:
            nodes_per_frame = int(60000/5)
            dly = 2
        elif 15000<len(nodes_list)<30000:
            nodes_per_frame = int(60000/4)
            dly = 1
        elif 30000< len(nodes_list) < 45000:
            nodes_per_frame  = int(60000/2)
            dly =1
        elif 45000< len(nodes_list) < 60000: 
            nodes_per_frame = 45000
            dly = 0
        if len(nodes_list) >= 60000:
            nodes_per_frame = 60000
            dly = 0
        # Loop to draw the action from several nodes in increments.    
        for i in range(0, len(nodes_list), nodes_per_frame):
            for node in nodes_list[i:i+nodes_per_frame]:
                for action in actions:
                    # if draw_action_curve(curve_surface, node[0], node[1], node[2], action[0], action[1], GREEN):
                    if visualization.draw_action_curve(WINDOW, node[0], node[1], node[2], action[0], action[1], config.NODES_COLOR):
                        # WINDOW.blit(curve_surface, (0, 0))  # Blit the curve surface onto the main window
                        pygame.display.update()
                        pygame.time.delay(dly)  # Delay between each curve

        visualization.animate_optimal_path(WINDOW, result['path'])
        animation_end_time = time.time()
        animation_run_time = animation_end_time - animation_strt_time
        print(f"Animation Execution Time: {animation_run_time} seconds, \n")
        print(f"Total execution time of search algorithm & animation {result['runtime']+animation_run_time} seconds")
        
        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            clock.tick(config.FPS)

    else:
        print('The algorithm could not find an optimal path')

if __name__ == "__main__":
    main()