#!/usr/bin/env python3
import pygame, sys, math, time
import heapq as hq
from collections import deque
import config
import obstacles
import pathsearch
import visualization

def start_goal_rpm_inputs(gazebo_coord_sw=0, strt_and_rpm_input_sw=1):
    """Ask the user for the start and goal point, and for the left and right wheel RPM.
    An optional gazebo_coord_sw allows for the start and goal points to be converted to the
    coordinates of the gazebo world, for future ros2 gazebo implementation
    """
    start_pt_trigger = 1
    goal_pt_trigger =1
    wheel_rpm_trigger =1
    if strt_and_rpm_input_sw ==1:
        while start_pt_trigger ==1:
            try:
                startb_x = float(input('Enter starting point x-cordinate in mm: '))/10
                startb_y = float(input('Enter starting point y-cordinate in mm: '))/10
                start_theta = float(input('Enter starting theta in degrees: '))%360
            except ValueError:
                print('you did not enter a number, please enter only numbers')
                continue
            if gazebo_coord_sw ==0:
                start_x = startb_x
                # convert from coordinate w.r.t lower-left corner of display to upper left-corner coordinate
                start_y = config.WINDOW_HEIGHT - startb_y 
            elif gazebo_coord_sw ==1: 
                start_x = startb_x + 500/10 # convert gazebo-x into pygame-x 
                start_y = 1000/10 - startb_y  # convert gazebo-y into pygame-y

            if obstacles.is_collision(start_x, start_y):
                print('The chosen start point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            else:
                start_pt_trigger = 0

    while goal_pt_trigger == 1:
        try:
            goalb_x = float(input('Enter goal point x- cordinate in mm: '))/10
            goalb_y = float(input('Enter goal point y-cordinate in mm: '))/10
        except ValueError:
            print('you did not enter a number, please enter only numbers')
            continue
        if gazebo_coord_sw == 0:
            goal_x = goalb_x
            # convert from coordinate wrt to lower-left corner of display to upper left-corner coordinate
            goal_y = config.WINDOW_HEIGHT - goalb_y  
        elif gazebo_coord_sw ==1:
            goal_x = goalb_x + 500/10 # convert gazebo-x into pygame-x
            goal_y = -goalb_y +1000/10 # convert gazebo-y into pygame-y
        if obstacles.is_collision(goal_x, goal_y):
            print('The chosen goal point is in the obstacle space or too close to the border or out of the display dimensions, choose another one.')
            continue
        if (start_x,start_y) == (goal_x, goal_y):
            print('you chose the same starting and goal points, choose different starting and goal points')
        else:
            goal_pt_trigger = 0

    if strt_and_rpm_input_sw ==1:
        while wheel_rpm_trigger == 1:
            try:
                RPM_L = float(input('Enter an rpm less than 76 for the left-wheel: '))
                RPM_R = float(input('Enter an rpm less than 76 for the right-wheel: '))
            except ValueError:
                print('you did not enter a number, please enter only numbers')
                continue
            if (RPM_L <0 or RPM_L >75) or (RPM_R <0 or RPM_R >75):
                print('The value you entered is outside the acceptable range (0 to 75), try again')
                continue
            wheel_rpm_trigger = 0
    return (start_x, start_y, start_theta), (goal_x, goal_y), (RPM_L, RPM_R)

def main():
    """ Main function to start and animate the algorithm search. Then start the ROS publisher node"""
    start, goal, wheel_rpm = start_goal_rpm_inputs()
    goal_x, goal_y = goal
    RPM_L, RPM_R = wheel_rpm
    actions = ((0,RPM_L), (RPM_L,0), (RPM_L,RPM_L), (0,RPM_R),\
            (RPM_R,0), (RPM_R,RPM_R), (RPM_L,RPM_R), (RPM_R,RPM_L)) # tuple of 8 posible actions
    
    # a_star_duration = A_star(1, 1, 0)
    result = pathsearch.find_path(start, goal, actions)

    print(f"A* Algorithm Execution Time: {result['runtime']:.2f} seconds")

    # if optimal path is found, create animation
    if result['message'] == "Path found!":
        animation_start_time = time.time()

        # Create Pygame window
        pygame.init()
        WINDOW = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
        pygame.display.set_caption("Robot Path Animation with A* ")
        visualization.draw_environment(WINDOW)
        pygame.draw.circle(WINDOW, config.PATH_COLOR, (int(goal_x), int(goal_y)), int(round(1.5)))
        nodes_list = result['visited_nodes']
        #set delay between each frame based on # of nodes to be drawn
        if 0<len(nodes_list)<15000:
            nodes_per_frame = int(60000/5)
            delay = 2
        elif 15000<len(nodes_list)<30000:
            nodes_per_frame = int(60000/4)
            delay = 1
        elif 30000< len(nodes_list) < 45000:
            nodes_per_frame  = int(60000/2)
            delay =1
        elif 45000< len(nodes_list) < 60000: 
            nodes_per_frame = 45000
            delay = 0
        if len(nodes_list) >= 60000:
            nodes_per_frame = 60000
            delay = 0

        # Loop to draw the action curves from several nodes in increments.    
        for i in range(0, len(nodes_list), nodes_per_frame):
            for node in nodes_list[i:i+nodes_per_frame]:
                action_curve_drawn_count = 0

                for action in actions:
                    if visualization.draw_action_curve(
                        WINDOW,
                        node[0], node[1], node[2],
                        action[0], action[1],
                        config.NODES_COLOR) is True:

                        action_curve_drawn_count += 1
                # Only update display after drawing the set of 8 actions has been attempted
                if action_curve_drawn_count > 0:
                    pygame.display.update()
                    pygame.time.delay(delay)  # Delay between each action curve set
         
        visualization.animate_optimal_path(WINDOW, result['path'])
        animation_run_time = time.time() - animation_start_time
        print(f"\nAnimation Execution Time: {animation_run_time:.2f} seconds.")
        print(f"Total execution time of search algorithm & animation {result['runtime']+animation_run_time} seconds")
        
        clock = pygame.time.Clock()
        while True:  # main loop to control animation & close window
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            clock.tick(config.FPS)

    else:
        print('The algorithm could not find an optimal path')

if __name__ == "__main__":
    main()