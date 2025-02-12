#!/usr/bin/env python3
""" Module for creating a function and a set to check if points are in the obstacle space"""

import numpy as np
import config

def is_collision(x, y):
    """ Checks if a point is in the obstacle space. 
    Returns True if the point is in the obstacle space, and False if it is not"""

    # Bloat around border walls
    if (x <=config.BLOAT or x >= (config.WINDOW_WIDTH-config.BLOAT)) or (y <=config.BLOAT or y >= (config.WINDOW_HEIGHT-config.BLOAT)):
        return True
    
    # First rectangular obstacles
    if ((config.LEFT_RECTANGLE['bloated']['x'] <= x <=(config.LEFT_RECTANGLE['bloated']['x']+
                                                       config.LEFT_RECTANGLE['bloated']['width']))\
        and y <= config.LEFT_RECTANGLE['bloated']['height']):
        return True
    
    # 2nd Rectangle
    if config.MIDDLE_RECTANGLE['bloated']['x'] <=x <=(config.MIDDLE_RECTANGLE['bloated']['x'] +
                                                        config.MIDDLE_RECTANGLE['bloated']['width'])\
                and (y >= config.MIDDLE_RECTANGLE['bloated']['y']):
        return True
    # Circle Obstacle
    # general equation of circle: (x-center_x)^2 + (y-center_y)^2 = radius^2
    if (x-config.LEFT_CIRCLE['center']['x'])**2+(y-config.LEFT_CIRCLE['center']['y'])**2<=\
        config.LEFT_CIRCLE['bloated_radius']**2:
        return True

    else:               # not in obstacle space
        return False
    
# Make an array of all the points in the layout at 0.5 spacing.
x_coords = np.arange(0, config.WINDOW_WIDTH, 0.5)
y_coords = np.arange(0, config.WINDOW_HEIGHT, 0.5)

# # Sweep accross all points in the layout, with a 0.5 interval, then add the points that are in the obstacle space in a set.
OBSTACLE_POINTS = {(x, y) for x in x_coords for y in y_coords if is_collision(x, y)}

    
        