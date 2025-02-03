import config

def is_collision(x, y):
    """ Checks if a point is in the obstacle space. Returns True if the point is in the obstacle space, and False if it is not"""
    # First 2 rectangular obstacles
    if ((config.LEFT_RECTANGLE['BLOATED']['X'] <= x <=(config.LEFT_RECTANGLE['BLOATED']['X']+config.LEFT_RECTANGLE['BLOATED']['WIDTH']))\
        and y <= config.LEFT_RECTANGLE['BLOATED']['LENGTH'])\
            or ((config.MIDDLE_RECTANGLE['BLOATED']['X'] <=x <=(config.MIDDLE_RECTANGLE['BLOATED']['X'] + config.MIDDLE_RECTANGLE['BLOATED']['WIDTH']))\
                and y >= config.MIDDLE_RECTANGLE['BLOATED']['Y']):
        return True
    # Circle Obstacle
    # general equation of circle: (x-center_x)^2 + (y-center_y)^2 = radius^2
    elif (x-config.LEFT_CIRCLE['CENTER']['X'])**2+(y-config.LEFT_CIRCLE['CENTER']['X'])**2<=\
        config.LEFT_CIRCLE['BLOATED_RADIUS']**2:
        return True
    # Border region outside all obstacles 
    elif (x <=config.BLOAT or x >= (config.WINDOW_WIDTH-config.BLOAT)) or (y <=config.BLOAT or y >= (config.WINDOW_HEIGHT-config.BLOAT)):
        return True
    else:               # not in obstacle space
        return False
    
def _range_float(start, stop, step):
    """Function to generate a range with float steps, since in-built range() function only allows interger steps"""
    while start <= stop:
        yield start
        start += step

# Sweep accross all points in the layout, with a 0.5 interval, then add the points that are in the obstacle space in a set.
OBSTACLE_POINTS = {(x,y) for x in _range_float(0, config.WINDOW_WIDTH, 0.5) for y in _range_float(0, config.WINDOW_HEIGHT, 0.5) if is_collision(x,y)}
# print(f'number of obstacle points {len(OBSTACLE_POINTS)} \n')   # only for debugging
    
        