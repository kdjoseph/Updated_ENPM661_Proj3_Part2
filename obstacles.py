import config

def is_collision(x, y):
    """ Checks if a point is in the obstacle space. Returns True if the point is in the obstacle space, and False if it is not"""
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
    # Border region outside all obstacles
    if (x <=config.BLOAT or x >= (config.WINDOW_WIDTH-config.BLOAT)) or (y <=config.BLOAT or y >= (config.WINDOW_HEIGHT-config.BLOAT)):
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
    
        