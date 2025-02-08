ROBOT_RADIUS = (220)/10 # 220 mm robot radius converted to cm
WHEEL_RADIUS = (66/2)/10 # wheel diameter = 66 mm converted to cm
WHEEL_SEPARATION = (287)/10 # 287 mm converted to cm, distance from a line through the middle of one wheel to another line through the middle of the other wheel

WINDOW_WIDTH, WINDOW_HEIGHT = 6000/10, 2000/10 # convert to cm
GAZEBO_START_X = 500/10 # 500 mm converted to cm
CLEARANCE = 29/10 # 100 mm converted to cm = 10 cm
BLOAT = ROBOT_RADIUS + CLEARANCE

THRESHOLD_X, THRESHOLD_Y, THRESHOLD_THETA = 0.5, 0.5, 30 # cm, cm, degrees

LEFT_RECTANGLE = {'height': 1000/10,
                  'width': 250/10,
                  'x': GAZEBO_START_X + 1000/10,
                  'y': 0,
                  'bloated': {
                      'height': 1000/10 + BLOAT,
                      'width': 250/10 + 2*BLOAT,
                      'x': GAZEBO_START_X+1000/10 - BLOAT,
                      'y': 0
                  },
                  }

MIDDLE_RECTANGLE = {'height': LEFT_RECTANGLE['height'],
                  'width': LEFT_RECTANGLE['width'],
                  'x': GAZEBO_START_X + 2000/10,
                  'y': WINDOW_HEIGHT/2,
                  'bloated': {
                      'height': LEFT_RECTANGLE['bloated']['height'],
                      'width': LEFT_RECTANGLE['bloated']['width'],
                      'x': GAZEBO_START_X + 2000/10 - BLOAT,
                      'y': WINDOW_HEIGHT/2 - BLOAT
                  },
                  }

LEFT_CIRCLE = {'center': {
    'x': GAZEBO_START_X + 3700/10,
    'y': 800/10},
    'radius': (1200/2)/10,
'bloated_radius': BLOAT + (1200/2)/10
}

# Bloated Border Rectangles (only for visualization)
BLOATED_BORDERS = {
    'left_vertical': {
        'x': 0,
        'y': 0,
        'height': WINDOW_HEIGHT,
        'width': BLOAT
    },
    'right_vertical': {
        'x': WINDOW_WIDTH-BLOAT,
        'y': 0,
        'height': WINDOW_HEIGHT,
        'width': BLOAT
    },
    'top_horizontal': {
        'x': 0,
        'y': 0,
        'height': BLOAT,
        'width': WINDOW_WIDTH
    },
    'bottom_horizontal': {
        'x': 0,
        'y': WINDOW_HEIGHT-BLOAT,
        'height': BLOAT,
        'width': WINDOW_WIDTH
    } 
}

# Colors (RGB tuples)
BACKGROUND_COLOR = (0, 40, 255) # blue
OBSTACLE_COLOR = (255, 30, 70) # red
BLOATED_OBSTACLE_COLOR = (255, 255, 0) # yellow
NODES_COLOR = (0, 100, 0) # green
PATH_POINTS_COLOR = (0, 0, 0) # black
PATH_COLOR = (255, 255, 255) # white
FPS: int = 60