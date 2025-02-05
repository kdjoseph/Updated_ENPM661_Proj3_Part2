ROBOT_RADIUS = (220)/10 # 220 mm robot radius converted to cm
WHEEL_RADIUS = (66/2)/10 # wheel diameter = 66 mm converted to cm
WHEEL_SEPARATION = (287)/10 # 287 mm converted to cm, distance from a line through the middle of one wheel to another line through the middle of the other wheel

WINDOW_WIDTH, WINDOW_HEIGHT = 6000/10, 2000/10 # convert to cm
START_X, START_Y = 500/10, WINDOW_HEIGHT/2
GOAL_X, GOAL_Y = WINDOW_WIDTH-250/10, START_Y
CLEARANCE = 20
BLOAT = ROBOT_RADIUS + CLEARANCE


THRESHOLD_X, THRESHOLD_Y, THRESHOLD_THETA = 0.5, 0.5, 30

# OBSTACLE 1 RECTANGLE 1 for the left side of the layout
# rect1_l, rect1_w = 1000/10, 250/10
# b_rect1_l = rect1_l + BLOAT  #BLOATed rectangle1 length
# b_rect1_w = rect1_w + 2*BLOAT  # BLOATed rectangle1 width
# b_rect1_x = (500+1000)/10-BLOAT     #x-coordinate of rect corner point

LEFT_RECTANGLE = {'length': 1000/10,
                  'width': 250/10,
                  'x': START_X + 1000/10,
                  'y': 0,
                  'bloated': {
                      'length': 1000/10 + BLOAT,
                      'width': 250/10 + 2*BLOAT,
                      'x': START_X+1000/10 - BLOAT,
                      'y': 0
                  },
                  }

MIDDLE_RECTANGLE = {'length': LEFT_RECTANGLE['length'],
                  'width': LEFT_RECTANGLE['width'],
                  'x': START_X + 2000/10,
                  'y': WINDOW_HEIGHT/2,
                  'bloated': {
                      'length': LEFT_RECTANGLE['bloated']['length'],
                      'width': LEFT_RECTANGLE['bloated']['width'],
                      'x': START_X + 2000/10 - BLOAT,
                      'y': WINDOW_HEIGHT/2 - BLOAT
                  },
                  }

LEFT_CIRCLE = {'center': {
    'x': START_X + 3700/10,
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
# # OBSTACLE 2: RECTANGLE 2
# rect2_l, rect2_w = 1000/10, 250/10
# b_rect2_l = rect2_l + BLOAT  # BLOATed rectangle 2 length
# b_rect2_w = rect2_w + 2*BLOAT # BLOATed rectangle 2 width
# b_rect2_x = (500 + 2000)/10 - BLOAT #x-coordinate of rect corner point
# b_rect2_y = WINDOW_HEIGHT - b_rect2_l #y-coordinate of rect corner point
# # OBSTACLE3: CIRLCE
# center_x, center_y = (500+3700)/10, 800/10
# circle_r, bltd_circle_r = (1200/2)/10, BLOAT+(1200/2)/10

# Colors (RGB tuples)
BACKGROUND_COLOR = (0, 40, 255) # blue
OBSTACLE_COLOR = (255, 30, 70) # red
BLOATED_OBSTACLE_COLOR = (255, 255, 0) # yellow
NODES_COLOR = (0, 100, 0) # green
PATH_POINTS_COLOR = (0, 0, 0) # black
PATH_COLOR = (255, 255, 255) # white
FPS: int = 60