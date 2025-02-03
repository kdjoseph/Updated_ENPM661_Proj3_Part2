ROBOT_RADIUS = (220)/10 # 220 mm robot radius converted to cm
WHEEL_RADIUS = (66/2)/10 # wheel diameter = 66 mm converted to cm
WHEEL_DISTANCE = (287)/10 # 287 mm converted to cm, distance from a line through the middle of one wheel to another line through the middle of the other wheel

WINDOW_WIDTH, WINDOW_HEIGHT = 6000/10, 2000/10 # convert to cm
START_X, START_Y = 500/10, WINDOW_HEIGHT/2
GOAL_X, GOAL_Y = WINDOW_WIDTH-250/10, START_Y
CLEARANCE = 10
BLOAT = ROBOT_RADIUS + CLEARANCE

# OBSTACLE 1 RECTANGLE 1 for the left side of the layout
# rect1_l, rect1_w = 1000/10, 250/10
# b_rect1_l = rect1_l + BLOAT  #BLOATed rectangle1 length
# b_rect1_w = rect1_w + 2*BLOAT  # BLOATed rectangle1 width
# b_rect1_x = (500+1000)/10-BLOAT     #x-coordinate of rect corner point

LEFT_RECTANGLE = {'LENGTH': 1000/10,
                  'WIDTH': 250/10,
                  'X': START_X + 1000/10,
                  'Y': 0,
                  'BLOATED': {
                      'LENGTH': 1000/10 + BLOAT,
                      'WIDTH': 250/10 + 2*BLOAT,
                      'X': START_X+1000/10 - BLOAT,
                      'Y': 0
                  },
                  }

MIDDLE_RECTANGLE = {'LENGTH': LEFT_RECTANGLE['LENGTH'],
                  'WIDTH': LEFT_RECTANGLE['WIDTH'],
                  'X': START_X + 2000/10,
                  'Y': WINDOW_HEIGHT/2,
                  'BLOATED': {
                      'LENGTH': LEFT_RECTANGLE['BLOATED']['LENGTH'],
                      'WIDTH': LEFT_RECTANGLE['BLOATED']['WIDTH'],
                      'X': START_X + 2000/10 - BLOAT,
                      'Y': WINDOW_HEIGHT/2 - BLOAT
                  },
                  }

LEFT_CIRCLE = {'CENTER': {
    'X': START_X + 3700/10,
    'Y': 800/10},
    'RADIUS': (1200/2)/10,
'BLOATED_RADIUS': BLOAT + (1200/2)/10
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