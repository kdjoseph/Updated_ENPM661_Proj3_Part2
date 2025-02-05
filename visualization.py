import math
import pygame
import config
import obstacles

def draw_environment(window):
    """ Draws the obstacles and background. Takes the surface display as input """
    window.fill(config.BACKGROUND_COLOR)
    #bloat around rectangular obstacles. pygame.draw.rect(surface, colour, rectangleObject)
    # Draw first blaoted rectangle
    pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR, 
                pygame.Rect(config.LEFT_RECTANGLE['bloated']['x'],
                            config.LEFT_RECTANGLE['bloated']['y'],
                            config.LEFT_RECTANGLE['bloated']['width'],
                            config.LEFT_RECTANGLE['bloated']['height']))
    # Draw second bloated rectangle
    pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR, 
                pygame.Rect(config.MIDDLE_RECTANGLE['bloated']['x'],
                            config.MIDDLE_RECTANGLE['bloated']['y'],
                            config.MIDDLE_RECTANGLE['bloated']['width'],
                            config.MIDDLE_RECTANGLE['bloated']['height']))
    # Draw Bloated Border
    for rect in config.BLOATED_BORDERS.values():
        pygame.draw.rect(window, config.BLOATED_OBSTACLE_COLOR,
                         pygame.Rect(rect['x'], rect['y'], rect['width'], rect['height']))
    # Draw bloated circle obstacle
    #pygame.draw.circle(surface, colour, center, radius, width) # width is optional
    pygame.draw.circle(window, config.BLOATED_OBSTACLE_COLOR,
                       (config.LEFT_CIRCLE['center']['x'],
                        config.LEFT_CIRCLE['center']['y']),
                        config.LEFT_CIRCLE['bloated_radius'])

    # Actual obstacles
    pygame.draw.rect(window, config.OBSTACLE_COLOR, 
                pygame.Rect(config.LEFT_RECTANGLE['x'],
                            config.LEFT_RECTANGLE['y'],
                            config.LEFT_RECTANGLE['width'],
                            config.LEFT_RECTANGLE['height']))
    # 2nd bloated rect obstacle on left
    pygame.draw.rect(window, config.OBSTACLE_COLOR, 
                pygame.Rect(config.MIDDLE_RECTANGLE['x'],
                            config.MIDDLE_RECTANGLE['y'],
                            config.MIDDLE_RECTANGLE['width'],
                            config.MIDDLE_RECTANGLE['height']))
    # Circle
    pygame.draw.circle(window, config.OBSTACLE_COLOR,
                    (config.LEFT_CIRCLE['center']['x'],
                    config.LEFT_CIRCLE['center']['y']),
                    config.LEFT_CIRCLE['radius'])
    pygame.display.update()
    
animation_visited_matrix_idx_set = set() # set to keep track of visted nodes during animation

def draw_action_curve(surface, Xi, Yi, Thetai, UL, UR, color):
    """ Draws the action curve of the robot at each node. 
    Takes as input: the node's x,y,theta, the left & right wheel rpm, and color.
    Returns False is the action would lead to an obstacle point or an already visted point. Returns True otherwise"""

    t = 0
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = math.radians(Thetai)
    UL = UL * 2 * math.pi / 60  # convert rpm to rad/s
    UR = UR * 2 * math.pi / 60  # convert rpm to rad/s
    points = []
    while t < 1:
        t = t + dt
        Xn += 0.5 * config.WHEEL_RADIUS * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * config.WHEEL_RADIUS * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (config.WHEEL_RADIUS / config.WHEEL_SEPARATION) * (UR - UL) * dt
        if ((round(Xn), round(Yn)) in obstacles.OBSTACLE_POINTS) \
            or (round(Xn)/config.THRESHOLD_X, round(Yn)/config.THRESHOLD_Y, round(Thetan)/config.THRESHOLD_THETA) in animation_visited_matrix_idx_set:
            return False
        points.append((int(Xn), int(Yn)))
    animation_visited_matrix_idx_set.add((round(Xn)/config.THRESHOLD_X, round(Yn)/config.THRESHOLD_Y, round(Thetan)/config.THRESHOLD_THETA))
    pygame.draw.lines(surface, color, False, points, 1)  # Draw on the curve surface
    return True

def animate_optimal_path(window, path):
    """Function to animate moving from start to goal using the optimal path, drawing lines between nodes."""
    # Convert nodes to int and also to (x,y) tuples and putting them in a list to use pygame.draw.lines
    path_points = [(int(node[0]), int(node[1])) for node in path]
    for node in path_points:
        pygame.draw.circle(window, config.PATH_POINTS_COLOR, (node[0], node[1]), 3)
        pygame.display.update()
        pygame.time.delay(100)
    pygame.draw.lines(window, config.PATH_COLOR, False, path_points, 1)  # Draw lines connecting each point in path
    pygame.display.update()
    pygame.time.delay(10)  # delay as needed for animation speed