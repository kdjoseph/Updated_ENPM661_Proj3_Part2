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

def draw_action_curves_batch(surface, nodes, actions, color):
    """Draws action curves for multiple nodes in a batch for better performance"""
    curves_surface = pygame.Surface((config.WINDOW_WIDTH, config.WINDOW_HEIGHT), pygame.SRCALPHA)
    
    for node in nodes:
        Xi, Yi, Thetai = node
        for action in actions:
            UL, UR = action
            points = calculate_curve_points(Xi, Yi, Thetai, UL, UR)
            if points:  # Only draw if points are valid (no collision)
                pygame.draw.lines(curves_surface, color, False, points, 1)
    
    surface.blit(curves_surface, (0, 0))
    return True

def calculate_curve_points(Xi, Yi, Thetai, UL, UR):
    """Calculates points for action curve without drawing - returns None if collision detected"""
    t = 0
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = math.radians(Thetai)
    UL = UL * 2 * math.pi / 60
    UR = UR * 2 * math.pi / 60
    points = []
    
    while t < 1:
        t = t + dt
        Xn += 0.5 * config.WHEEL_RADIUS * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * config.WHEEL_RADIUS * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (config.WHEEL_RADIUS / config.WHEEL_SEPARATION) * (UR - UL) * dt
        
        if (round(Xn), round(Yn)) in obstacles.OBSTACLE_POINTS:
            return None
            
        points.append((int(Xn), int(Yn)))
    
    return points

def animate_optimal_path(window, path):
    """Animates the optimal path with improved performance"""
    path_surface = pygame.Surface((config.WINDOW_WIDTH, config.WINDOW_HEIGHT), pygame.SRCALPHA)
    path_points = [(int(node[0]), int(node[1])) for node in path]
    
    # Draw all points at once
    for node in path_points:
        pygame.draw.circle(path_surface, config.PATH_POINTS_COLOR, node, 3)
    
    # Draw the connecting lines
    if len(path_points) > 1:
        pygame.draw.lines(path_surface, config.PATH_COLOR, False, path_points, 1)
    
    window.blit(path_surface, (0, 0))
    pygame.display.update()