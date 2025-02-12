# TurtleBot3 Waffle Pi Robot A-star Algorithm Simulation

This code simulates the motions of a TurtleBot3 Waffle Pi robot, which has a differential drive, using the A-star algorithm to find the optimal path from a starting point to a goal point. This code performs the A-star search and Pygame animations; the ROS2 Gazebo simulation will be added later.

## Modules

The code consists of five modules:

-   `a_star_main.py`: Prompts the user for input values, then starts the A-star search and animations.
-   `config.py`: Contains key constants, such as the robot's specifications and obstacle dimensions.
-   `obstacles.py`: Contains a function to check if a point is within the obstacle space, and a set of all points in the obstacle space at 0.5 cm intervals.
-   `pathsearch.py`: Contains the A-star algorithm implementation.
-   `visualization.py`: Contains the Pygame functions to display and create the animation of the search.

## Usage

To run the code:

1.  Ensure that the dependencies are installed.
2.  Download all modules into a single folder.
3.  Open the `a_star_main.py` module and run it.
4.  Input prompts will appear, asking for the starting and goal points, and the angular speed for each wheel in RPM.
5.  The search for the optimal path will begin.
6.  Once an optimal path is found, a Pygame window will appear, showing the animation.

## Dependencies

### Software Packages

-   Python 3.8 or later

### Libraries

-   numba
-   numpy
-   pygame

