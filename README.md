The Code is an update to the Project 3 Part 2 code that was made for the ENPM 661 class assignment in Spring 2024. The code simulates the motions of a TurtleBot3 Waffle Pi robot, which has a differential drive, using the A-star algorithm to find the optimal point from a starting point to a goal point. This code only performs the A-star search and pygame animations; the ROS2 Gazebo simulation will be added later. Five modules make up the overall code:
	1) a_star_main.py: prompts the user to enter values, then starts the a_star_search and the animations
	2) config.py: contains key constants, such as the robot's specs, the obstacle dimensions, and others.
	3) obstacles.py: contains a function to check if a point is in the obstacle space, and a set with all the points in the obstacle space at 0.5 cm intervall
	4) pathsearch.py: contains the a_star algorithm
	5) visualization.py: contains the pygame functions to display and create the animation of the search.

Before running the code ensure that the dependencies are installed. To run the code, download all the modules into one folder, then open the a_star_main.py module and run it. Input prompts will appear asking for the starting and goal points, and the angular speed for each wheel in rpm. Then the search for the optimal path will begin. Once an optimal path is found, a pygame window will appear to show the animation. 


DEPENDENCIES
	- Software Packages:
		- Python 3.8 or later
	-Libraries:
		- numba
		- numpy
		- pygame
