# Robotics-AUT

Welcome to the Robotics-AUT repository! Here, you'll find code and documentation for exercises and projects related to the Introduction to Robotics course at AUT (Amirkabir University of Technology). This repository is a comprehensive resource for learning and implementing robotics concepts using ROS (Robot Operating System) and Gazebo simulation.

## Exercise One: ROS and Gazebo

Exercise One introduces you to ROS and Gazebo with a focus on fundamental concepts and practical implementation. This exercise includes tasks such as:

- Creating a ROS package with multiple nodes.
- Moving the TurtleBot in the Gazebo simulation world.
- Reporting robot navigation information.
- Making the TurtleBot follow a square path.
- Measuring and minimizing deviation from the expected path.
- Visualizing robot paths and discussing the results.

## Exercise Two: PI Controller

Exercise Two challenges you to design a PI (Proportional-Integral) controller within the ROS/Gazebo environment. The goal is to enable a robot to follow predefined parametric paths, including ellipses and Archimedean spirals. Key tasks include:

- Precisely explaining the PI controller design.
- Detailing controller coefficients and parameters.
- Determining appropriate P and I coefficients.
- Analyzing the effects of changing P and I coefficients.
- Calculating and displaying tracking error for clarity.

## ROS Maze Navigation Project

The ROS Maze Navigation Project is a major undertaking in the course, combining ROS and Gazebo for autonomous maze navigation. Here's what the project entails:

- Developing a reactive robot controller for maze navigation.
- Utilizing on-board sensors, including depth cameras, laser scans, bumpers, and cliff sensors.
- Configuring the maze environment using the provided "funky-maze.world" file.
- Creating a ROS package with two nodes: "velocity-controller" and "obstacle-detector."
- Safely and effectively navigating the maze while avoiding obstacles.
- Running the robot for 15 minutes and reporting performance metrics.
- Evaluating controller performance, identifying issues, and proposing improvements.


# Exercise One: ROS and Gazebo

The first exercise is quite straightforward, but it requires a thorough study and complete understanding of the "start-with-ros.pdf" article, which has been uploaded as a supplementary file in the first exercise section of the course system.

In this exercise, you are required to precisely implement the package described in the above-mentioned article and test various ROS commands.

Then, proceed to the main tasks of this exercise to assess your fundamental knowledge of ROS. These tasks include:

• Create a package with at least two nodes. One of the nodes should be responsible for moving the turtlebot in the Gazebo simulation world. The other node should continuously report robot navigation information (including position prediction using robot odometry and groundtruth robot position via Gazebo) and measure the robot's performance as described below.

• Using the system topics and services introduced in the aforementioned article, the two nodes should work together to make the turtlebot follow a square path in the empty.world scenario in Gazebo. The center of the square should be located at (0, 0), and each side should be 3 meters long. The linear speed of the robot along the path should be set to 0.9 meters per second.

• The robot should follow the square path 10 times. At the end of the execution, using a node, measure the deviation between the expected path (in the last round) and what the robot actually achieved. Minimize the error, report it, and discuss your findings.

• Display graphically all the paths traversed by the robot during the 10 rounds on the expected path and discuss the results.


# Exercise Two: PI Controller

The goal of Exercise 2 is to design a PI controller to enable a robot to follow a specified path. In this exercise, you are required to design and implement a closed-loop PI controller in the ROS/Gazebo environment that allows a robot to follow a predefined path. The path is given parametrically, defining a sequence of Cartesian coordinates (x(s), y(s)). The robot must always start moving from a point outside the path.

To achieve this, consider two families of parametric paths: elliptical paths and Archimedean spirals. Your code should be parametric, meaning it should be able to define an ellipse or a spiral (both centered at (0, 0)) based on input. Additionally, the initial position of the robot can be taken as an optional input parameter.

In particular, report your results for an ellipse with a y-axis of 3 and an x-axis of 1 and an Archimedean spiral with a growth factor of 0.1. Use ds = 0.1 for the ellipse and ds = 0.25 for the Archimedean spiral. Place the center of both the ellipse and the spiral at (0, 0), and initiate robot movement from position (2, 1).

- Precisely explain your PI controller and detail all coefficients and parameters.
- Explain which coefficients for P and I provide an appropriate response.
- Discuss the effects of increasing/decreasing P and I coefficients, providing several examples to illustrate.
- Calculate and display the tracking error as desired, making it understandable.

# ROS Maze Navigation Project

## Introduction
The ROS Maze Navigation Project is part of the AUT CE Introduction to Robotics course. The goal of this project is to develop a robot controller using ROS and Gazebo to navigate through a maze-like environment autonomously. The robot must safely and effectively navigate the maze, avoiding obstacles and following a predefined path.

## Project Components
### On-board Sensors
Before starting the project, review the documentation in [start-with-ros.pdf](link-to-documentation) to understand the on-board sensors, including depth cameras, laser scans, bumpers, and cliff sensors. This knowledge is crucial for the project.

### Setting Robot Pose
While not strictly necessary, you can refer to Section 5 of the same document to learn about setting the initial robot pose during simulation experiments.

## Maze Environment
The maze-like environment is described in the [funky-maze.world](link-to-maze-world) file in SDF format. This file defines the maze layout. To set up the environment, follow these steps:
1. Create a "worlds" folder in your catkin workspace.
2. Place the "funky-maze.world" file in the "worlds" folder.
3. Launch the Gazebo simulation using the provided command.

## Project Tasks 
Design a reactive robot controller to navigate through the maze. The robot has no prior knowledge of the maze's structure and must rely solely on sensor data for navigation. The robot's behavior should consist of two basic sub-behaviors:
- Move straight when there are no obstacles in front.
- Make turns to avoid colliding with obstacles.

The robot's goal is to keep making tours within the maze. Strong deviations from the intended path could lead to difficulties in navigation.

## Requirements
- Create a new ROS package with at least two nodes: "velocity-controller" and "obstacle-detector."
- The "velocity-controller" node controls the robot's motion by setting velocity commands.
- The "obstacle-detector" node subscribes to laser scan and bumper sensor topics to build an instantaneous obstacle map based on sensor data.
- Ensure safe and effective navigation through the maze.
- Quantify your achievements by running the robot for 15 minutes in the maze and report specific metrics (e.g., number of passes near the origin, bumper triggers, average speed).

## Report and Discussion
- Evaluate the performance of your controller.
- Identify any issues encountered during navigation.
- Propose improvements for enhancing the controller's navigation performance.

## Extra Credit
Design a deliberative robot controller that explores the entire maze and reports its performance. This controller should incorporate sensing, localization, map generation, planning, and action.

![Watch the Project Gif](https://github.com/mhbadiei/Robotics/blob/main/gif.gif)

Explore our project in action by watching the full version video.

[Watch the Project Video](https://github.com/mhbadiei/Robotics/blob/main/Project/video.mp4)
