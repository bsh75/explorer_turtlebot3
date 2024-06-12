# Autonomous Exploration

This project is a ROS node for autonomous exploration, designed to orchestrate a robot's exploration process. The node subscribes to the map and odometry topics, uses the `GoalSelector` to identify potential exploration goals, publishes markers in RViz to visualize these goals, and sends goals to the `move_base` action server for navigation. A simple state implementation has also been started.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Node Details](#node-details)
- [Areas for Improvement](#areas-for-improvement)
  - [Goal Selection](#goal-selection)
  - [State Transitions](#state-transitions)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This ROS node implements a frontier-based exploration strategy for autonomous robots. It integrates with the ROS navigation stack and provides a comprehensive solution for autonomous exploration, goal selection, and visualization. This project was developed on a virtual machine running Ubuntu 20.04 with ROS Noetic packages.

## Features

- Subscribes to map and odometry topics.
- Uses a custom `GoalSelector` to identify potential exploration goals.
- Publishes markers in RViz to visualize these goals.
- Sends goals to the `move_base` action server for navigation.
- Monitors robot velocity to detect when the robot is stuck.
- Saves the current map if the robot is idle for too long.
- Handles recovery behaviors when the robot is stuck.

## Installation

1. Ensure you have ROS installed on your system.
2. Clone the repository into your catkin workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/bsh75/explorer_turtlebot3.git
    ```
3. Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
4. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

## Usage

To run the exploration node:
```bash
roslaunch explorer_turtlebot3 exploration_node.launch

## Node Details

**Node Name**: `exploration_node`

**Subscribed Topics**:
- `/map` (`nav_msgs/OccupancyGrid`)
- `/odom` (`nav_msgs/Odometry`)
- `/cmd_vel` (`geometry_msgs/Twist`)

**Published Topics**:
- `move_base` (`move_base_msgs/MoveBaseAction`)

**Parameters**:
- `~robot_type` (default: 'waffle')
- `~config_folder` (default: 'config')

## Areas for Improvement

### Goal Selection

The current goal selection method in the `GoalSelector` class could be significantly improved by implementing a frontier-based approach. A frontier-based method would better identify boundaries between explored and unexplored areas, leading to more efficient and effective exploration.

### State Transitions

State transitions in the current implementation require further development to handle more exceptions, particularly once the robot has found a goal. Improvements in state management would ensure more robust handling of various scenarios the robot may encounter during exploration.

In the interest of time, these enhancements were not implemented but are crucial for achieving a more refined exploration strategy.

### Parameter Tuning

The parameter files in the `params` folder need calibrating and tuning for better performance. Adjustments to parameters such as map resolution, robot footprint, and exploration settings can greatly influence the effectiveness of the exploration strategy.
