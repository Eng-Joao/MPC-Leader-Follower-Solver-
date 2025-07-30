# NMPC Leader-Follower Robot Control

A Nonlinear Model Predictive Control (NMPC) framework for coordinating two differential-drive robots in leader-follower formation.

## Overview

This project implements an NMPC controller that enables a follower robot to maintain a desired relative pose with respect to a moving leader robot. The controller handles nonholonomic constraints and uses a kinematic error model to solve optimal control problems in real-time.

## Key Features

- **Predictive Control**: Leverages NMPC's predictive capabilities for smooth trajectory tracking
- **Constraint Handling**: Manages nonholonomic constraints of differential-drive robots
- **Real-time Performance**: Discrete-time optimization using multi-shooting and quadratic cost functions
- **ROS2/Gazebo Integration**: Full simulation environment with realistic robot dynamics
- **PS4 Controller Support**: Manual control of the leader robot for dynamic scenarios

## Installation

0. Make sure you have ROS2, Gazebo and Casadi correctly installed!

1. Clone this repository to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Eng-Joao/MPC-Leader-Follower-Solver-.git
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

## Usage

1. **Launch the simulation environment:**
```bash
ros2 launch diff_drive_robot 2robots.launch.py
```

2. **Connect PS4 controller (via USB) and launch teleop:**
```bash
ros2 launch p9n_bringup teleop.launch.py topic_name:=/robot2/cmd_vel linear_speed:=1.0 angular_speed:=1.0
```

3. **Run the NMPC controller:**
```bash
ros2 run vehicle_control mpc_controller_error
```

## Results

![Leader-Follower Demo](example.gif)

## Credits

- Two-wheel robot model inspired by: [diff_drive_robot](https://github.com/adoodevv/diff_drive_robot)
- PS4 controller interface forked from: [PlayStation-JoyInterface-ROS2](https://github.com/HarvestX/PlayStation-JoyInterface-ROS2)
