# tallysman_ros2
This repository contains the ROS2 package for integrating Tallysman GNSS and GNRMC receivers with ROS2-based systems.

ROS2 Driver

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)


## Introduction

Tallysman ROS2 is a ROS2 package that provides functionality for interfacing with Tallysman GNSS receivers. It allows you to receive and process GNSS data within your ROS2-based systems. This package provides ROS 2 nodes, drivers, and utilities to interact with Tallysman GNSS receivers, enabling accurate localization, navigation, and time synchronization for your robotic projects.


### Features

- Real-Time Positioning: Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- ROS 2 Integration: Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS-based applications.
- Customizable Configuration: Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- Monitoring Tool: Monitor you robot's location a map view.

## Installation

To install Tallysman ROS2, follow these steps:

1. Workspace: Move to your workspace
2. Clone the repository: 
  'git clone https://github.com/indro-robotics/tallysman_ros2.git'
3. Build the package using colcon: 
  'cd ~/ros_ws'
  'colcon build'
4. 3. Source the setup file:
  'source install/setup.bash'


## Usage

To use Tallysman ROS2 in your ROS2-based system, follow these steps:

1. Launch the Tallysman ROS2 node:
   ```
   ros2 run tallysman_ros2 tallysman_gps
   ```
3. 



  



  

