# tallysman_ros2
This repository contains the ROS2 package for integrating Tallysman GNSS and GNRMC receivers with ROS2-based systems.

ROS2 Driver

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [Reference](#reference)


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
  ```
  git clone https://github.com/indro-robotics/tallysman_ros2.git
  ```
3. Build the package using colcon:
  ```
  cd ~/ros_ws
  colcon build
  ```
4. source the setup file:
  ```
  source install/setup.bash
  ```


## Usage

To use Tallysman ROS2 in your ROS2-based system, follow these steps:

* Launch the Tallysman ROS2 node:
   ```
   ros2 run tallysman_ros2 tallysman_gps
   ```
      If you face permission error like below image, use   
      ```
      sudo chmod a+rw /dev/ttyUSB0
      ```
    ![Screenshot from 2023-08-23 12-53-12](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/c46049b1-e139-4b14-b243-a2f6754a18fb)

   When you ros run the ``` tallysman_gps ``` node, you will see latitude and longitude data.
   ![Screenshot from 2023-08-23 12-59-25](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/0d7bc44d-9f89-4aa1-9c1f-ba5538103a3b)
    

## Contributing

Contributions to Tallysman ROS2 are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

# Reference:

  For more information about the Tallysman ROS2 driver please refer to https://github.com/indro-robotics/tallysman_ros2.git


  



  

