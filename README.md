# :world_map: tallysman_ros2 
This repository contains the ROS2 package for integrating Tallysman GNSS and GNRMC receivers with ROS2-based systems.

ROS2 Driver

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Video](#video)
- [Contributing](#contributing)
- [Reference](#reference)


## :hugs: Introduction

Tallysman ROS2 is a ROS2 package that provides functionality for interfacing with Tallysman GNSS receivers. It allows you to receive and process GNSS data within your ROS2-based systems. This package provides ROS 2 nodes, drivers, and utilities to interact with Tallysman GNSS receivers, enabling accurate localization, navigation, and time synchronization for your robotic projects.


### :dizzy: Features

- Real-Time Positioning: Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- ROS 2 Integration: Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS-based applications.
- Customizable Configuration: Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- Monitoring Tool: Monitor you robot's location a map view.

## :envelope_with_arrow: Requirements 
* [Ubuntu 22](https://indrorobotics.notion.site/Installing-Dual-OS-and-upgrade-laptop-SSD-0d7c4b8ee9d54e14bbeb9f7ac24f8079?pvs=4)
* [ROS (Humble)](https://www.notion.so/indrorobotics/Getting-Started-with-ROS2-a3960c906f0d46789cd1d7b329784dd0)
* [Python](https://docs.python.org/3/)
  ```diff
  + IMP NOTE: Source your package every time you make change or open a new terminal. 
  + Else you will see Error like <<Package 'tallysman_ros2' not found>> even if you have clone it.
  ```

## :rocket: Installation 

To install Tallysman ROS2, follow these steps:

1. ROS Workspace: Move to your ROS2 workspace (example: mine is test_ws)
   ```
    cd ~/test_ws/src
    ```

3. Clone the repository:
    ```
    git clone https://github.com/indro-robotics/tallysman_ros2.git
    ```
4. Build the package using colcon:
    ```
    cd ~/test_ws
    colcon build
    ```
5. source the setup file:
    ```
    source install/setup.bash
    ```
6. Go to your project, and install requirements.txt for installing all the required libraries at once. (for exampe mine is under test_ws/src/tallysman_ros2)
    ```
    cd test_ws/src/tallysman_ros2
    pip install -r requirements.txt
      ```


## :books: Usage

To use Tallysman ROS2 in your ROS2-based system, follow these steps:

* Launch the tallysman_gps node:
   ```
   ros2 run tallysman_ros2 tallysman_gps
   ```
:bangbang: :bangbang: :bangbang: :bangbang: :bangbang: :bangbang: :bangbang: :bangbang: :bangbang: :bangbang:
  ```diff
  - If you face permission error like below image, use 'chmod' command
  
  sudo chmod a+rw /dev/ttyUSB0
  ```

![Screenshot from 2023-08-23 12-53-12](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/c46049b1-e139-4b14-b243-a2f6754a18fb)




   When you ros run the ``` tallysman_gps ``` node, you will see latitude and longitude data.
   ![Screenshot from 2023-08-23 12-59-25](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/0d7bc44d-9f89-4aa1-9c1f-ba5538103a3b)
  
* Launch tallysman_gps_visualize node:
  Open another terminal and source it.
  Now, launch the tallysman_gps_visualize node
  ```
   ros2 run tallysman_ros2 tallysman_gps_visualize
  ```
  ![Screenshot from 2023-08-23 13-04-24](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/dab1c3fa-3960-4bcc-b427-448d003fe5be)
* Next, Open your favourite web browser, and navigate to gps_maps.html, this will point your location
  ![Screenshot from 2023-08-23 13-07-25](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/f8028fba-fbd7-4d91-a5a2-e98fcd2631e6)
* When you move around the area, you can see all the points where you have navigated. (Note: You need to refresh the browser every time you move)
  ![Screenshot from 2023-08-23 13-10-57](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/f429e21d-d209-46b7-83b0-1f55882ac869)
* To view active topics:
  ![Screenshot from 2023-08-25 09-26-54](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/b0748f2a-5087-4135-b3a4-aa13047c5741)

  
## :camera_flash: Video

https://github.com/indro-robotics/tallysman_ros2/assets/128490600/ec1810b3-661d-4fbc-b535-047140455af2


## :handshake: Contributing

Contributions to Tallysman ROS2 are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

# :sparkles: Reference:

  For more information about the Tallysman ROS2 driver please refer to https://github.com/indro-robotics/tallysman_ros2.git
