# :world_map: Tallysman Ros2

This repository contains the ROS2 package for integrating Tallysman GNSS 5790 and GNRMC receivers with ROS2-based systems.

## ROS2 Driver

## Table of Contents

- [:hugs: Introduction](#hugs-introduction)
- [:dizzy: Features](#dizzy-features)
- [:envelope\_with\_arrow: Requirements](#envelope_with_arrow-requirements)
- [:rocket: Installation](#rocket-installation)
- [:bar_chart: Parameters](#bar_chart-parameters)
- [:gear: Operating Modes](#gear-operating-modes)
- [:computer: Udev Rule](#computer-udev-rule)
- [:books: Usage](#books-usage)
- [:camera\_flash: Video](#camera_flash-video)
- [:handshake: Contributing](#handshake-contributing)
- [:phone: Purchase Call](#phone-purchase-call)
- [:sparkles: Reference](#sparkles-reference)
- [:ledger: Resources](#ledger-resources)
- [:page\_with\_curl: License](#page_with_curl-license)

## :hugs: Introduction

Tallysman ROS2 is a ROS2 package that provides functionality for interfacing with Tallysman GNSS receivers. It allows you to receive and process GNSS data within your ROS2-based systems. This package provides ROS 2 nodes, drivers, and utilities to interact with Tallysman GNSS receivers, enabling accurate localization, navigation, and time synchronization for your robotic projects.

### USB port

You can find your port by command "ls /dev/ttyUSB*" replace in launch file with your correct port as shown below
![image](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/a7c957ba-d4a4-4160-80e0-fedfa7e63486)

## :dizzy: Features

- **Real-Time Positioning:** Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- **ROS 2 Integration:** Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS-based applications.
- **Customizable Configuration:** Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- **Monitoring Tool:** Monitor you robot's location a map view.

## :envelope_with_arrow: Requirements

- [Tallymatics Antenna](https://tallymatics.com/product/tw5390/)
- [Ubuntu 22](https://indrorobotics.notion.site/Installing-Dual-OS-and-upgrade-laptop-SSD-0d7c4b8ee9d54e14bbeb9f7ac24f8079?pvs=4)
- [ROS (Humble)](https://www.notion.so/indrorobotics/Getting-Started-with-ROS2-a3960c906f0d46789cd1d7b329784dd0)
- [Python](https://docs.python.org/3/)

  ```diff
  + IMP NOTE: Source your package every time you make change or open a new terminal. 
  + Else you will see Error like <<Package 'tallysman_ros2' not found>> even if you have clone it.
  ```

## :rocket: Installation

To install Tallysman ROS2, follow these steps:

1. **ROS Workspace:** Move to your ROS2 workspace (example: mine is test_ws)

   ```bash
    cd ~/test_ws/src
    ```

2. **Clone the repository:**

    ```bash
    git clone https://github.com/indro-robotics/tallysman_ros2.git
    ```

3. **Build the package using colcon:**

    ```bash
    cd ~/test_ws
    colcon build
    ```

4. **source the setup file:**

    ```bash
    source install/setup.bash
    ```

5. Go to your project, and install requirements.txt for installing all the required libraries at once. (for exampe mine is under test_ws/src/tallysman_ros2)

    ```bash
    cd test_ws/src/tallysman_ros2
    pip install -r requirements.txt
      ```

## :bar_chart: Parameters

These are the Parameters defined in the Tallysman Ros2 Node

1. `usb_port`: Standard/Read and write port for the Tallysman antenna.
2. `baud_rate`: Baud rate for serial communication.
3. `save_logs`: Flag to save logs.
4. `log_level`: Logging level.
5. `use_corrections`: Flag indicating whether corrections are used.
6. `config_path`: Path to configuration files.
7. `region`: Region information.

## :gear: Operating Modes

The Tallysman antenna can be operated in different modes based on configuration. The mode of operation cannot be changed when the node is running. It must be passed as an argument in the launch file.




## :computer: Udev Rule

If an Ubuntu OS is being used, add the following udev rule to detect the standard bidirectional port of the Tallysman antenna. This rule detects the standard bidirectional port of the antenna and renames it to “Tallysman_USB” followed by the Kernel number.

```udev
KERNEL=="ttyUSB*", SUBSYSTEMS=="usb", DRIVERS=="cp210x", ATTRS{interface}=="Standard Com Port", SYMLINK+="Tallysman_USB%n"
```

**Note:** Ensure that this udev rule is added to the system configuration to facilitate the detection and naming of the Tallysman antenna's standard bidirectional port.

## :books: Usage

To use Tallysman ROS2 in your ROS2-based system, follow these steps:

- Launch the tallysman_gps node:

   ```
   ros2 run tallysman_ros2 tallysman_gps
   ```

  ```diff
  - If you face permission error like below image, use 'chmod' command
  
  sudo chmod a+rw /dev/ttyUSB0
  ```

  ![tallysman1](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/b1640fd8-1d59-4c8e-af26-7435f9b13373)

   When you ros run the ``` tallysman_gps ``` node, you will see latitude and longitude data.
  ![tallysman2](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/a8a1d9ba-3aa0-4ad2-b6b3-57460abee6ba)

- Launch tallysman_gps_visualize node:
  Open another terminal and source it.
  Now, launch the tallysman_gps_visualize node

  ```
   ros2 run tallysman_ros2 tallysman_gps_visualize
  ```

![tallysman3](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/8e2966a7-eeab-4263-9336-2652a8be6cbe)

- Next, Open your favourite web browser, and navigate to gps_maps.html, this will point your location
![tallysman4](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/021543d2-bf22-4336-bb18-b11032190e2b)

- When you move around the area, you can see all the points where you have navigated. (Note: You need to refresh the browser every time you move)
![tallysman5](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/a64d5f7f-6260-4bda-ad56-82c8e89c9b9f)

- To view active topics:
![tallysman6](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/15d5bd97-fad7-4944-b339-7ea77791b593)

- All the latitude and longitude data will be stored in 'gps_history.json' file, you can find .json file in your ros directoy after running 'tallysman_gps_visualizer.py' node

    ```
   cd ~/test_ws
  ```

## :camera_flash: Video

<https://github.com/indro-robotics/tallysman_ros2/assets/128490600/bf561eb6-1bd1-4250-8b38-5b735d711047>

## :handshake: Contributing

Contributions to Tallysman ROS2 are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request on GitHub.

## :phone: Purchase Call

For inquiries or to purchase our Tallymatics Antenna, please contact us at [Luke Corbeth(at InDro Robotics)](lcorbeth@indrorobotics.com) or [Tallysman](info@tallymatics.com). We are excited to assist you and provide further information about our offerings.

## :sparkles: Reference

For more information about the Tallysman ROS2 driver please refer to [GitHub](https://github.com/indro-robotics/tallysman_ros2.git)

## :ledger: Resources

We would like to express our sincere gratitude to [Tallymatics](https://tallymatics.com/) for their collaboration and support. Their contributions have been invaluable to our project's success, and we look forward to continuing our partnership in the future.

## :page_with_curl: License

This project is licensed under the terms of the [MIT License](https://github.com/indro-robotics/tallysman_ros2/blob/main/LICENSE)
