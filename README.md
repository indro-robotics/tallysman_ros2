# :world_map: Calian Gnss Ros2 Driver

This repository contains the ROS2 package for integrating Calian GNSS receivers with ROS2-based systems.

## ROS2 Driver

## Table of Contents

- [:hugs: Introduction](#hugs-introduction)
- [:dizzy: Features](#dizzy-features)
- [:envelope\_with\_arrow: Requirements](#envelope_with_arrow-requirements)
- [:bar_chart: Parameters](#bar_chart-parameters)
- [:gear: Operating Modes](#gear-operating-modes)
- [:round_pushpin: PointPerfect Setup](#round_pushpin-pointperfect-setup)
- [:rocket: Installation](#rocket-installation)
- [:books: Usage](#books-usage)
- [:camera\_flash: Video](#camera_flash-video)
- [:handshake: Contributing](#handshake-contributing)
- [:phone: Purchase Call](#phone-purchase-call)
- [:sparkles: Reference](#sparkles-reference)
- [:ledger: Resources](#ledger-resources)
- [:page\_with\_curl: License](#page_with_curl-license)

## :hugs: Introduction

Calian Gnss ROS2 is a ROS2 package that provides functionality for interfacing with Calian GNSS receivers. It allows you to receive and process GNSS data within your ROS2-based systems. This package provides ROS 2 nodes and utilities to interact with Calian GNSS receivers, enabling accurate localization, navigation, and time synchronization for your robotic projects.

## :dizzy: Features

- **Real-Time Positioning:** Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- **ROS 2 Integration:** Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS-based applications.
- **Customizable Configuration:** Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- **Monitoring Tool:** Monitor you robot's location a map view.

## :envelope_with_arrow: Requirements

- [Calian GNSS Antenna](https://tallymatics.com/product/tw5390/)
- [Ubuntu 22](https://indrorobotics.notion.site/Installing-Dual-OS-and-upgrade-laptop-SSD-0d7c4b8ee9d54e14bbeb9f7ac24f8079?pvs=4)
- [ROS2 (Humble)](https://www.notion.so/indrorobotics/Getting-Started-with-ROS2-a3960c906f0d46789cd1d7b329784dd0)
- [Python](https://docs.python.org/3/)

  ```diff
  + IMP NOTE: Source your package every time you make change or open a new terminal. 
  + Else you will see Error like <<Package 'calian_gnss_ros2' not found>> even if you have cloned it.
  ```
## :bar_chart: Parameters

These are the Parameters defined in the Tallysman Ros2 Node

1. **`unique_id (string)`:**
   - Unique Id of Calian Gnss receiver. Run the `Unique_id_finder` node (assumes default baudrate) to get the unique ids of all connected antennas.

2. **`baud_rate (integer)`:**
   - Baud rate for serial communication. Default value should be 230400.

3. **`save_logs (boolean)`:**
   - Flag to save logs. If true, all the logs will be saved to the logs folder.

4. **`log_level (integer)`:**
   - Logging level. Log level values are of ROS2 logging standards. Default is `Info`.
     - `(NotSet: 0, Debug: 10, Info: 20, Warn: 30, Error: 40, Critical: 50)`.

5. **`use_corrections (boolean)`:**
   - Flag indicating whether PPP-RTK corrections should be used.

6. **`config_path (string)`:**
   - Path to PPP-RTK configuration file.

7. **`region (string)`:**
   - Region information. Accepted values are `us, eu, kr, au`.

## :gear: Operating Modes

The Calian GNSS antenna can be operated in different modes based on configuration. The mode of operation cannot be changed when the node is running. It must be passed as an argument in the launch file.

### Disabled Mode:

- **Description:** This mode works as a standalone node. Any Calian Gnss antenna can work in this mode.
- **Functionality:** Connects to the Calian Gnss antenna and publishes GPS data (latitude and longitude) on the `gps` topic with the message type NavSatFix. Complete Gnss receiver signal status is provided in the topic `gps_extended` with the message type GnssSignalStatus (Depends on calian_gnss_ros2_msg package)
- **Launch File:** `calian_gnss_ros2/launch/disabled.launch.py`
- **Parameters:**
  - unique_id
  - baud_rate
  - save_logs
  - log_level
  - use_corrections
  - config_path
  - region
- **Launch Arguments:** `arguments=['Disabled']`

### Heading_Base Mode:

- **Description:** Part of the moving baseline configuration. Tallysman antennas equipped with Zed-f9p will only work in this mode.
- **Functionality:** Connects to the serial port of the Tallysman antenna and publishes Rtcm corrections on the "rtcm_corrections" topic. This topic is intended for internal use by the respective rover nodes.
- **Launch File:** `Tallysman_ros2/launch/tallysman_moving_baseline.launch.py`
- **Parameters:**
  - usb_port
  - baud_rate
  - save_logs
  - log_level
  - use_corrections
  - config_path
  - region
- **Launch Arguments:** `arguments=['Heading_Base']`

### Rover Mode:

- **Description:** Can be used in both moving baseline configuration and static baseline configuration. Tallysman antennas equipped with Zed-f9p/Zed-f9r(Doesn't work in moving baseline configuration only f9p does) will only work in this mode.
- **Functionality:** Connects to the serial port of the Tallysman antenna, subscribes to the "rtcm_corrections" topic created by the base node.
- **Requirements:** Requires another node running with Heading_Base or Static_Base.
- **Parameters:**
  - usb_port
  - baud_rate
  - save_logs
  - log_level
- **Launch Arguments:** `arguments=['Rover']`

It's crucial to configure the launch files with the appropriate parameters and arguments based on the desired mode of operation. Additionally, ensure that the necessary dependencies are met and the topic names are unique for the antennas in same configuration.

## :round_pushpin: PointPerfect Setup

To achieve centimeter-level accuracy in real-time, PPP-RTK corrections are essential. These corrections can be obtained through the Pointperfect subscription service, accessible at **`https://www.u-blox.com/en/product/pointperfect`**. Follow the steps below to acquire and configure the necessary files:
- Visit the website to subscribe to the Pointperfect service.
- Once subscribed, navigate to the "Credentials" tab under the "Location Thing Details" section on the website.
- Download the ucenter configuration file provided and rename it to **`ucenter-config.json`**.
- Create a new folder named **`pointperfect_files`** at the following directory: **`humble_ws/src/calian_gnss_ros2/pointperfect_files/`**.
- Place the **`ucenter-config.json`** file inside the newly created **`pointperfect_files`** folder.
- When you run the node, it will generate several files within the **`pointperfect_files`** folder, which are necessary for establishing a connection to the subscription service.

## :rocket: Installation

To install Calian GNSS ROS2, follow these steps:

1. **ROS Workspace:** Move to your ROS2 workspace (example: mine is humble_ws)

   ```bash
    cd ~/humble_ws/src
    ```

2. **Clone the repository:**

    ```bash
    git clone https://github.com/indro-robotics/tallysman_ros2.git
    git clone https://github.com/indro-robotics/tallysman_msg.git
    ```
   
3. **Build the package using colcon:**

    ```bash
    cd ~/humble_ws
    colcon build
    ```

4. **source the setup file:**

    ```bash
    source install/setup.bash
    ```

5. Go to your project, and install requirements.txt for installing all the required libraries at once. (for exampe mine is under humble_ws/src/tallysman_ros2)

    ```bash
    cd humble_ws/src/calian_gnss_ros2
    pip install -r requirements.txt
      ```

## :books: Usage

The Calian GNSS ROS2 package provides flexibility in its configurations, and example launch files for different setups can be found in the **`launch`** folder (**`/src/calian_gnss_ros2/launch/`**). The package includes the **`gps_visualizer`** node, designed to run alongside the **`gps`** node, enabling the visualization of the published location data. Ensure to change the **`unique_id`** parameter in the launch files to the desired gnss receiver.

### RTK disabled configuration.

- Ensure the presence of the **`config_file.json`** in the designated location, as specified in the PPP-RTK corrections setup section.
- Set the parameters in the launch file:
  - **`use_corrections`** to True if the corrections service needs to be used.
  - **`config_path`** to 'src/calian_gnss_ros2/pointperfect_files/ucenter_config_file.json'
  - **`region`** to the desired region
- Build the workspace using **`colcon build`** and source the setup file with **`source install/setup.bash`** and your ROS setup. Repeat this step for any changes in the launch file.
- Launch the nodes using the following command:
   ```
   ros2 launch calian_gnss_ros2 disabled.launch.py
   ```

- Upon execution, the **`tallysman_gps`** node starts in disabled configuration, publishing location data to the **`gps`** topic.
- The visualizer node is also initiated, and you can view the mapped location data at **http://localhost:8080**.

- **Note**: You can modify the default port number (8080) of the visualizer node by adjusting the **`port`** parameter in the launch file.
<img width="1245" alt="tallysman_disabled.launch" src="https://github.com/indro-robotics/tallysman_ros2/assets/29984780/79fa878a-4112-4980-8d31-fe0dfe91d498">

### RTK-Moving Baseline configuration:

For the RTK-Moving Baseline configuration, which involves two Tallysman antennas (one base and one rover), and only antennas with Zed-f9p chips acting as the base:

- Follow similar steps as the RTK disabled configuration and make necessary changes to the launch file.
- Build and source the terminal.
- Launch the nodes with the command:

   ```
   ros2 launch tallysman_ros2 tallysman_moving_baseline.launch.py
   ```
- Upon execution, the tallysman_gps nodes start in Base and Rover modes in moving baseline configuration, publishing location data to the **`gps`** topic and extended information like heading, quality and accuracies to **`gps_extended`** topics.
- Access the location data at **http://localhost:8080**.
- Ensure to have clear skies to get good precision values.

To view active topics use command
```bash
ros2 topic list
```

## :camera_flash: Video


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
