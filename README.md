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

To find the connected USB devices use command "ls /dev/ttyUSB*". If the udev rule is added you can find the tallysman antenna ports by command "ls /dev/Tallysman_USB*". Replace the `usb_port` parameter in the launch file with correct port name.

## :dizzy: Features

- **Real-Time Positioning:** Achieve high-precision real-time positioning for your robots, essential for tasks like autonomous navigation and mapping
- **ROS 2 Integration:** Seamlessly integrate GNSS data into your ROS 2 ecosystem, making it easy to access and use GNSS information within your ROS-based applications.
- **Customizable Configuration:** Tailor the configuration to your specific requirements, with options for different GNSS constellations, frequencies, and data output formats.
- **Monitoring Tool:** Monitor you robot's location a map view.

## :envelope_with_arrow: Requirements

- [Tallymatics Antenna](https://tallymatics.com/product/tw5390/)
- [Ubuntu 22](https://indrorobotics.notion.site/Installing-Dual-OS-and-upgrade-laptop-SSD-0d7c4b8ee9d54e14bbeb9f7ac24f8079?pvs=4)
- [ROS2 (Humble)](https://www.notion.so/indrorobotics/Getting-Started-with-ROS2-a3960c906f0d46789cd1d7b329784dd0)
- [Python](https://docs.python.org/3/)

  ```diff
  + IMP NOTE: Source your package every time you make change or open a new terminal. 
  + Else you will see Error like <<Package 'tallysman_ros2' not found>> even if you have clone it.
  ```

## :rocket: Installation

To install Tallysman ROS2, follow these steps:

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
    cd humble_ws/src/tallysman_ros2
    pip install -r requirements.txt
      ```

## :bar_chart: Parameters

These are the Parameters defined in the Tallysman Ros2 Node

1. **`usb_port (string)*`:**
   - Standard/Read and write port for the Tallysman antenna. The udev rule will automatically detect this port and rename it to "Tallysman_USB" if added.

2. **`baud_rate (integer)*`:**
   - Baud rate for serial communication. Default value should be 230400.

3. **`save_logs (boolean)*`:**
   - Flag to save logs. If true, all the logs will be saved to the logs folder.

4. **`log_level (integer)*`:**
   - Logging level. Log level values are of ROS2 logging standards. Default is `Info`.
     - `(NotSet: 0, Debug: 10, Info: 20, Warn: 30, Error: 40, Critical: 50)`.

5. **`use_corrections (boolean)`:**
   - Flag indicating whether PPP-RTK corrections are used.

6. **`config_path (string)`:**
   - Path to PPP-RTK configuration file.

7. **`region (string)`:**
   - Region information. Accepted values are `us, eu, kr, au`.

Note: Parameters marked with `*` can be changed dynamically; others have no effect if changed.


## :gear: Operating Modes

The Tallysman antenna can be operated in different modes based on configuration. The mode of operation cannot be changed when the node is running. It must be passed as an argument in the launch file.

### Disabled Mode:

- **Description:** This mode is under development and works as a standalone node.
- **Functionality:** Connects to the serial port of the Tallysman antenna and publishes GPS data (latitude and longitude) on the "gps" topic with the message type NavSatFix.
- **Launch File:** `Tallysman_ros2/launch/tallysman_disabled.launch.py`
- **Parameters:**
  - usb_port
  - baud_rate
  - save_logs
  - log_level
  - use_corrections
  - config_path
  - region
- **Launch Arguments:** `arguments=['Disabled']`

### Static_Base Mode:

- **Description:** Under development (details not provided).
- **Functionality:** Yet to be developed.

### Heading_Base Mode:

- **Description:** Part of the moving baseline heading configuration.
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

- **Description:** Can be used in both moving baseline heading configuration and static baseline heading configuration.
- **Functionality:** Connects to the serial port of the Tallysman antenna, subscribes to the "rtcm_corrections" topic created by the base node.
- **Requirements:** Requires another node running with Heading_Base or Static_Base.
- **Parameters:**
  - usb_port
  - baud_rate
  - save_logs
  - log_level
- **Launch Arguments:** `arguments=['Rover']`

It's crucial to configure the launch files with the appropriate parameters and arguments based on the desired mode of operation. Additionally, ensure that the necessary dependencies are met and the topic names are unique for the antennas in same configuration.

## :computer: Udev Rule

If an Ubuntu OS is being used, add the following udev rule to detect the standard bidirectional port of the Tallysman antenna. This rule detects the standard bidirectional port of the antenna and renames it to “Tallysman_USB” followed by the Kernel number.

Ensure that this udev rule is added to the system configuration ( `/lib/udev/rules.d/40-tallysman.rules` file ) to facilitate the detection and naming of the Tallysman antenna's standard bidirectional port. You need to have sudo permissions to add this rule.

```udev
KERNEL=="ttyUSB*", SUBSYSTEMS=="usb", DRIVERS=="cp210x", ATTRS{interface}=="Standard Com Port", SYMLINK+="Tallysman_USB%n"
```

After adding the rule run the below command to reload the udev rules without doing a reboot.

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Note:** Change the `usb_port` parameter of the launch files to `/dev/ttyUSB{kernel_number}` of the bidirectional port of the antenna if udev rule is not configured.

## :books: Usage

Tallysman Ros2 package can be used in different configurations. Example launch files for different configurations can be found in the `launch` folder (`/src/tallysman_ros2/launch/`) of the Tallysman ROS2 package. The `tallysman_gps_visualizer` node is run alongside the `tallysman_gps` node to display the published location data.

### RTK disabled configuration.

- To use PPP-RTK corrections, make sure you have the config_file.json in place as mentioned in the PPP-RTK corrections setup section and parameters `use_corrections` is True and `config_path` is 'src/tallysman_ros2/pointperfect_files/config_file.json' and `region` is desired region. These three parameters are necessary for the PPP-RTK corrections.
- After modifying the launch file build the workspace using `colcon build` and source the terminal using `source install/setup.bash`. You need to build and source it every time you change something in the launch file.
- Launch the nodes using the command below.
   ```
   ros2 launch tallysman_ros2 tallysman_disabled.launch.py
   ```
- When the above command is executed the tallysman_gps node is started in disabled configuration and you can see location data getting published onto `gps` topic.
- Also, The visualizer node is also started and you can visit http://localhost:8080 to see the location data mapped onto a map.

- **Note**: Default port_number of visualizer node is 8080 but you can change it by modifying the `port` parameter of the launch file.

### RTK-Moving Baseline configuration:

Moving Baseline configuration requires two tallysman antennas (One base, one rover). Only antennas with Zed-f9p chips can act as Base and Antennas with both ZED-F9P/ ZED-F9R chips can act as Rover.

- Follow similar steps of RTK disabled configuration and make necessary changes to the launch file
- Build and source the terminal
- Launch the nodes using the command below.

   ```
   ros2 launch tallysman_ros2 tallysman_moving_baseline.launch.py
   ```
 When the above command is executed the tallysman_gps node is started in disabled configuration and you can see location data getting published onto `gps` topic.
- Find the location data at http://localhost:8080

- To view active topics:
![tallysman6](https://github.com/indro-robotics/tallysman_ros2/assets/128490600/15d5bd97-fad7-4944-b339-7ea77791b593)

- All the latitude and longitude data will be stored in 'gps_history.json' file, you can find .json file in your ros directoy.

    ```
   cd ~/humble_ws
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
