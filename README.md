# tallysman_ros2
ROS2 (humble) Python Script for Tallysman Antenna: Extracting Latitude and Longitude

ROS2 Driver


# GPS Data Publisher and Subscriber
## Description
This project provides a GPS data publisher that reads NMEA data from a Tallysman USB GPS receiver and publishes the latitude and longitude information to the `/gps_data` topic. Additionally, it includes a subscriber that listens to the `/gps_data` topic and prints the received GPS data.

The publisher and subscriber are implemented as ROS 2 nodes and can be used to interface with a GPS module in ROS 2 applications.


## Features- Reads NMEA data from a Tallysman USB GPS receiver.- Parses the NMEA data to extract latitude and longitude information.- Publishes GPS data as `sensor_msgs/NavSatFix` messages on the `/gps_data` topic.- Subscribes to the `/gps_data` topic and prints the received GPS data.


## Prerequisites
Before running the project, ensure you have the following installed:
ROS 2 (Humble Hawksbill) or later version.- Python 3 and the `serial` library.


## Installation1. Clone the repository:
   <code>git clone https://github.com/indro-robotics/ROS2_Tallysman.git</code>

   <code>colcon build</code>

   

To run the GPS data publisher node, use the following command:

<code>ros2 run gps_project gps_publisher.py</code>


The publisher will start reading data from the USB GPS receiver and publishing the GPS data on the /gps_data topic.

GPS Data Subscriber

To run the GPS data subscriber node, use the following command:

<code>ros2 run gps_project gps_subscriber.py<code>



The subscriber will start listening to the /gps_data topic and print the received GPS data (latitude and longitude) whenever new messages are published.

Contributing

Contributions to the project are welcome. Feel free to submit bug reports, feature requests, or pull requests. For major changes, please open an issue first to discuss the changes.

License

This project is licensed under the MIT License - see the LICENSE file for details.

Contact

For any inquiries or questions about the project, please contact:

Nirali Patel
Arif Anjum

Email: nirali.patel@indrorobotics.com
       arif.anjum@indrorobotics.com





The README.md file is an essential part of your project, so make sure to provide clear instructions and information about your GPS project to help users and contributors understand and use it effectively.
