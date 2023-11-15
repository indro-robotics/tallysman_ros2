# import launch
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('config_file', default_value='/params/config.yaml'),
#         Node(
#             package='tallysman_ros2',  
#             executable='tallysman_gps',  
#             name='tallysman_gps_publisher',
#             output='screen',
#             parameters=[LaunchConfiguration('config_file')]
#         ),
#     ])


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tallysman_ros2',
            executable='tallysman_gps',
            name='tallysman_gps_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud_rate': 230400},
                {'topic_name': 'gps_data_launch'},
                {'usb_port':'/dev/ttyUSB10'}
            ]
        )
    ])