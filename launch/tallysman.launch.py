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
            name='base',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud_rate': 38400},
                {'topic_name': 'gps_data_launch'},
                {'usb_port':'/dev/ttyUSB3'},
                {'is_base': True},
                {'use_corrections': True},
                {'config_path': '/root/humble_ws/src/tallysman_ros2/pointperfect_files/ucenter-config.json'},
                {'region': 'us'},
                {'rtcm_topic_name': 'rtcm_corrections'}
            ]
        ),
        Node(
            package='tallysman_ros2',
            executable='tallysman_gps',
            name='rover',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud_rate': 230400},
                {'topic_name': 'gps_data_launch'},
                {'usb_port':'/dev/ttyUSB1'},
                {'use_corrections': False},
                {'is_base': False},
                {'rtcm_topic_name': 'rtcm_corrections'}
            ]
        )
    ])