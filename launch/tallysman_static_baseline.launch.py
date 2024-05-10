# import launch
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

#     Log Level values for different logging levels
#     NotSet = 0
#     Debug = 10
#     Info = 20
#     Warn = 30
#     Error = 40
#     Critical = 50

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tallysman_ros2'),
        'params',
        'config.yaml'
        )
    
    corrections_config = os.path.join(
        get_package_share_directory('tallysman_ros2'),
        'params',
        'pointperfect.yaml'
        )
    
    logs_config = os.path.join(
        get_package_share_directory('tallysman_ros2'),
        'params',
        'logs.yaml'
        )
    return LaunchDescription([
        Node(
            package='tallysman_ros2',
            executable='remote_rtcm_corrections_handler',
            name='rtcm_handler',
            output='screen',
            emulate_tty=False,
            parameters=[
                config, logs_config
            ],
            namespace='tallysman',
            remappings=[
                ('rtcm_corrections','rtcm_topic')
            ],
        ),
        Node(
            package='tallysman_ros2',
            executable='tallysman_gps',
            name='rover',
            output='screen',
            emulate_tty=True,
            parameters=[
                config, corrections_config, logs_config
            ],
            namespace='tallysman',
            remappings=[
                ('rtcm_corrections','rtcm_topic')
            ],
            arguments=['Rover']
        ),
        # Node(
        #     package='tallysman_ros2',
        #     executable='tallysman_gps_visualizer',
        #     name='gps_visualizer',
        #     output='screen',
        #     emulate_tty=False,
        #     parameters=[
        #         {'port': 8080}
        #     ],
        #     namespace='tallysman',
        # )
    ])