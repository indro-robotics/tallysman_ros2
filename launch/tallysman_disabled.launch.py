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
    return LaunchDescription([
        Node(
            package='tallysman_ros2',
            executable='tallysman_gps',
            name='gps_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud_rate': 230400},
                {'usb_port':'/dev/Tallysman_USB1'},
                {'use_corrections': True},
                {'config_path': 'src/tallysman_ros2/pointperfect_files/ucenter-config.json'},
                {'region': 'us'},
                {'save_logs': False},
                {'log_level': 20}
            ],
            namespace='tallysman',            
            arguments=['Disabled']
        ),
        Node(
            package='tallysman_ros2',
            executable='tallysman_gps_visualizer',
            name='gps_visualizer',
            output='screen',
            emulate_tty=False,
            parameters=[
                {'port': 8080}
            ],
            namespace='tallysman',
        )
    ])