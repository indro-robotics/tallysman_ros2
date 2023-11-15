import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='params/config.yaml'),
        Node(
            package='tallysman_ros2',  
            executable='tallysman_gps',  
            name='tallysman_gps_publisher',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),
    ])
