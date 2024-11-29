import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取 YAML 文件的路径
    param_file = os.path.join(
        '/home/bei/dev_ws/src/traffic_light_detection',
        'config',
        'traffic_light_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=param_file, description='Path to the parameter file'),
        
        Node(
            package='traffic_light_detection',
            executable='traffic_light_node',
            name='traffic_light_node',
            parameters=[param_file]
        ),
    ])

