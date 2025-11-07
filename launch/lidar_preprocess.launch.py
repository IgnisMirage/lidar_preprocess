from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_preprocess'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lidar_preprocess',
            executable='lidar_preprocess_node',
            name='lidar_preprocess_node',
            parameters=[config],
            remappings=[
                ('input', '/lidar/points_raw'),
                ('output', '/lidar/points_filtered')
            ]
        )
    ])
