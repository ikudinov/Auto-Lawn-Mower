import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('lawn_mower_web_controller'),
      'config',
      'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='lawn_mower_web_controller',
            executable='serial_node',
            name='serial',
            parameters=[config]
        ),
        Node(
            package='lawn_mower_web_controller',
            executable='ws_node',
            name='webscoket',
            parameters=[config]
        ),
        Node(
            package='lawn_mower_web_controller',
            executable='web_node',
            name='webserver',
            parameters=[config]
        ),
    ])
