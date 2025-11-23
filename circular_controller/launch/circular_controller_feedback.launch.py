"""
Launch file per il controller circolare con feedback dalla posizione
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('circular_controller'),
        'config',
        'circular_feedback_params.yaml'
    )
    
    circular_controller_feedback_node = Node(
        package='circular_controller',
        executable='circular_controller_feedback_node',
        name='circular_controller_feedback',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        circular_controller_feedback_node
    ])
