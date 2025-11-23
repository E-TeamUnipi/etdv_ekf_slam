"""
Launch file per avviare il controller circolare
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Percorso al file di configurazione
    config_file = os.path.join(
        get_package_share_directory('circular_controller'),
        'config',
        'circular_params.yaml'
    )
    
    # Nodo controller
    circular_controller_node = Node(
        package='circular_controller',
        executable='circular_controller_node',
        name='circular_controller',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        circular_controller_node
    ])
