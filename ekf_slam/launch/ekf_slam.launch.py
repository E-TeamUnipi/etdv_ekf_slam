from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('ekf_slam'),
        'config',
        'ekf_params.yaml'  # nome del tuo file yaml
    ])

    return LaunchDescription([
        Node(
            package='ekf_slam',
            executable='ekf_node',
            name='ekf_node',
            parameters=[params_file],
            output='screen'
        )
    ])