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
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),

        Node(
            package='ekf_slam',
            executable='tf_to_pose.py',
            name='tf_to_pose_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])