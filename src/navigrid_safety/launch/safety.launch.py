from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigrid_safety',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='navigrid_safety',
            executable='twist_priority_mux',
            name='twist_priority_mux',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
