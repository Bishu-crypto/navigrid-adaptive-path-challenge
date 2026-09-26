import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('navigrid_gazebo')
    world_path = os.path.join(pkg_gazebo, 'worlds', 'navigrid_arena.sdf')

    headless = LaunchConfiguration('headless', default='false')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        additional_env={'GZ_IP': '127.0.0.1', 'GZ_TRANSPORT_DISCOVERY_INTERFACE': 'lo'}
    )

    return LaunchDescription([
        declare_headless,
        gz_sim
    ])
