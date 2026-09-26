import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

# Ensure gz-transport uses loopback interface to prevent multicast network issues
os.environ['GZ_IP'] = '127.0.0.1'
os.environ['GZ_TRANSPORT_DISCOVERY_INTERFACE'] = 'lo'


def generate_launch_description():
    pkg_bringup = get_package_share_directory('navigrid_bringup')
    pkg_description = get_package_share_directory('navigrid_description')
    pkg_gazebo = get_package_share_directory('navigrid_gazebo')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'navigrid_arena.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (-s) if true'
    )

    # Robot State Publisher
    robot_description = Command(['xacro ', xacro_file])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Robot Node via native Gazebo Harmonic service
    spawn_robot_node = Node(
        package='navigrid_bringup',
        executable='spawn_robot.py',
        output='screen',
        arguments=[
            '-world', 'navigrid_arena',
            '-name', 'navigrid_robot',
            '-x', '-11.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0',
            '-xacro', xacro_file
        ],
        additional_env={'GZ_IP': '127.0.0.1', 'GZ_TRANSPORT_DISCOVERY_INTERFACE': 'lo'}
    )

    # Native Gazebo Harmonic Bridge for /scan, /imu, /odom, /clock, /cmd_vel
    bridge_node = Node(
        package='navigrid_bringup',
        executable='harmonic_bridge',
        name='harmonic_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'GZ_IP': '127.0.0.1', 'GZ_TRANSPORT_DISCOVERY_INTERFACE': 'lo'}
    )

    # Static transforms: base_link -> lidar_link and base_link -> imu_link
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_pub',
        arguments=['--x', '0.10', '--y', '0.0', '--z', '0.15', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf_pub',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.12', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Odometry TF Broadcaster (odom -> base_footprint)
    odom_tf_node = Node(
        package='navigrid_bringup',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Secondary AMR Dynamic Patrol Node
    secondary_amr_node = Node(
        package='navigrid_gazebo',
        executable='amr_patrol.py',
        name='secondary_amr_patrol',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # High-Priority Dynamic Safety Override Node
    safety_override_node = Node(
        package='navigrid_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'k': 0.5,
            'd_min': 0.5,
            'cone_angle_deg': 120.0
        }]
    )

    # Twist Priority Multiplexer Node (/cmd_vel_safety overrides /cmd_vel_nav -> /cmd_vel)
    twist_priority_mux_node = Node(
        package='navigrid_safety',
        executable='twist_priority_mux',
        name='twist_priority_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'safety_timeout_sec': 0.3,
            'nav_timeout_sec': 0.5
        }]
    )

    def launch_gazebo(context, *args, **kwargs):
        is_headless = context.perform_substitution(headless).lower() in ['true', '1']
        gz_cmd = ['gz', 'sim', '-r']
        if is_headless:
            gz_cmd.append('-s')
        gz_cmd.append(world_path)
        return [
            ExecuteProcess(
                cmd=gz_cmd,
                output='screen',
                additional_env={'GZ_IP': '127.0.0.1', 'GZ_TRANSPORT_DISCOVERY_INTERFACE': 'lo'}
            )
        ]

    gazebo_process = OpaqueFunction(function=launch_gazebo)

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        gazebo_process,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        lidar_tf_node,
        imu_tf_node,
        odom_tf_node,
        secondary_amr_node,
        safety_override_node,
        twist_priority_mux_node
    ])
