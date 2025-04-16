import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyACM0')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='true')

    # Get package paths
    pkg_bringup = get_package_share_directory('qube_bringup')
    pkg_driver = get_package_share_directory('qube_driver')

    # URDF file with hardware integration
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'controlled_qube.urdf.xacro')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('simulation'),
            'robot_description': Command([
                'xacro ', urdf_file,
                ' device:=', LaunchConfiguration('device'),
                ' baud_rate:=', LaunchConfiguration('baud_rate'),
                ' simulation:=', LaunchConfiguration('simulation')
            ])
        }],
        output='screen'
    )

    # Driver and controller manager
    driver_launch = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('simulation')
        }],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Velocity controller
    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        robot_state_publisher,
        driver_launch,
        joint_state_broadcaster,
        velocity_controller
    ])
