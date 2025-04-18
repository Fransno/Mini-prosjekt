import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments for hardware vs simulation setup
    device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyACM0', description='Serial device path for Qube hardware')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200', description='Baud rate for serial communication')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='true', description='Simulation mode (true) or hardware (false)')

    # Get package paths
    pkg_bringup = get_package_share_directory('qube_bringup')
    pkg_driver = get_package_share_directory('qube_driver')

    # Path to URDF (xacro)
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'controlled_qube.urdf.xacro')

    # Create robot_description by running xacro with args
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file, ' ',
            'device:=', LaunchConfiguration('device'), ' ',
            'baud_rate:=', LaunchConfiguration('baud_rate'), ' ',
            'simulation:=', LaunchConfiguration('simulation')
        ]),
        value_type=str
    )

    # robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('simulation'),
            'robot_description': robot_description
        }]
    )

    # Include Qube driver launch (handles controller_manager, hardware interface, etc.)
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_driver, 'launch', 'qube_driver.launch.py')
        )
    )

    # RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'qube.rviz')],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('simulation')}]
    )

    # Final LaunchDescription
    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        robot_state_publisher,
        qube_driver_launch,
        rviz_node
    ])
