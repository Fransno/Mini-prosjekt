import os
from ament_index_python.packages import get_package_share_directory

# ROS2 Launch imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare configurable launch arguments
    # These let you override values from the command line without changing code
    device_arg = DeclareLaunchArgument('device', default_value='/dev/ttyACM0')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='true')

    # Locate the URDF (robot description) file 
    pkg_path = get_package_share_directory('qube_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'qube.urdf.xacro')

    # Put in launch parameters (device, baud_rate, simulation) into xacro at runtime
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' device:=', LaunchConfiguration('device'),
            ' baud_rate:=', LaunchConfiguration('baud_rate'),
            ' simulation:=', LaunchConfiguration('simulation')
        ]),
        value_type=str
    )

    # Publishes the robot's TF tree (transforms) from the URDF description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('simulation'), # Use simulation clock if simulation:=true
            'robot_description': robot_description # Provide the processed URDF
        }],
        output='screen'
    )

    # Provides a simple GUI to manually move joints (for simulation / testing)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # Starts RViz2 with basic settings
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('simulation')    # RViz also listens to sim time if enabled
        }]
    )

    # Launch everything: the arguments + the nodes
    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
