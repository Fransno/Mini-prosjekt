import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# To run xacro commands at launch time
from launch.substitutions import Command

# To include other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths
    pkg_bringup = get_package_share_directory('qube_bringup')
    pkg_driver = get_package_share_directory('qube_driver')
    
    # URDF
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'controlled_qube.urdf.xacro')
    
    # ROS2 Control config
    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    # Robot State Publisher
    # This publishes the transforms (TFs) from the URDF to the rest of ROS
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False, # Set to True if you're running a simulation
            'robot_description': Command(['xacro ', urdf_file]) # Run xacro and parse at launch time
        }]
    )

    # Include Qube driver launch
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_driver, 'launch', 'qube_driver.launch.py')
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'qube.rviz')],
        output='screen'
    )

    # Final launch description
    return LaunchDescription([
        robot_state_publisher,
        qube_driver_launch,
        #joint_state_broadcaster,
        #velocity_controller,
        rviz_node
    ])
