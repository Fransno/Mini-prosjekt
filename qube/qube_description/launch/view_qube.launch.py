import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Locate the qube_description package 
    pkg_path = get_package_share_directory('qube_description')
    
    # Build the full path to the URDF (xacro) file 
    urdf_file = os.path.join(pkg_path, 'urdf', 'qube.urdf.xacro')
    
    # Robot State Publisher, publishes TF (transform) frames based on the URDF structure
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False, # Set to True if you're running in a simulated clock
            'robot_description': Command(['xacro ', urdf_file]) # Run xacro to generate URDF
        }],
    )
    
    # Joint State Publisher
    # GUI slider to manually manipulate joint angles (for testing and visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # RViz2, responisible for visualisation
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'qube.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': False}],
    )

    # Return a list of all nodes to be launched
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
