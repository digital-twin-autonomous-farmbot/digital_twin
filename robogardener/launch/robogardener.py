from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = 'false'

    pkg_dir = get_package_share_directory('robogardener')

    urdf_file_path = os.path.join(pkg_dir, 'description', 'robo.urdf.xacro')
    control_yaml = os.path.join(pkg_dir, 'config', 'control.yaml')


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use simulation (Gazebo) clock if true'),

        # Node for robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', urdf_file_path])
            }]
        ),
        
        # State Publisher
        Node(
            package='nodes',
            executable='state_publisher',
            name='state_publisher',
            output='screen'
        ),
        ])