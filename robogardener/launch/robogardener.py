from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_dir = get_package_share_directory('robogardener')

    urdf_file_path = os.path.join(pkg_dir, 'description', 'robo.urdf.xacro')
    control_yaml = os.path.join(pkg_dir, 'config', 'control.yaml')


    return LaunchDescription([
        # Node for robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path])
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        ################################################################################################
        # Node for RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_link'],
            output='screen'
        ),


    ])
