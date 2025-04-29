from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from launch.substitutions import Command
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True  # Boolean statt String (True für Verwendung der Simulationszeit)

    pkg_dir = get_package_share_directory('robogardener')
    urdf_file_path = os.path.join(pkg_dir, 'description', 'robo.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file_path])
    robot_description = {'robot_description': robot_description_content}
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    control_yaml = os.path.join(pkg_dir, 'config', 'control.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Standardwert als String 'true'
            description='Use simulation (Gazebo) clock if true'),

        # Gazebo starten (gz_sim)
        # Gazebo starten mit "gz sim"
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            arguments=[
            '-topic', '/robot_description',  # Beschreibung des Roboters
            '-name', 'robogardener'  # Name des Roboters in Gazebo
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file_path]),
                'use_sim_time': use_sim_time  # Boolean statt String
            }]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # RViz starten
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                robot_description,                            
                {'use_sim_time': use_sim_time},
                control_yaml
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # Log info message
        LogInfo(
            msg="Controller manager is starting..."
        ),

        # Log info message
        LogInfo(
            msg="Waiting for the controller manager service to become available..."
        ),
        
        # Delay action: wait 15 seconds before spawning controllers
        TimerAction(
            period=15.0,  # wait for 15 seconds
            actions=[
                # Spawner nodes
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['diff_drive_controller']
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=['joint_state_broadcaster']
                ),
            ]
        ),
    ])
