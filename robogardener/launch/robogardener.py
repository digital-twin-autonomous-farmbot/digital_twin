from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = True
    pkg_dir = get_package_share_directory('robogardener')
    
    # Path definitions
    urdf_file = os.path.join(pkg_dir, 'description', 'robo.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    control_yaml = os.path.join(pkg_dir, 'config', 'control.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    
    robot_description_content = Command(
        ['xacro ', urdf_file, ' use_mock_hardware:=false']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0
            }],
            output='screen'
        ),

        # Joint State Publisher (optional, but useful for debugging)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Spawn Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description', '-name', 'robogardener'],
            output='screen'
        ),

        # ROS-Gazebo Bridge (for cmd_vel and odom)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        ),

        # ROS2 Control (only for joint_state_broadcaster)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description_content},
                control_yaml,
                {'use_sim_time': use_sim_time},
            ],
            output='screen'
        ),

        # Joint State Broadcaster (for TF)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'params_file': nav2_params
            }.items()
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])