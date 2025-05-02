from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = True
    pkg_dir = get_package_share_directory('robogardener')
    
    # Path definitions
    urdf_file = os.path.join(pkg_dir, 'description', 'robo.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    control_yaml = os.path.join(pkg_dir, 'config', 'control.yaml')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')  # Make sure this exists
    
    robot_description_content = Command(
        ['xacro ', urdf_file, ' use_mock_hardware:=false']
    )
    robot_description = {'robot_description': robot_description_content}

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
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 50  # Match controller update rate
            }],
            output='screen'
        ),

        # Spawn Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'robogardener',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # ROS-Gazebo Bridge for basic topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.025', '0', '0', '0', 'world', 'odom'],
            output='screen'
        ),


        # ROS2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                control_yaml,  # Load entire control config file
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        LogInfo(msg="Controller manager is starting..."),

        # Spawn controllers after delay
        TimerAction(
            period=5.0,  # Reduced from 15s to 5s
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen'
                ),
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])