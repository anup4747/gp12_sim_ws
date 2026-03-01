from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default='')

    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_gp12_gazebo = FindPackageShare('gp12_gazebo')

    # ── Robot description (process xacro) ───────────────────────────────────────
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_gp12_gazebo,
            'urdf',
            'gp12.xacro'
        ]),
    ])

    # ── Robot state publisher ───────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_file,
            '--ros-args',
            '--log-level', 'info'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ── Optional: default RViz config if none provided ──────────────────────────
    default_rviz_config = PathJoinSubstitution([
        pkg_gp12_gazebo,
        'rviz',
        'gp12_default.rviz'
    ])

    # If you want a default config, create this file later (see note below)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='Full path to RViz config file (optional)'
        ),

        robot_state_publisher,
        rviz_node,
    ])