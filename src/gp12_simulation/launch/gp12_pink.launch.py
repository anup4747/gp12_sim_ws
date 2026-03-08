from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_gp12_simulation = FindPackageShare('gp12_simulation')
    urdf_path = PathJoinSubstitution([pkg_gp12_simulation, 'urdf', 'gp12.urdf'])

    # ── Robot description (process xacro for Rviz) ──────────────────────────────
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_gp12_simulation,
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

    # ── Pink IK Node ────────────────────────────────────────────────────────────
    pink_ik_node = Node(
        package='gp12_simulation',
        executable='pink_ik_node.py',
        name='pink_ik_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'udp_port': 5555,
            'ee_frame': 'tool0'
        }]
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        robot_state_publisher,
        rviz_node,
        pink_ik_node
    ])
