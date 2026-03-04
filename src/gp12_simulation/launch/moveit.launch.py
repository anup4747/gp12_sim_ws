# moveit.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find package shares
    pkg_gp12_simulation = FindPackageShare('gp12_simulation').find('gp12_simulation')
    pkg_moveit_config = FindPackageShare('gp12_moveit_config').find('gp12_moveit_config')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description from XACRO
    robot_description_content = ParameterValue(
        Command(['xacro ', os.path.join(pkg_gp12_simulation, 'urdf/gp12.xacro')]),
        value_type=str
    )

    # Robot semantic description (SRDF)
    robot_description_semantic_content = ParameterValue(
        Command(['xacro ', os.path.join(pkg_moveit_config, 'config/gp12.srdf.xacro')]),
        value_type=str
    )

    # Joint limits YAML (optional)
    joint_limits_file = os.path.join(pkg_moveit_config, 'config/joint_limits.yaml')

    # Robot controllers YAML (optional)
    moveit_controller_file = os.path.join(pkg_moveit_config, 'config/moveit_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
        ),

        # MoveIt MoveGroup node
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description_content},
                {'robot_description_semantic': robot_description_semantic_content},
                joint_limits_file,
                moveit_controller_file
            ]
        ),

        # RViz for MoveIt
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_moveit_config, 'launch/moveit.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])