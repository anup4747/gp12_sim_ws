from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction

def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_gp12_gazebo = FindPackageShare('gp12_gazebo')
    pkg_gazebo_ros   = FindPackageShare('gazebo_ros')

    # ── Robot description (process xacro) ───────────────────────────────────────
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_gp12_gazebo,
            'urdf',
            'gp12.xacro'
        ]),
        # Add any xacro args here if needed, e.g.:
        # ' prefix:=gp12_ use_gazebo:=true'
    ])

    # ── Gazebo Classic (server + GUI) ───────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_gazebo_ros,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_gazebo_ros, 'worlds', 'empty.world']),
            'paused': 'false',
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'headless': 'false',
            'debug': 'false',
            # Optional: extra_gazebo_args: '-v 4' for verbose
        }.items()
    )

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

    # ── Spawn robot in Gazebo Classic ───────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'motoman_gp12',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'   # slight lift to avoid initial penetration
        ],
        output='screen'
    )

    # ── ros2_control_node (optional but recommended for Humble + separate CM) ───
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': ParameterValue(robot_description_content, value_type=str)},
    #         PathJoinSubstitution([pkg_gp12_gazebo, 'config', 'gp12_controllers.yaml'])
    #     ],
    #     output='both'
    # )

    # ── Load controllers (spawners) ─────────────────────────────────────────────
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Wrap spawners in TimerAction (e.g., wait 5–10 seconds after spawn)
  

    # ── Launch description ──────────────────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        # TimerAction(
        #     period=5.0,
        #     actions=[load_joint_state_broadcaster]
        # ),
        # TimerAction(
        #     period=6.0,
        #     actions=[load_arm_controller]
        # ),
        load_joint_state_broadcaster,
        load_arm_controller
    ])