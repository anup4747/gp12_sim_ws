from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_gp12_gazebo = FindPackageShare('gp12_gazebo')
    pkg_ros_gz_sim  = FindPackageShare('ros_gz_sim')

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

    # ── Gazebo (full sim + GUI) ─────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ' -r -v 4 empty.sdf',   # -r = run, -v 4 = verbose, empty world
            'use_sim_time': use_sim_time
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

    # ── Spawn robot in Gazebo ───────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'motoman_gp12',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'   # slight lift to avoid initial penetration
        ],
        output='screen'
    )

    # ── ros2_control / controller_manager ───────────────────────────────────────
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            PathJoinSubstitution([pkg_gp12_gazebo, 'config', 'gp12_controllers.yaml'])
        ],
        output='both'
    )

    # ── Load controllers ────────────────────────────────────────────────────────
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ── Clock bridge (recommended for sim time sync) ─────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

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
        controller_manager,
        load_joint_state_broadcaster,
        load_arm_controller,
        clock_bridge,
    ])