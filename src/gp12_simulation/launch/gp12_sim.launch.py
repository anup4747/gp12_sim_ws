from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Launch arguments ────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── Package paths ───────────────────────────────────────────────────────────
    pkg_gp12_simulation = FindPackageShare('gp12_simulation')

    # ── Robot description (process xacro) ───────────────────────────────────────
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
            'robot_description': ParameterValue(robot_description_content, value_type=str),
        }]
    )

    # ── Joint state publisher GUI (for manual motion in sim) ────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # ── Hyper IMU listener node (simulation only) ──────────────────────────────
    hyper_imu_node = Node(
        package='gp12_simulation',
        executable='script.py',
        name='hyper_imu_listener',
        output='screen',
        parameters=[{
            'udp_port': 5555,
            'frame_id': 'imu_link',
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        hyper_imu_node,
    ])

