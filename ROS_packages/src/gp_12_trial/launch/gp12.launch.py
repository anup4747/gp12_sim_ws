from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
	moveit_demo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([
				FindPackageShare('gp12_moveit_config'),
				'launch',
				'demo.launch.py',
			])
		)
	)

	return LaunchDescription([moveit_demo])
