from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
	robot_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py']
			)
		)
	)

	slam_toolbox = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'slam_toolbox.launch.py']
			)
		)
	)

	teleop = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'teleop.launch.py']
			)
		)
	)

	camera = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_perception'), 'launch', 'camera.launch.py']
			)
		)
	)

	apriltag = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_perception'), 'launch', 'apriltag.launch.py']
			)
		)
	)

	return LaunchDescription([
		robot_bringup,
		slam_toolbox,
		teleop,
		camera,
		apriltag,
	])

