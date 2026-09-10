from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
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

	frontier_detection = Node(
		package='asr_summer_school',
		executable='frontier_detection_node_exe',
		name='frontier_detection_node',
		output='screen',
		parameters=[{
			'map_topic': 'map',
			'pose_topic': 'pose',
			'epsilon': 0.5,
			'min_points': 3,
			'min_frontier_size': 20,
			'active_area_radius': 10.0
		}]
	)

	apriltag_detector = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'apriltag.launch.py']
			)
		)
	)

	control = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'control.launch.py']
			)
		)
	)

	return LaunchDescription([
		robot_bringup,
		slam_toolbox,
		teleop,
		camera,
		apriltag,
		frontier_detection,
		apriltag_detector,
		control
	])

