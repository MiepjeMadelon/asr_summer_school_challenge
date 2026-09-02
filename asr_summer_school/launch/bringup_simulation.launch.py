from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time", default="true")

	slam_toolbox = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'slam_toolbox.launch.py']
			)
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	teleop = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'teleop.launch.py']
			)
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	apriltag = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_perception'), 'launch', 'apriltag.launch.py']
			)
		),
		launch_arguments={'use_sim_time': use_sim_time}.items()
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
			'active_area_radius': 10.0,
			'use_sim_time': use_sim_time
		}]
	)

	return LaunchDescription([
		slam_toolbox,
		teleop,
		apriltag,
		frontier_detection
	])

