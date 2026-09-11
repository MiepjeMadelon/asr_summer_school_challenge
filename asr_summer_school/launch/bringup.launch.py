from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def asr(name, **kwargs):
	return IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([FindPackageShare('asr_summer_school'), 'launch', name])
		),
		**kwargs
	)


def generate_launch_description():
	# sul robot vero il clock e' quello di sistema
	use_sim_time = LaunchConfiguration('use_sim_time')
	sim_arg = {'use_sim_time': use_sim_time}.items()

	robot_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py']
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
			'active_area_radius': 10.0,
			'use_sim_time': use_sim_time
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

	nav2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'nav2.launch.py']
			)
		)
	)

	mission_controller = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'mission_controller.launch.py']
			)
		)
	)


	return LaunchDescription([
		DeclareLaunchArgument('use_sim_time', default_value='false'),
		robot_bringup,
		asr('slam_toolbox.launch.py', launch_arguments=sim_arg),
		camera,
		apriltag,
		frontier_detection,
		asr('nav2.launch.py', launch_arguments=sim_arg),
		asr('apriltag.launch.py', launch_arguments=sim_arg),
		asr('control.launch.py', launch_arguments=sim_arg),
		asr('mission.launch.py', launch_arguments=sim_arg),
	])
