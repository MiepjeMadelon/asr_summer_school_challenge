from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory


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

	apriltag = Node(
		package='apriltag_ros',
		executable='apriltag_node',
		namespace='/camera',
		name='apriltag',
		output='screen',
		parameters=[
			{'use_sim_time': use_sim_time},
			os.path.join(get_package_share_directory('turtlebot3_perception'), "config", "apriltag.yaml"
    )],
		remappings=[('image_rect', 'image_raw')]
	)

	detection2landmark = Node(
		package='turtlebot3_perception',
		executable='detection2landmark',
		namespace='/camera',
		output='screen',
		parameters=[
			{'use_sim_time': use_sim_time},
			{'robot_base_frame': 'base_link'}
		]
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

	return LaunchDescription([
		slam_toolbox,
		teleop,
		apriltag,
		detection2landmark,
		frontier_detection,
		apriltag_detector,
		control
	])

