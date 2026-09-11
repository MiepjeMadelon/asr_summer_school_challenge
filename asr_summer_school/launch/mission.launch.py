from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # deadline della prova
        DeclareLaunchArgument('mission_duration', default_value='240.0'),
        # velocita' media misurata sul campo: in dubbio sottostimare, se e'
        # troppo alta il rientro parte tardi e si perdono 150 punti.
        DeclareLaunchArgument('avg_speed', default_value='0.12'),

        Node(
            package='asr_summer_school',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'mission_duration': LaunchConfiguration('mission_duration'),
                'avg_speed': LaunchConfiguration('avg_speed'),
                'n_tags_target': 11,
                'return_margin': 15.0,
                'safety_factor': 1.5,
                'goal_timeout': 30.0,
                'min_cluster': 6,
                # coerente con inflation_radius del costmap Nav2
                'min_obstacle_dist': 0.25,
            }],
        ),
    ])
