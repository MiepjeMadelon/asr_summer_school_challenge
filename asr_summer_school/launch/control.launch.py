from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os   


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='asr_summer_school',
            executable='control.py',
            name='control',
            parameters=[]
        ),
    ])
