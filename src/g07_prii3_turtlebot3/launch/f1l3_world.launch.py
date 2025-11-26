#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Mapa por defecto: $HOME/f1l3_map.yaml
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(os.environ['HOME'], 'f1l3_map.yaml')
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_navigation2 = get_package_share_directory('turtlebot3_navigation2')

    return LaunchDescription([

        # --- GAZEBO SERVER ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(
                    get_package_share_directory('g07_prii3_turtlebot3'),
                    'worlds',
                    'F1L3_world.world'
                )
            }.items(),
        ),

        # --- GAZEBO GUI ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # --- ROBOT STATE PUBLISHER ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
        ),

        # --- NAVIGATION2 + RVIZ ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_navigation2, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
            }.items(),
        ),
    ])

