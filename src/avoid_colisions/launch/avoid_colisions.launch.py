from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtén el path del paquete turtlebot3_gazebo
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        # Lanza Gazebo con el mundo por defecto
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
            )
        ),
        # Lanza tu nodo avoid_colisions
        Node(
            package='avoid_colisions',
            executable='avoid_colisions',
            name='avoid_colisions',
            output='screen'
        ),
    ])

