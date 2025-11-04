from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')

    # Lanzar Gazebo vacío
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    # Spawn del robot
    spawn_turtlebot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-database', 'turtlebot3_burger',
            '-entity', 'turtlebot3_burger',
            '-x', '0', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )

    # Nodo de movimiento
    move_turtlebot_node = Node(
        package='g07_prii3_move_turtlebot',
        executable='move_turtlebot.py',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_turtlebot,
        move_turtlebot_node
    ])

