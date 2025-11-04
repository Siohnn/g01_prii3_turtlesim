from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de simulación de la tortuga con fondo blanco
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                'background_r': 255,
                'background_g': 255,
                'background_b': 255
            }]
        ),

        # Nodo que dibuja el número 01
        Node(
            package='g01_prii3_turtlesim',
            executable='draw_number.py',
            name='draw_number_node',
            output='screen'
        )
    ])

