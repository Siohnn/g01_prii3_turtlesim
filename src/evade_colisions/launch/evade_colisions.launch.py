import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Esta función es OBLIGATORIA y debe llamarse EXACTAMENTE así.
def generate_launch_description():
    
    # 1. Definir rutas y variables
    
    # Obtiene la ruta de instalación del paquete turtlebot3_gazebo
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Configura el modelo del robot (debe ser 'burger', 'waffle' o 'waffle_pi')
    # Usamos os.environ para leer la variable de entorno, si existe
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # 2. Argumentos (Opcional, pero bueno para la modularidad)
    
    # Argumento para el nombre del mundo de Gazebo
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            turtlebot3_gazebo_dir,
            'worlds',
            'turtlebot3_world.world'  # El mundo por defecto del TurtleBot3
        ]),
        description='Path to the Gazebo world file'
    )

    # 3. Lanzar Gazebo y el Robot
    
    # Incluye el launch file que inicia Gazebo y spawnea el TurtleBot3 Burger.
    # Este archivo se encuentra dentro del paquete 'turtlebot3_gazebo'.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot3_gazebo_dir,
                'launch',
                'turtlebot3_world.launch.py' 
            ])
        ),
        # Pasa el modelo del robot como argumento al launch de Gazebo
        launch_arguments={'model': TURTLEBOT3_MODEL}.items()
    )

    # 4. Lanzar tu Nodo de Python
    
    # Lanza el nodo 'evade_colisions_node' definido en tu setup.py
    evade_colisions_node = Node(
        package='evade_colisions',
        executable='evade_colisions_node', # Nombre del entry_point en setup.py
        name='evade_colisions_manager',     # Nombre visible en ros2 node list
        output='screen',                    # Muestra los logs en la consola
        # Si tu nodo necesitara parámetros:
        # parameters=[{'parametro_1': 1.5}], 
        # remappings=[('/cmd_vel', '/turtlebot3/cmd_vel')] # Ejemplo de remapeo
    )

    # 5. Retorna la descripción del lanzamiento (REQUIRED)
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        evade_colisions_node
    ])