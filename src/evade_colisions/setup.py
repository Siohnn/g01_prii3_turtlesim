import os
from glob import glob
from setuptools import setup

package_name = 'evade_colisions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # ** LÍNEA CORREGIDA **
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.py')) + glob(os.path.join('launch', '*launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre', 
    maintainer_email='tu_email@example.com', 
    description='Paquete de Python para la lógica de evitación de colisiones en ROS 2.',
    license='TODO: License declaration', 
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'evade_colisions_node = evade_colisions.evade_colisions:main',
        ],
    },
)
