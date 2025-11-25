from setuptools import setup

package_name = 'avoid_colisions'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/avoid_colisions.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_correo@example.com',
    description='Control de TurtleBot con detección de obstáculos mediante LIDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avoid_colisions = avoid_colisions.avoid_colisions:main',
        ],
    },
)

