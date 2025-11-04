from setuptools import setup
from glob import glob

package_name = 'g01_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],  # solo setuptools, no rclpy
    zip_safe=True,
    maintainer='domenico',
    maintainer_email='damilici@etsii.upv.es',
    description='Nodo que dibuja el número de grupo en turtlesim',
    license='MIT',
    tests_require=['pytest'],
    scripts=['g01_prii3_turtlesim/draw_number.py'],  # ejecutable directo
)

