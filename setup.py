from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aptrg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noczero',
    maintainer_email='noczero@todo.todo',
    description='Hands on with Robot Operating System 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_generator_node = aptrg.number_generator_node:main',
            'number_subscriber_node = aptrg.number_subscriber_node:main',
            'turtlesim_controller_node = aptrg.turtlesim_controller_node:main',
            'px4_controller_node = aptrg.px4_controller_node:main',
            'px4_takeoff_node = aptrg.px4_takeoff_node:main',
            'square_mission_node = aptrg.square_mission_node:main',
            'px4_square_mission_node = aptrg.px4_square_mission_node:main'
        ],
    },
)
