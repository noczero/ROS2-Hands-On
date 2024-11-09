from setuptools import find_packages, setup

package_name = 'aptrg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'number_subscriber_node = aptrg.number_subscriber_node:main'
        ],
    },
)
