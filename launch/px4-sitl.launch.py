from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14540@0.0.0.0:14557',
                'gcs_url': 'udp://@77.22.9.4:14550',
                'target_system_id': 1,
                'target_component_id': 1, 
            }]
        ),
    ])
