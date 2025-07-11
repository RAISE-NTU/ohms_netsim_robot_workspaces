from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ohms_robot_comms_manager',
            executable='ohms_robot_comms_subscriber',
            name='ohms_robot_comms_subscriber',
            namespace='atlas',
            output='screen',
            parameters=[{
                'robot_name': 'atlas',
                'other_robots': ['bestla'],
                'link_quality_threshold': 5.0,
                'window_size': 10
            }]
        )
    ])