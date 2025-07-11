from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ohms_robot_comms_manager',
            executable='ohms_robot_comms_publisher',
            name='ohms_robot_comms_publisher',
            namespace='bestla',
            output='screen',
            parameters=[{
                'robot_name': 'bestla'
            }]
        )
    ])