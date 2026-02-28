from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]
        ),
        Node(
            package='can_motor_driver',
            executable='motor_node',
            name='motor_node',
            output='screen'
        )
    ])
