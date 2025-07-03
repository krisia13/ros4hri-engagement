from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='engagement_action',
            executable='engagement_action',
            name='engagement_action',
            output='screen',
            emulate_tty=True,
        )
    ])