from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='engagement_action',
            executable='tts',
            name='tts',
            output='screen',
            emulate_tty=True,
        )
    ])