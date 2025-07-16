from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo respeaker_ros
        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker',
            output='screen',
            parameters=[{
                'device_index': 2,  # Ajustar según tu sistema
                'channels': 6,
                'sample_rate': 16000,
                'publish_audio': True,
                'publish_vad': True,
                'publish_doa': True,
            }]
        ),
        
        # Nodo de interacción de voz
        Node(
            package='engagement_action',
            executable='voice_interaction_node',
            name='tiago_voice_interaction',
            output='screen',
            parameters=[{
                'wake_words': ['tiago', 'hola', 'oye', 'robot', 'hola tiago', 'ey tiago'],
                'recognition_language': 'es-ES',
                'max_rotation_speed': 0.6,
                'audio_buffer_duration': 4.0,
                'min_audio_duration': 0.5,
            }]
        ),
    ])