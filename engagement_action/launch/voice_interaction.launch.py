"""Launch file para iniciar los nodos de detección y procesamiento de voz.

Este archivo lanza el nodo respeaker_ros para la adquisición de audio y el nodo
de interacción de voz para procesar eventos y direcciones detectadas.

Functions:
    generate_launch_description(): Genera la descripción de lanzamiento con los nodos necesarios.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Genera la descripción de lanzamiento para los nodos de voz.

    Returns:
        LaunchDescription: Descripción con los nodos respeaker_ros y voice_interaction_node.
    """
    return LaunchDescription([
        # Nodo respeaker_ros para adquisición y procesamiento de audio
        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker',
            output='screen',
            parameters=[{
                'device_index': 2,  # Índice del dispositivo de audio (ajustar según el sistema)
                'channels': 6,      # Número de canales de micrófono
                'sample_rate': 16000,  # Frecuencia de muestreo en Hz
                'publish_audio': True, # Publicar datos de audio
                'publish_vad': True,   # Publicar detección de voz
                'publish_doa': True,   # Publicar dirección de llegada de audio
            }]
        ),
        
        # Nodo de interacción de voz para procesar eventos y direcciones
        Node(
            package='engagement_action',
            executable='voice_interaction_node',
            name='tiago_voice_interaction',
            output='screen',
            parameters=[{
                'wake_words': ['tiago', 'hola', 'oye', 'robot', 'hola tiago', 'ey tiago'], # Palabras clave para activar
                'recognition_language': 'es-ES',  # Idioma de reconocimiento
                'max_rotation_speed': 0.6,        # Velocidad máxima de rotación del robot
                'audio_buffer_duration': 4.0,     # Duración del buffer de audio en segundos
                'min_audio_duration': 0.5,        # Duración mínima de audio para procesar
            }]
        ),
    ])