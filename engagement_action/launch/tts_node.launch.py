"""Launch file para iniciar el nodo de síntesis de voz (TTS).

Este archivo lanza el nodo TTS encargado de la síntesis de voz en el paquete engagement_action.

Functions:
    generate_launch_description(): Genera la descripción de lanzamiento con el nodo TTS.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Genera la descripción de lanzamiento para el nodo TTS.

    Returns:
        LaunchDescription: Descripción con el nodo TTS configurado.
    """
    return LaunchDescription([
        Node(
            package='engagement_action',   # Paquete ROS2 donde está el nodo TTS
            executable='tts',              # Ejecutable del nodo TTS
            name='tts',                    # Nombre del nodo
            output='screen',               # Salida en pantalla
            emulate_tty=True,              # Emulación de terminal para mejor visualización
        )
    ])