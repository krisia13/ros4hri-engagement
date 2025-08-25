"""Launch file para iniciar el nodo de control de LEDs del robot.

Este archivo lanza el nodo led_control encargado de gestionar los LEDs según el engagement.

Functions:
    generate_launch_description(): Genera la descripción de lanzamiento con el nodo led_control.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Genera la descripción de lanzamiento para el nodo de control de LEDs.

    Returns:
        LaunchDescription: Descripción con el nodo led_control configurado.
    """
    return LaunchDescription([
        Node(
            package='engagement_action',   # Paquete ROS2 donde está el nodo led_control
            executable='led_control',      # Ejecutable del nodo led_control
            name='led_control',            # Nombre del nodo
            output='screen'                # Salida en pantalla
        )
    ])