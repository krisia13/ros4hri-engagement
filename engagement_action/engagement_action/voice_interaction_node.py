"""Nodo ROS2 para detección de voz y dirección usando ReSpeaker.

Este nodo detecta eventos de voz y calcula la dirección de origen usando el sensor ReSpeaker.
Reporta los eventos al Behavior Tree (BT) mediante los tópicos '/interaction_detected' y '/voice_direction'.
Bloquea la detección si el engagement está activo.

Attributes:
    TiagoVoiceInteractionNode (Node): Nodo principal para la lógica de interacción por voz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped
import math
import time

class TiagoVoiceInteractionNode(Node):
    """Nodo para detectar voz y calcular dirección usando ReSpeaker."""

    def __init__(self):
        """Inicializa el nodo TiagoVoiceInteractionNode.

        Configura publishers, suscripciones y estado interno.
        """
        super().__init__('tiago_voice_interaction')
        
        # Publisher para notificar interacción detectada al BT
        self.interaction_status_pub = self.create_publisher(Bool, '/interaction_detected', 10)
        # Publisher para reportar dirección de la voz al BT
        self.voice_direction_pub = self.create_publisher(Int32, '/voice_direction', 10)
        
        # Suscripción al estado de engagement para bloqueo
        self.engagement_sub = self.create_subscription(
            Int32, '/engagement/general_status', self.engagement_callback, 10)
        
        # Suscripción a dirección de llegada de audio (DOA)
        self.doa_sub = self.create_subscription(PoseStamped, '/doa', self.doa_callback, 10)
        # Suscripción a detección de voz (VAD)
        self.vad_sub = self.create_subscription(Bool, '/vad', self.vad_callback, 10)
        
        # Estado interno del nodo
        self.current_engagement_level = 0
        self.engagement_active = False
        self.last_direction_angle = 0  # Última dirección detectada (int)
        self.voice_active = False
        self.last_interaction_time = 0
        self.interaction_cooldown = 3.0  # Segundos de espera entre interacciones
        
        self.get_logger().info('🎤 Voice Detection Node - SOLO SENSOR')
        self.get_logger().info('📡 Reportando a BT: /interaction_detected, /voice_direction')

    def engagement_callback(self, msg):
        """Callback para actualizar el estado de engagement.

        Args:
            msg (Int32): Mensaje con el nivel general de engagement.
        """
        self.current_engagement_level = msg.data
        self.engagement_active = (msg.data >= 2)

    def doa_callback(self, msg):
        """Callback para procesar la dirección de llegada de audio (DOA).

        Args:
            msg (PoseStamped): Mensaje con la orientación detectada.
        """
        if self.engagement_active:
            return  # Bloqueo: no actualizar dirección si hay engagement
            
        angle = self.quaternion_to_respeaker_angle(msg.pose.orientation)
        self.last_direction_angle = int(angle)  # ASEGURAR QUE ES INT
        
        self.get_logger().debug(f'📍 DOA actualizado: {self.last_direction_angle}°')

    def vad_callback(self, msg):
        """Callback para procesar eventos de detección de voz.

        Args:
            msg (Bool): Mensaje que indica si hay voz detectada.
        """
        if self.engagement_active:
            return  # Bloqueo: no reportar voz si hay engagement
            
        current_time = time.time()
        
        if msg.data and not self.voice_active:
            # Si hay voz y no estaba activa, verificar cooldown
            if current_time - self.last_interaction_time < self.interaction_cooldown:
                self.get_logger().debug('⏰ Cooldown activo')
                return
                
            self.voice_active = True
            
            # Validar dirección
            if self.last_direction_angle is None:
                self.last_direction_angle = 0  # Por defecto: delante
                self.get_logger().warn('⚠️ Sin dirección DOA - usando delante (0°)')
            
            # Mapear dirección a nombre legible
            direction_names = {0: 'delante', 90: 'izquierda', 180: 'detrás', 270: 'derecha'}
            direction_name = direction_names.get(self.last_direction_angle, f'{self.last_direction_angle}°')
            
            self.get_logger().info(f'🎤 Voz detectada desde {direction_name} ({self.last_direction_angle}°) - reportando a BT')
            
            # Publicar mensajes al BT
            try:
                interaction_msg = Bool()
                interaction_msg.data = True
                self.interaction_status_pub.publish(interaction_msg)
                
                direction_msg = Int32()
                direction_msg.data = int(self.last_direction_angle)  # CAST explícito
                self.voice_direction_pub.publish(direction_msg)
                
                self.get_logger().debug(f'✅ Mensajes enviados - interaction: True, direction: {direction_msg.data}')
                
            except Exception as e:
                self.get_logger().error(f'❌ Error enviando mensajes: {e}')
                self.get_logger().error(f'❌ last_direction_angle type: {type(self.last_direction_angle)}, value: {self.last_direction_angle}')
            
        elif not msg.data and self.voice_active:
            # Fin de evento de voz
            self.voice_active = False
            self.last_interaction_time = current_time
            self.get_logger().debug('🔇 Fin de voz detectado')

    def quaternion_to_respeaker_angle(self, quaternion):
        """Convierte un quaternion a un ángulo principal del ReSpeaker.

        Args:
            quaternion: Orientación en formato quaternion.

        Returns:
            int: Ángulo principal (0: delante, 90: izquierda, 180: detrás, 270: derecha).
        """
        try:
            siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw_rad)
            
            # Normalizar ángulo a [-180, 180]
            while yaw_deg > 180:
                yaw_deg -= 360
            while yaw_deg < -180:
                yaw_deg += 360
            
            # Mapear a 4 direcciones principales
            if yaw_deg > 100:
                return 180  # Detrás
            elif yaw_deg < -100:
                return 270  # Derecha
            elif yaw_deg > 30:
                return 90   # Izquierda
            else:
                return 0    # Delante
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en quaternion_to_respeaker_angle: {e}')
            return 0  # Fallback seguro

def main(args=None):
    """Función principal para iniciar el nodo TiagoVoiceInteractionNode.

    Args:
        args (list, optional): Argumentos de inicialización ROS2.
    """
    rclpy.init(args=args)
    node = TiagoVoiceInteractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Voice sensor detenido')
    except Exception as e:
        node.get_logger().error(f'❌ Error en main: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()