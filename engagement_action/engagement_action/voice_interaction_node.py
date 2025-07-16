import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from audio_common_msgs.msg import AudioData
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import threading
import numpy as np
import time
import math

class TiagoVoiceInteractionNode(Node):
    def __init__(self):
        super().__init__('tiago_voice_interaction')
        
        # Action Client para Nav2
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers para TIAGo
        self.speech_text_pub = self.create_publisher(String, '/speech_text', 10)
        self.interaction_status_pub = self.create_publisher(Bool, '/interaction_detected', 10)
        self.led_pub = self.create_publisher(String, '/status_led', 10)
        
        # Subscriber para pose actual
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Pose del robot en el mapa
            self.pose_callback,
            10
        )
        
        # Subscribers del ReSpeaker
        self.doa_sub = self.create_subscription(
            PoseStamped,
            '/doa',
            self.doa_callback,
            10
        )
        
        self.vad_sub = self.create_subscription(
            Bool,
            '/vad',
            self.vad_callback,
            10
        )
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Estado
        self.last_direction_angle = 0
        self.voice_active = False
        self.voice_session_active = False
        self.last_interaction_time = 0
        self.interaction_cooldown = 5.0
        self.current_pose = None
        
        # Configurar LEDs iniciales
        self.set_led_status('listen')
        
        # Esperar a que Nav2 esté disponible
        self.get_logger().info('🤖 TIAGo Voice Interaction Node iniciado')
        self.get_logger().info('🎯 Esperando a que Nav2 esté disponible...')
        
        # Verificar conexión con Nav2
        threading.Timer(2.0, self.check_nav2_connection).start()

    def check_nav2_connection(self):
        """Verificar si Nav2 está disponible"""
        if self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('✅ Nav2 conectado y listo')
        else:
            self.get_logger().warn('⚠️ Nav2 no disponible - usando modo fallback')
        self.get_logger().info('🎯 Detectando actividad de voz para rotación')

    def pose_callback(self, msg):
        """Callback para pose actual del robot"""
        self.current_pose = msg.pose.pose
        self.get_logger().debug('📍 Pose actualizada')

    def quaternion_to_respeaker_angle(self, quaternion):
        """Convertir quaternion del DOA a ángulo del ReSpeaker"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        
        while yaw_deg > 180:
            yaw_deg -= 360
        while yaw_deg < -180:
            yaw_deg += 360
        
        if yaw_deg > 100:
            return 180  # Detrás
        elif yaw_deg < -100:
            return 270  # Derecha
        elif yaw_deg > 30:
            return 90   # Izquierda
        else:
            return 0    # Delante

    def doa_callback(self, msg):
        """Callback para dirección de llegada del audio"""
        angle = self.quaternion_to_respeaker_angle(msg.pose.orientation)
        self.last_direction_angle = angle

    def vad_callback(self, msg):
        """Callback para detección de actividad de voz"""
        current_time = time.time()
        
        if msg.data and not self.voice_active:
            if current_time - self.last_interaction_time < self.interaction_cooldown:
                self.get_logger().debug(f'⏰ Cooldown activo')
                return
            
            self.voice_active = True
            self.voice_session_active = True
            
            direction_names = {0: 'delante', 90: 'izquierda', 180: 'detrás', 270: 'derecha'}
            direction_name = direction_names.get(self.last_direction_angle, f'{self.last_direction_angle}°')
            
            self.get_logger().info(f'🎤 Voz detectada desde {direction_name} ({self.last_direction_angle}°)')
            self.set_led_status('active')
            
        elif not msg.data and self.voice_active:
            self.voice_active = False
            
            if self.voice_session_active:
                self.get_logger().info(f'🔄 Fin de voz - Iniciando rotación Nav2')
                threading.Timer(0.5, self.process_voice_interaction).start()

    def process_voice_interaction(self):
        """Procesar interacción de voz"""
        if not self.voice_session_active:
            return
            
        self.voice_session_active = False
        self.last_interaction_time = time.time()
        
        direction_names = {0: 'delante', 90: 'izquierda', 180: 'detrás', 270: 'derecha'}
        direction_name = direction_names.get(self.last_direction_angle, f'{self.last_direction_angle}°')
        
        self.get_logger().info(f'🤖 ¡Procesando interacción con Nav2!')
        self.get_logger().info(f'🎯 Dirección: {direction_name} ({self.last_direction_angle}°)')
        
        # Publicar interacción detectada
        interaction_msg = Bool()
        interaction_msg.data = True
        self.interaction_status_pub.publish(interaction_msg)
        
        # Publicar texto
        text_msg = String()
        text_msg.data = f"Voz detectada desde {direction_name}"
        self.speech_text_pub.publish(text_msg)
        
        # Rotar usando Nav2
        self.rotate_with_nav2(self.last_direction_angle)

    def rotate_with_nav2(self, respeaker_angle):
        """Rotar usando Nav2 NavigateToPose"""
        
        # Mapeo de ángulos
        angle_mappings = {
            0: 0,           # Delante → no girar
            90: math.pi/2,  # Izquierda → girar +90°
            180: math.pi,   # Detrás → girar 180°
            270: -math.pi/2 # Derecha → girar -90°
        }
        
        target_yaw_delta = angle_mappings.get(respeaker_angle, 0)
        
        if abs(target_yaw_delta) < 0.1:
            self.get_logger().info('✅ Ya estoy mirando hacia ti')
            self.set_led_status('success')
            return
        
        # Obtener pose actual
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('❌ No se pudo obtener pose actual')
            return
        
        # Calcular nueva orientación
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)
        target_yaw = current_yaw + target_yaw_delta
        
        # Normalizar ángulo
        while target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2 * math.pi
        
        direction = "izquierda" if target_yaw_delta > 0 else "derecha"
        angle_degrees = math.degrees(abs(target_yaw_delta))
        
        self.get_logger().info(f'🔄 Nav2: Girando {angle_degrees:.1f}° hacia la {direction}')
        self.get_logger().info(f'📍 Pose actual: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}, {math.degrees(current_yaw):.1f}°)')
        self.get_logger().info(f'🎯 Nueva orientación: {math.degrees(target_yaw):.1f}°')
        
        # Crear goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Mantener la misma posición, solo cambiar orientación
        goal_pose.pose.position.x = current_pose.position.x
        goal_pose.pose.position.y = current_pose.position.y
        goal_pose.pose.position.z = current_pose.position.z
        goal_pose.pose.orientation = self.yaw_to_quaternion(target_yaw)
        
        # Crear y enviar goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.set_led_status('processing')
        
        # Verificar si el servidor está disponible
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('❌ Nav2 no disponible')
            return
        
        # Enviar goal
        self.get_logger().info('📤 Enviando goal a Nav2...')
        future = self.nav_action_client.send_goal_async(goal_msg)
        
        # Esperar resultado con timeout
        start_time = time.time()
        timeout = 15.0  # 15 segundos timeout
        
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warn('⏰ Timeout esperando respuesta de Nav2')
                return
            time.sleep(0.1)
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado por Nav2')
            return
        
        self.get_logger().info('✅ Goal aceptado por Nav2, esperando resultado...')
        
        # Esperar resultado
        result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().warn('⏰ Timeout esperando resultado de Nav2')
                goal_handle.cancel_goal_async()
                return
            time.sleep(0.1)
        
        result = result_future.result().result
        self.get_logger().info(f'📊 Resultado Nav2: {result}')
        
        self.get_logger().info('✅ Rotación Nav2 completada - ¡Ahora te estoy mirando!')
        
        # LEDs de confirmación
        self.set_led_status('success')
        threading.Timer(2.0, lambda: self.set_led_status('listen')).start()

    def get_current_pose(self):
        """Obtener pose actual del robot"""
        if self.current_pose is not None:
            return self.current_pose
        
        # Intentar obtener desde TF
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = type('Pose', (), {})()
            pose.position = transform.transform.translation
            pose.orientation = transform.transform.rotation
            return pose
            
        except Exception as e:
            self.get_logger().error(f'❌ Error obteniendo pose: {e}')
            return None

    def quaternion_to_yaw(self, quaternion):
        """Convertir quaternion a yaw"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convertir yaw a quaternion"""
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    def set_led_status(self, status):
        """Establecer estado de LEDs del ReSpeaker"""
        msg = String()
        msg.data = status
        self.led_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TiagoVoiceInteractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Deteniendo nodo...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()