#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from text_to_speech_msgs.action import TTS
import random
import time

class TTSEngagementNode(Node):
    def __init__(self):
        super().__init__('tts_engagement_node')
        
        # Cliente de acción para TTS del TiAGo
        self._tts_client = ActionClient(self, TTS, '/text_to_speech/tts')
        
        # QoS confiable
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Estado anterior para detectar cambios
        self.previous_status = 0
        self.last_speech_time = 0
        self.speech_cooldown = 10.0
        
        # Suscripción al estado general de engagement
        self.create_subscription(
            Int32,
            '/engagement/general_status',
            self.engagement_status_callback,
            qos
        )
        
        # Frases predefinidas en inglés
        self.engaged_phrases = [
            "Hello! I'm glad to see you here.",
            "How great that you're here with me!",
            "Hello! How are you?",
            "I like it when you pay attention to me!",
            "Excellent! You seem interested."
        ]
        
        self.engaging_phrases = [
            "Hello, are you interested in something?",
            "Come, come closer!",
            "Do you need help with something?"
        ]
        
        # Parámetros con tipos correctos
        self.declare_parameter('speech_cooldown', 10.0)
        self.declare_parameter('volume', 0.5)  # float
        self.declare_parameter('rate', 100)    # int
        self.declare_parameter('gender', 'f')  # string
        self.declare_parameter('tool', 4)      # int
        
        self.speech_cooldown = self.get_parameter('speech_cooldown').get_parameter_value().double_value
        self.volume = self.get_parameter('volume').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.gender = self.get_parameter('gender').get_parameter_value().string_value
        self.tool = self.get_parameter('tool').get_parameter_value().integer_value
        
        self.get_logger().info('TTS Engagement Node started (using TiAGo TTS action)')
        self.get_logger().info(f'Cooldown: {self.speech_cooldown}s')
        self.get_logger().info(f'TTS Config: volume={self.volume}, rate={self.rate}, gender={self.gender}, tool={self.tool}')

    def engagement_status_callback(self, msg):
        """Procesa cambios en el estado general de engagement"""
        current_status = msg.data
        current_time = time.time()
        
        # Solo hablar si ha pasado el tiempo de cooldown
        if current_time - self.last_speech_time > self.speech_cooldown:
            
            # Hablar cuando se vuelve ENGAGED (3)
            if current_status == 3 and self.previous_status != 3:
                phrase = random.choice(self.engaged_phrases)
                self.speak(phrase)
                self.get_logger().info(f'Speaking for full engagement: "{phrase}"')
            
            # Opcionalmente, hablar cuando empieza transición (2)
            elif current_status == 2 and self.previous_status == 0:
                phrase = random.choice(self.engaging_phrases)
                self.speak(phrase)
                self.get_logger().info(f'Speaking for engagement transition: "{phrase}"')
        
        self.previous_status = current_status

    def speak(self, text):
        """Envía texto al TTS del TiAGo usando la acción correcta"""
        if not self._tts_client.server_is_ready():
            self.get_logger().warn('TTS server not available')
            return
        
        # Crear el goal para la acción TTS
        goal_msg = TTS.Goal()
        goal_msg.text = text
        
        # Configurar los parámetros TTS con tipos correctos
        goal_msg.config.volume = float(self.volume)     # float
        goal_msg.config.rate = int(self.rate)           # int
        goal_msg.config.language = 'en'                 # string
        goal_msg.config.gender = self.gender            # string
        goal_msg.config.tool = int(self.tool)           # int
        
        # Enviar la acción de forma asíncrona
        future = self._tts_client.send_goal_async(goal_msg)
        future.add_done_callback(self.tts_goal_response_callback)
        
        self.last_speech_time = time.time()

    def tts_goal_response_callback(self, future):
        """Callback para manejar la respuesta del servidor TTS"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('TTS goal rejected')
            return
        
        self.get_logger().info('TTS goal accepted - TiAGo should be speaking')
        
        # Obtener el resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.tts_result_callback)

    def tts_result_callback(self, future):
        """Callback para manejar el resultado de la acción TTS"""
        result = future.result().result
        self.get_logger().info('TTS completed successfully')

def main(args=None):
    rclpy.init(args=args)
    node = TTSEngagementNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()