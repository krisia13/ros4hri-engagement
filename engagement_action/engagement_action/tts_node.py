#!/usr/bin/env python3

"""Nodo ROS2 para síntesis de voz (TTS) en función de comandos recibidos.

Este nodo recibe comandos de texto desde el topic '/speech_text' y utiliza el sistema
de texto a voz (TTS) del robot TiAGo para hablar frases predefinidas o texto arbitrario.
Incluye control de cooldown para evitar repeticiones rápidas.

Attributes:
    TTSEngagementNode (Node): Nodo principal que gestiona la lógica de TTS.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from text_to_speech_msgs.action import TTS
import random
import time

class TTSEngagementNode(Node):
    """Nodo que gestiona la síntesis de voz del robot TiAGo según comandos recibidos."""

    def __init__(self):
        """Inicializa el nodo TTSEngagementNode.

        Configura el cliente de acción TTS, parámetros, frases y suscripciones.
        """
        super().__init__('tts_engagement_node')
        
        # Cliente de acción para TTS del TiAGo
        self._tts_client = ActionClient(self, TTS, '/text_to_speech/tts')
        
        # Control de cooldown para evitar spam de voz
        self.last_speech_time = 0
        self.speech_cooldown = 3.0  # 3 segundos entre comandos

        # Suscripción al topic de comandos de voz del BT
        self.create_subscription(
            String,
            '/speech_text', 
            self.bt_speech_command_callback,
            10
        )
        
        # Diccionario de frases predefinidas para comandos específicos
        self.bt_phrases = {
            'ENGAGEMENT_GREETING': [
                "Hello! I see you looking at me. How can I help you today?",
                "Hi there! You have my attention. What would you like to know?",
                "Great! I notice you're interested. How can I assist you?",
                "Hello! I'm glad to see you here. What can I do for you?",
                "Excellent! You seem interested. How may I help?"
            ],
            'ENGAGEMENT_GOODBYE': [
                "Goodbye! It was nice talking to you. Have a great day!",
                "Thank you for the interaction! See you later!",
                "It was a pleasure meeting you. Take care!",
                "Farewell! I enjoyed our conversation. Until next time!",
                "Goodbye! Hope to see you again soon!"
            ],
            'VOICE_RESPONSE': [
                "I heard you speaking! What can I do for you?",
                "Yes, I'm listening. How can I help?",
                "I heard your voice. What would you like to discuss?",
                "I'm here! What did you want to tell me?",
                "You have my attention! How can I assist you?"
            ],
            'VOICE_RESPONSE_FRONT': [
                "I heard a voice coming from the front."
            ],
            'VOICE_RESPONSE_RIGHT': [
                "I heard a voice coming from the right."
            ],
            'VOICE_RESPONSE_LEFT': [
                "I heard a voice coming from the left."
            ],
            'VOICE_RESPONSE_BACK': [
                "I heard a voice coming from behind me."
            ]
        }

        # Parámetros configurables para TTS
        self.declare_parameter('speech_cooldown', 3.0)
        self.declare_parameter('volume', 0.7)
        self.declare_parameter('rate', 120)
        self.declare_parameter('gender', 'f')
        self.declare_parameter('tool', 4)
        
        self.speech_cooldown = self.get_parameter('speech_cooldown').get_parameter_value().double_value
        self.volume = self.get_parameter('volume').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.gender = self.get_parameter('gender').get_parameter_value().string_value
        self.tool = self.get_parameter('tool').get_parameter_value().integer_value
        
        self.get_logger().info('🗣️ TTS Node iniciado - SOLO para comandos BT')
        self.get_logger().info(f'⏱️ Cooldown: {self.speech_cooldown}s')
        self.get_logger().info(f'🔧 TTS Config: volume={self.volume}, rate={self.rate}, gender={self.gender}')
        self.get_logger().info(f'📝 Comandos disponibles: {list(self.bt_phrases.keys())}')

    def bt_speech_command_callback(self, msg):
        """Procesa comandos de voz recibidos desde el topic '/speech_text'.

        Args:
            msg (String): Mensaje recibido con el comando de voz.
        """
        command = msg.data.strip()
        current_time = time.time()
        
        # Control de cooldown para evitar repeticiones rápidas
        if current_time - self.last_speech_time < self.speech_cooldown:
            self.get_logger().warn(f'🚫 Cooldown activo - ignorando comando: "{command}"')
            return
        
        # Procesar comando
        if command in self.bt_phrases:
            # Comando predefinido - selecciona frase aleatoria
            phrase = random.choice(self.bt_phrases[command])
            self.speak(phrase)
            self.get_logger().info(f'🤖 BT command "{command}": "{phrase}"')
            
        elif command.startswith('SAY:'):
            # Comando directo - habla el texto después de "SAY:"
            text = command[4:].strip()
            if text:
                self.speak(text)
                self.get_logger().info(f'🗣️ BT direct speech: "{text}"')
            else:
                self.get_logger().warn('⚠️ Comando SAY: vacío')
                
        else:
            # Habla el texto literal recibido
            self.speak(command)
            self.get_logger().info(f'🔤 BT literal speech: "{command}"')

    def speak(self, text):
        """Envía texto al sistema TTS del TiAGo para que lo pronuncie.

        Args:
            text (str): Texto a pronunciar.
        """
        if not text or not text.strip():
            self.get_logger().warn('⚠️ Texto vacío - no se puede hablar')
            return
            
        if not self._tts_client.server_is_ready():
            self.get_logger().warn('❌ TTS server no disponible')
            return
        
        # Crear el goal para la acción TTS
        goal_msg = TTS.Goal()
        goal_msg.text = text.strip()
        
        # Configurar parámetros TTS
        goal_msg.config.volume = float(self.volume)
        goal_msg.config.rate = int(self.rate)
        goal_msg.config.language = 'en'
        goal_msg.config.gender = self.gender
        goal_msg.config.tool = int(self.tool)
        
        # Enviar la acción al servidor TTS
        future = self._tts_client.send_goal_async(goal_msg)
        future.add_done_callback(self.tts_goal_response_callback)
        
        # Actualizar tiempo del último speech
        self.last_speech_time = time.time()
        
        self.get_logger().debug(f'🚀 Enviando TTS: "{text}"')

    def tts_goal_response_callback(self, future):
        """Callback para la respuesta del servidor TTS tras enviar el goal.

        Args:
            future: Futuro con el resultado de la petición de goal.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('❌ TTS goal rechazado')
                return
            
            self.get_logger().debug('✅ TTS goal aceptado - TiAGo hablando...')
            
            # Obtener resultado de la acción TTS
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.tts_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en TTS goal response: {e}')

    def tts_result_callback(self, future):
        """Callback para el resultado final de la acción TTS.

        Args:
            future: Futuro con el resultado de la acción TTS.
        """
        try:
            result = future.result().result
            self.get_logger().debug('✅ TTS completado exitosamente')
        except Exception as e:
            self.get_logger().error(f'❌ Error en TTS result: {e}')

def main(args=None):
    """Función principal para iniciar el nodo TTSEngagementNode.

    Args:
        args (list, optional): Argumentos de inicialización ROS2.
    """
    rclpy.init(args=args)
    node = TTSEngagementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('🛑 TTS Node finalizando...')
        rclpy.shutdown()

if __name__ == '__main__':
    main()