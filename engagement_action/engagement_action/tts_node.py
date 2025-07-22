#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from text_to_speech_msgs.action import TTS
import random
import time

class TTSEngagementNode(Node):
    def __init__(self):
        super().__init__('tts_engagement_node')
        
        # Cliente de acción para TTS del TiAGo
        self._tts_client = ActionClient(self, TTS, '/text_to_speech/tts')
        
        # Control de cooldown para evitar spam de voz
        self.last_speech_time = 0
        self.speech_cooldown = 3.0  # 3 segundos entre comandos

        # =================================================================
        # SUSCRIPCIÓN ÚNICA: Solo comandos del BT
        # =================================================================
        self.create_subscription(
            String,
            '/speech_text',  # Topic que publica el BT
            self.bt_speech_command_callback,
            10
        )
        
        # =================================================================
        # FRASES ESPECÍFICAS PARA COMANDOS BT
        # =================================================================
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

        # =================================================================
        # PARÁMETROS TTS
        # =================================================================
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
        """Procesa SOLO comandos de habla del BT"""
        command = msg.data.strip()
        current_time = time.time()
        
        # Control de cooldown
        if current_time - self.last_speech_time < self.speech_cooldown:
            self.get_logger().warn(f'🚫 Cooldown activo - ignorando comando: "{command}"')
            return
        
        # Procesar comando
        if command in self.bt_phrases:
            # Comando predefinido - seleccionar frase aleatoria
            phrase = random.choice(self.bt_phrases[command])
            self.speak(phrase)
            self.get_logger().info(f'🤖 BT command "{command}": "{phrase}"')
            
        elif command.startswith('SAY:'):
            # Comando directo - hablar el texto después de "SAY:"
            text = command[4:].strip()
            if text:
                self.speak(text)
                self.get_logger().info(f'🗣️ BT direct speech: "{text}"')
            else:
                self.get_logger().warn('⚠️ Comando SAY: vacío')
                
        else:
            # Hablar directamente el texto completo
            self.speak(command)
            self.get_logger().info(f'🔤 BT literal speech: "{command}"')

    def speak(self, text):
        """Envía texto al TTS del TiAGo"""
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
        
        # Enviar la acción
        future = self._tts_client.send_goal_async(goal_msg)
        future.add_done_callback(self.tts_goal_response_callback)
        
        # Actualizar tiempo del último speech
        self.last_speech_time = time.time()
        
        self.get_logger().debug(f'🚀 Enviando TTS: "{text}"')

    def tts_goal_response_callback(self, future):
        """Callback para respuesta del servidor TTS"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('❌ TTS goal rechazado')
                return
            
            self.get_logger().debug('✅ TTS goal aceptado - TiAGo hablando...')
            
            # Obtener resultado
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.tts_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en TTS goal response: {e}')

    def tts_result_callback(self, future):
        """Callback para resultado de la acción TTS"""
        try:
            result = future.result().result
            self.get_logger().debug('✅ TTS completado exitosamente')
        except Exception as e:
            self.get_logger().error(f'❌ Error en TTS result: {e}')

def main(args=None):
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