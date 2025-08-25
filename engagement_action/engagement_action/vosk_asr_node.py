"""Nodo ROS2 para reconocimiento automático de voz usando Vosk.

Este nodo recibe datos de audio desde el topic '/audio/channel0', procesa el audio usando
el modelo Vosk y publica el texto reconocido en el topic 'voz_texto'.

Attributes:
    VoskASRNode (Node): Nodo principal para reconocimiento de voz.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import numpy as np
from audio_common_msgs.msg import AudioData


class VoskASRNode(Node):
    """Nodo para reconocimiento de voz usando Vosk."""

    def __init__(self):
        """Inicializa el nodo VoskASRNode.

        Carga el modelo Vosk, configura el reconocedor y las suscripciones/publicaciones.
        """
        super().__init__('vosk_asr_node')
        self.model = Model('/home/cristina/Downloads/vosk-model-es-0.42')
        self.recognizer = KaldiRecognizer(self.model, 16000)

        # Suscripción al audio del micrófono
        self.subscription = self.create_subscription(
            AudioData,
            '/audio/channel0',
            self.audio_callback,
            10
        )
        # Publicador del texto reconocido
        self.publisher = self.create_publisher(String, 'voz_texto', 10)


    def audio_callback(self, msg):
        """Procesa los datos de audio recibidos y publica el texto reconocido.

        Args:
            msg (AudioData): Mensaje con datos de audio en formato int16.
        """
        # Convierte la lista de int16 a bytes
        audio_np = np.array(msg.int16_data, dtype=np.int16)
        audio_bytes = audio_np.tobytes()
        if self.recognizer.AcceptWaveform(audio_bytes):
            result = self.recognizer.Result()
            self.get_logger().info(f"🗣️ Texto reconocido: {result}")
            self.publisher.publish(String(data=result))
        else:
            partial = self.recognizer.PartialResult()
            self.get_logger().info(f"⏳ Parcial: {partial}")

def main(args=None):
    """Función principal para iniciar el nodo VoskASRNode.

    Args:
        args (list, optional): Argumentos de inicialización ROS2.
    """
    rclpy.init(args=args)
    node = VoskASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()