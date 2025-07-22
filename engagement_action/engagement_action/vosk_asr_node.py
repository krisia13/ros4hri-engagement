import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import numpy as np
from audio_common_msgs.msg import AudioData


class VoskASRNode(Node):
    def __init__(self):
        super().__init__('vosk_asr_node')
        self.model = Model('/home/cristina/Downloads/vosk-model-es-0.42')
        self.recognizer = KaldiRecognizer(self.model, 16000)

        self.subscription = self.create_subscription(
            AudioData,
            '/audio/channel0',
            self.audio_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'voz_texto', 10)


    def audio_callback(self, msg):
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
    rclpy.init(args=args)
    node = VoskASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()