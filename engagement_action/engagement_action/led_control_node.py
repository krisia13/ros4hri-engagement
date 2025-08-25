#!/usr/bin/env python3

"""Nodo ROS2 para controlar el color de los LEDs en función del nivel de engagement detectado.

Este nodo suscribe a los tópicos de personas detectadas y sus niveles de engagement,
y publica el estado general de engagement. Cambia el color de los LEDs usando una acción
según el estado general de engagement.

Attributes:
    EngagementLEDNode (Node): Nodo principal que gestiona la lógica de engagement y LEDs.
"""


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pal_device_msgs.action import DoTimedLedEffect
from hri_msgs.msg import EngagementLevel, IdsList
from std_msgs.msg import String, Int32
from builtin_interfaces.msg import Duration

class EngagementLEDNode(Node):
    """Nodo que gestiona el color de los LEDs según el engagement detectado."""

    def __init__(self):
        """Inicializa el nodo EngagementLEDNode.

        Crea los clientes, publishers y suscripciones necesarias para el funcionamiento.
        """
        super().__init__('led_control')
        
        # Cliente de acción para los LEDs
        self._action_client = ActionClient(self, DoTimedLedEffect, '/led_manager_node/do_effect')
        
        # QoS confiable para comunicaciones importantes
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publisher para el estado general de engagement
        self.engagement_status_pub = self.create_publisher(
            Int32,
            '/engagement/general_status',
            qos
        )
        
        # Diccionarios para gestionar suscripciones y estados
        self.face_id_subscriptions = {}  # Suscripciones a face_id por persona
        self.engagement_subscriptions = {}  # Suscripciones a engagement por persona
        self.persons_with_face = set()  # Personas con face_id válido
        self.engagement_levels = {}  # Niveles de engagement por persona
        self.current_general_status = 0  # Estado general actual (0: DISENGAGED, 2: TRANSITION, 3: ENGAGED)

        # Suscripción a la lista de personas detectadas
        self.create_subscription(
            IdsList,
            '/humans/persons/tracked',
            self.tracked_persons_callback,
            qos
        )
        
        # Colores predefinidos para los LEDs
        self.green_color = {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}
        self.yellow_color = {"r": 1.0, "g": 1.0, "b": 0.0, "a": 1.0}
        self.white_color = {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}
        
        # Temporizador para actualizar el color de los LEDs cada segundo
        self.create_timer(1.0, self.update_led_status)
        
        # Inicializa los LEDs en color blanco
        self.set_led_color(self.white_color)
        
        self.get_logger().info('Nodo de control de LEDs por engagement iniciado')

    def tracked_persons_callback(self, msg):
        """Callback para procesar la lista de personas detectadas.

        Args:
            msg (IdsList): Mensaje con los IDs de las personas detectadas.
        """
        # Verifica nuevas personas y crea suscripciones a sus face_id
        for person_id in msg.ids:
            if person_id not in self.face_id_subscriptions:
                # Primero verificamos si tiene face_id
                face_topic = f'/humans/persons/{person_id}/face_id'
                self.face_id_subscriptions[person_id] = self.create_subscription(
                    String,
                    face_topic,
                    lambda msg, id=person_id: self.face_id_callback(msg, id),
                    QoSProfile(depth=1)
                )
                self.get_logger().info(f'Verificando si {person_id} tiene face_id...')

    def face_id_callback(self, msg, person_id):
        """Callback para verificar si la persona tiene un face_id válido.

        Args:
            msg (String): Mensaje con el face_id.
            person_id (int): ID de la persona.
        """
        if msg.data and person_id not in self.persons_with_face:
            self.get_logger().info(f'Persona con face_id válido: {person_id}, face_id: {msg.data}')
            self.persons_with_face.add(person_id)
            
            # Suscribe al engagement_status de la persona
            engagement_topic = f'/humans/persons/{person_id}/engagement_status'
            self.engagement_subscriptions[person_id] = self.create_subscription(
                EngagementLevel,
                engagement_topic,
                lambda msg, id=person_id: self.engagement_callback(msg, id),
                QoSProfile(depth=1)
            )
            self.get_logger().info(f'Monitoreando engagement de {person_id}')

    def engagement_callback(self, msg, person_id):
        """Callback para procesar el nivel de engagement de una persona.

        Args:
            msg (EngagementLevel): Mensaje con el nivel de engagement.
            person_id (int): ID de la persona.
        """
        self.engagement_levels[person_id] = msg.level
        self.get_logger().info(f'Persona {person_id}: nivel de engagement = {msg.level}')

    def update_led_status(self):
        """Actualiza el color de los LEDs según los niveles de engagement detectados.

        Determina el estado general de engagement y publica el cambio si es necesario.
        """        
        has_engaged = False
        has_transition = False
        
        # Solo considera personas con face_id válido
        for person_id, level in self.engagement_levels.items():
            if person_id in self.persons_with_face:
                if level == 3:  # ENGAGED
                    has_engaged = True
                    break
                elif level in [2, 4]:  # ENGAGING o DISENGAGING
                    has_transition = True
        
        # Determina nuevo estado y color
        new_status = 0  # DISENGAGED por defecto
        if has_engaged:
            new_status = 3  # ENGAGED
            new_color = self.green_color
            status = "ENGAGED (verde)"
        elif has_transition:
            new_status = 2  # TRANSITION
            new_color = self.yellow_color
            status = "EN TRANSICIÓN (amarillo)"
        else:
            new_status = 0  # DISENGAGED
            new_color = self.white_color
            status = "DESENGAGED (blanco)"

        # Publicar cambio de estado si es diferente al actual
        if new_status != self.current_general_status:
            self.current_general_status = new_status
            status_msg = Int32()
            status_msg.data = new_status
            self.engagement_status_pub.publish(status_msg)
            self.get_logger().info(f'Cambio de estado general: {new_status}')
        
        # Cambia el color de los LEDs
        self.get_logger().info(f'Estado actual: {status}')
        self.set_led_color(new_color)

    def set_led_color(self, color_dict):
        """Envía un comando para cambiar el color de los LEDs.

        Args:
            color_dict (dict): Diccionario con los valores RGBA del color.
        """
        goal_msg = DoTimedLedEffect.Goal()
        goal_msg.devices = []
        goal_msg.params.effect_type = 0  
        
        # Asigna los valores de color
        goal_msg.params.fixed_color.color.r = color_dict["r"]
        goal_msg.params.fixed_color.color.g = color_dict["g"]
        goal_msg.params.fixed_color.color.b = color_dict["b"]
        goal_msg.params.fixed_color.color.a = color_dict["a"]
        
        # Configura duración y prioridad
        goal_msg.effect_duration = Duration(sec=300, nanosec=0)
        goal_msg.priority = 100
        
        # Espera por el servidor de acción
        self._action_client.wait_for_server()
        
        # Envía la acción para cambiar el color
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    """Función principal para iniciar el nodo EngagementLEDNode.

    Args:
        args (list, optional): Argumentos de inicialización ROS2.
    """
    rclpy.init(args=args)
    node = EngagementLEDNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()