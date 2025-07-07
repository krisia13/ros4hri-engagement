#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pal_device_msgs.action import DoTimedLedEffect
from hri_msgs.msg import EngagementLevel, IdsList
from std_msgs.msg import String
from builtin_interfaces.msg import Duration

class EngagementLEDNode(Node):
    def __init__(self):
        super().__init__('engagement_led_control')
        
        # Cliente de acción para los LEDs
        self._action_client = ActionClient(self, DoTimedLedEffect, '/led_manager_node/do_effect')
        
        # QoS confiable
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Suscripción a la lista de personas detectadas
        self.create_subscription(
            IdsList,
            '/humans/persons/tracked',
            self.tracked_persons_callback,
            qos
        )
        
        # Diccionario para mantener las suscripciones activas
        self.engagement_subscriptions = {}
        self.face_id_subscriptions = {}
        self.persons_with_face = set()
        
        # Registro de niveles de engagement por persona
        self.engagement_levels = {}
        
        # Constantes para niveles de engagement
        self.DISENGAGED = 1
        self.ENGAGING = 2
        self.ENGAGED = 3
        self.DISENGAGING = 4
        
        # Colores predefinidos
        self.green_color = {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}
        self.yellow_color = {"r": 1.0, "g": 1.0, "b": 0.0, "a": 1.0}
        self.white_color = {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}
        
        # Color actual
        self.current_color = self.white_color
        
        # Iniciar con LEDs blancos
        self.set_led_color(self.white_color)
        
        self.get_logger().info('Nodo de control de LEDs por engagement iniciado')

    def tracked_persons_callback(self, msg):
        """Procesa la lista de personas detectadas"""
        # Verificar nuevas personas y crear suscripciones
        for person_id in msg.ids:
            if person_id not in self.face_id_subscriptions:
                self.get_logger().info(f'Nueva persona detectada: {person_id}')
                
                # Primero verificamos si tiene face_id
                face_topic = f'/humans/persons/{person_id}/face_id'
                self.face_id_subscriptions[person_id] = self.create_subscription(
                    String,
                    face_topic,
                    lambda msg, id=person_id: self.face_id_callback(msg, id),
                    QoSProfile(depth=1)
                )
        
        # Eliminar suscripciones para personas que ya no están
        for person_id in list(self.face_id_subscriptions.keys()):
            if person_id not in msg.ids:
                if person_id in self.engagement_subscriptions:
                    self.engagement_subscriptions[person_id].destroy()
                    del self.engagement_subscriptions[person_id]
                
                self.face_id_subscriptions[person_id].destroy()
                del self.face_id_subscriptions[person_id]
                
                if person_id in self.persons_with_face:
                    self.persons_with_face.remove(person_id)
                
                # Si la persona tenía un nivel de engagement, eliminarlo
                if person_id in self.engagement_levels:
                    del self.engagement_levels[person_id]
                    # Actualizar LEDs basado en el estado actual
                    self.update_led_status()
                    
                self.get_logger().info(f'Persona ya no detectada: {person_id}')

    def face_id_callback(self, msg, person_id):
        """Verifica si la persona tiene un face_id válido"""
        if msg.data and person_id not in self.persons_with_face:
            self.get_logger().info(f'Persona con cara detectada: {person_id}, face_id: {msg.data}')
            self.persons_with_face.add(person_id)
            
            # Ahora nos suscribimos al engagement de esta persona
            engagement_topic = f'/humans/persons/{person_id}/engagement_status'
            self.engagement_subscriptions[person_id] = self.create_subscription(
                EngagementLevel,
                engagement_topic,
                lambda msg, id=person_id: self.engagement_callback(msg, id),
                QoSProfile(depth=1)
            )

    def engagement_callback(self, msg, person_id):
        """Procesa mensajes de engagement"""
        # Guardar nivel anterior para detectar cambios
        previous_level = self.engagement_levels.get(person_id, 0)
        
        # Actualizar nivel actual
        self.engagement_levels[person_id] = msg.level
        
        # Solo actuar si hay cambio en el nivel
        if previous_level != msg.level:
            self.get_logger().info(f'Persona {person_id}: cambio de nivel {previous_level} a {msg.level}')
            self.update_led_status()

    def update_led_status(self):
        """Actualiza el color de los LEDs según los niveles de engagement"""
        # Determinar el estado general de engagement basado en prioridad
        has_engaged = False
        has_transition = False
        
        for person_id, level in self.engagement_levels.items():
            if level == self.ENGAGED:
                has_engaged = True
                break
            elif level == self.ENGAGING or level == self.DISENGAGING:
                has_transition = True
        
        # Aplicar color según prioridad
        if has_engaged:
            # Al menos una persona engaged: verde
            self.get_logger().info('Al menos una persona ENGAGED. LEDs en verde.')
            new_color = self.green_color
        elif has_transition:
            # Al menos una persona en transición (engaging/disengaging): amarillo
            self.get_logger().info('Al menos una persona en transición. LEDs en amarillo.')
            new_color = self.yellow_color
        else:
            # Todas las personas desengaged o no hay personas: blanco
            self.get_logger().info('Nadie engaged o en transición. LEDs en blanco.')
            new_color = self.white_color
        
        # Cambiar color solo si es diferente al actual
        if new_color != self.current_color:
            self.set_led_color(new_color)
            self.current_color = new_color

    def set_led_color(self, color_dict, duration_sec=0):
        """Envía un comando para cambiar el color de los LEDs"""
        goal_msg = DoTimedLedEffect.Goal()
        goal_msg.devices = []
        goal_msg.params.effect_type = 0  # Fixed color
        
        # Asignar valores de color desde el diccionario
        goal_msg.params.fixed_color.color.r = color_dict["r"]
        goal_msg.params.fixed_color.color.g = color_dict["g"]
        goal_msg.params.fixed_color.color.b = color_dict["b"]
        goal_msg.params.fixed_color.color.a = color_dict["a"]
        
        # Duration 0 significa mantener el efecto hasta nuevo cambio
        goal_msg.effect_duration = Duration(sec=duration_sec, nanosec=0)
        goal_msg.priority = 10
        
        # Esperar por el servidor
        self._action_client.wait_for_server()
        
        # Enviar la acción
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EngagementLEDNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()