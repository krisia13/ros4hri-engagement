#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
from action_msgs.msg import GoalStatus
from rclpy.action import CancelResponse
import time

class EngagementNavigationNode(Node):
    def __init__(self):
        super().__init__('engagement_navigation_node')
        
        # Cliente de acción para navegación
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # QoS confiable
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Suscripción al estado de engagement
        self.create_subscription(
            Int32,
            '/engagement/general_status',
            self.engagement_callback,
            qos
        )
        
        # Variables de estado
        self.current_engagement_status = 0
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_waypoint_index = 0
        self.navigation_paused = False
        self.pause_start_time = None
        self.start_navigation_timer = None
        self.navigation_timer = None
        
        # Definir waypoints con las poses reales
        self.waypoints = self.define_waypoints()
        
        # Parámetros
        self.declare_parameter('pause_duration', 5.0)
        self.declare_parameter('engagement_threshold', 2.0)
        self.declare_parameter('loop_navigation', True)
        self.declare_parameter('wait_between_waypoints', 3.0)
        
        self.pause_duration = self.get_parameter('pause_duration').get_parameter_value().double_value
        self.engagement_threshold = self.get_parameter('engagement_threshold').get_parameter_value().integer_value
        self.loop_navigation = self.get_parameter('loop_navigation').get_parameter_value().bool_value
        self.wait_between_waypoints = self.get_parameter('wait_between_waypoints').get_parameter_value().double_value
        
        # Timer para iniciar navegación - aumentar tiempo de espera
        self.start_navigation_timer = self.create_timer(5.0, self.start_navigation_sequence)
        
        self.get_logger().info('Engagement Navigation Node started')
        self.get_logger().info(f'Engagement threshold: {self.engagement_threshold}, Pause duration: {self.pause_duration}s')

    def define_waypoints(self):
        """Define los puntos de navegación con las poses actualizadas"""
        waypoints = []
        
        # Waypoint 1
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 1.5596764087677002
        pose1.pose.position.y = 1.8583741188049316
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = -0.0017850486899719097
        pose1.pose.orientation.w = 0.9999984067993181
        waypoints.append(pose1)
        
        # Waypoint 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 1.2248239517211914
        pose2.pose.position.y = 6.7751898765563965
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.x = 0.0
        pose2.pose.orientation.y = 0.0
        pose2.pose.orientation.z = 0.002445134492758572
        pose2.pose.orientation.w = 0.9999970106541881
        waypoints.append(pose2)
    
        return waypoints

    def start_navigation_sequence(self):
        """Inicia la secuencia de navegación"""
        if not self.is_navigating and not self.navigation_paused:
            self.navigate_to_next_waypoint()
        
        # Destruir el timer después del primer uso
        if self.start_navigation_timer:
            self.destroy_timer(self.start_navigation_timer)
            self.start_navigation_timer = None

    def navigate_to_next_waypoint(self):
        """Navega al siguiente waypoint"""
        # DIFERENCIA CLAVE: Esperar a que el servidor esté disponible
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available after 5 seconds')
            # Reintentar después de un tiempo
            self.navigation_timer = self.create_timer(5.0, self.navigate_to_next_waypoint_delayed)
            return
        
        # Obtener el siguiente waypoint
        target_pose = self.waypoints[self.current_waypoint_index]
        target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Crear goal de navegación
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(f'Target: x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}')
        
        # Enviar goal
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_callback)
        
        self.is_navigating = True

    def navigation_goal_callback(self, future):
        """Callback cuando se acepta/rechaza el goal de navegación"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by Nav2')
            self.is_navigating = False
            # Reintentar después de un tiempo
            self.navigation_timer = self.create_timer(5.0, self.navigate_to_next_waypoint_delayed)
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Navigation goal accepted, starting navigation')
        
        # Obtener resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Callback cuando termina la navegación"""
        result = future.result()
        status = result.status
        self.is_navigating = False
        self.current_goal_handle = None
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully reached waypoint {self.current_waypoint_index + 1}')
            self.advance_to_next_waypoint()
            
        elif status == 5:  # ABORTED
            self.get_logger().warn(f'Navigation ABORTED for waypoint {self.current_waypoint_index + 1}')
            self.handle_navigation_failure()
            
        elif status == 2:  # CANCELED
            self.get_logger().info('Navigation CANCELED')
            if not self.navigation_paused:
                self.handle_navigation_failure()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.handle_navigation_failure()

    def advance_to_next_waypoint(self):
        """Avanza al siguiente waypoint"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        
        if not self.loop_navigation and self.current_waypoint_index == 0:
            self.get_logger().info('Navigation sequence completed')
            return
        
        # Esperar antes del siguiente waypoint
        self.navigation_timer = self.create_timer(self.wait_between_waypoints, self.navigate_to_next_waypoint_delayed)

    def handle_navigation_failure(self):
        """Maneja fallos de navegación"""
        self.get_logger().warn('Retrying navigation in 10 seconds...')
        
        # Reintentar después de un tiempo más largo
        self.navigation_timer = self.create_timer(10.0, self.navigate_to_next_waypoint_delayed)

    def navigate_to_next_waypoint_delayed(self):
        """Timer callback para navegar al siguiente waypoint"""
        if self.navigation_timer:
            self.destroy_timer(self.navigation_timer)
            self.navigation_timer = None
        
        self.navigate_to_next_waypoint()

    def engagement_callback(self, msg):
        """Callback para manejar cambios en el engagement"""
        self.current_engagement_status = msg.data
        
        # Si detectamos engagement y estamos navegando
        if (msg.data >= self.engagement_threshold and 
            self.is_navigating and 
            not self.navigation_paused):
            
            self.pause_navigation()
            
        # Si ya no hay engagement y estamos pausados
        elif (msg.data < self.engagement_threshold and 
              self.navigation_paused):
            
            self.check_resume_navigation()

    def pause_navigation(self):
        """Pausa la navegación cuando se detecta engagement"""
        if self.current_goal_handle:
            # Cancelar el goal actual
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.navigation_cancel_callback)
            
            self.navigation_paused = True
            self.pause_start_time = time.time()
            
            self.get_logger().info('Navigation paused due to engagement detection')

    def navigation_cancel_callback(self, future):
        """Callback cuando se cancela la navegación"""
        cancel_result = future.result()
        if cancel_result.return_code == 1:  # ACCEPTED
            self.get_logger().info('Navigation successfully cancelled')
        else:
            self.get_logger().warn('Navigation cancellation failed or was rejected')
        
        self.is_navigating = False
        self.current_goal_handle = None

    def check_resume_navigation(self):
        """Verifica si se puede reanudar la navegación"""
        # Esperar un tiempo mínimo antes de reanudar
        if (self.pause_start_time and 
            time.time() - self.pause_start_time >= self.pause_duration):
            
            self.resume_navigation()

    def resume_navigation(self):
        """Reanuda la navegación cuando ya no hay engagement"""
        self.navigation_paused = False
        self.pause_start_time = None
        
        self.get_logger().info('Resuming navigation')
        
        # Reanudar navegación al mismo waypoint
        self.navigation_timer = self.create_timer(1.0, self.navigate_to_next_waypoint_delayed)

def main(args=None):
    rclpy.init(args=args)
    node = EngagementNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()