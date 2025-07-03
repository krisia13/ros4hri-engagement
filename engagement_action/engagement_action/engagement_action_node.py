#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hri_actions_msgs.msg import Intent
from std_msgs.msg import String
import json

class EngagementActionNode(Node):
    def __init__(self):
        super().__init__('engagement_action')
        
        # Subscribe to intent messages to detect ENGAGE_WITH
        self.intent_sub = self.create_subscription(
            Intent, 
            '/intents',
            self.intent_callback,
            10)
            
        # Publisher for engagement notifications
        self.engagement_pub = self.create_publisher(
            String, 
            '/engagement_detected', 
            10)
            
        # Publisher for robot speech
        self.speech_pub = self.create_publisher(
            String, 
            '/robot_speech', 
            10)
            
        # Keep track of people we've already greeted
        self.greeted_people = set()
        
        self.get_logger().info('Engagement Action Node is ready!')
        
    def intent_callback(self, msg):
        self.get_logger().info(f"Received intent: {msg.intent}")
        
        # The intent format appears to be "__intent_engage_with__"
        if "engage_with" in msg.intent:
            try:
                # Parse the person ID from JSON data
                data = json.loads(msg.data)
                person_id = data.get('recipient', 'unknown')
                
                # Publish engagement detection
                engage_msg = String()
                engage_msg.data = f"Person {person_id} has engaged with the robot"
                self.engagement_pub.publish(engage_msg)
                self.get_logger().info(f"Published engagement for {person_id}")
                
                # Only greet each person once (until they leave and return)
                if person_id not in self.greeted_people:
                    # Generate speech
                    speech_msg = String()
                    speech_msg.data = f"Hello there! I see you're interested in talking with me. How can I help you today?"
                    self.speech_pub.publish(speech_msg)
                    
                    # Remember we've greeted this person
                    self.greeted_people.add(person_id)
                    
                    # Create a timer to forget this person after 60 seconds
                    # so we can greet them again if they re-engage later
                    self.create_timer(60.0, lambda p=person_id: self.reset_greeting(p))
                
            except Exception as e:
                self.get_logger().error(f"Error processing intent: {e}")
    
    def reset_greeting(self, person_id):
        """Remove person from greeted list after timeout"""
        if person_id in self.greeted_people:
            self.greeted_people.remove(person_id)
            self.get_logger().info(f"Reset greeting status for {person_id}")

def main(args=None):
    rclpy.init(args=args)
    node = EngagementActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()