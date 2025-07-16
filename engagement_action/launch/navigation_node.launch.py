#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pause_duration',
            default_value='8.0',
            description='Duration to pause when engagement detected (seconds)'
        ),
        
        DeclareLaunchArgument(
            'engagement_threshold',
            default_value='2',
            description='Minimum engagement level to pause (2=transition, 3=engaged)'
        ),
        
        DeclareLaunchArgument(
            'loop_navigation',
            default_value='true',
            description='Whether to loop navigation between waypoints'
        ),
        
        Node(
            package='engagement_action',
            executable='engagement_navigation_node',
            name='engagement_navigation',
            output='screen',
            parameters=[{
                'pause_duration': LaunchConfiguration('pause_duration'),
                'engagement_threshold': LaunchConfiguration('engagement_threshold'),
                'loop_navigation': LaunchConfiguration('loop_navigation'),
            }]
        ),
    ])