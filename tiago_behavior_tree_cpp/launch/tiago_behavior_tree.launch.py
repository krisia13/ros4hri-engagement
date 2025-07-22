from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        
        # SECUENCIA CORE (tu método manual)
        
        # 0s: Detección Facial
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('hri_face_detect'),
                '/launch/face_detect.launch.py'
            ]),
            launch_arguments={
                'filtering_frame': 'base_footprint',
                'rgb_camera': '/head_front_camera/rgb/image_raw',
                'camera_frame_id': 'head_front_camera_color_optical_frame'
            }.items()
        ),
        
        # 3s: Detección Corporal
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('hri_fullbody'),
                    '/launch/hri_fullbody.launch.py'
                ]),
                launch_arguments={
                    'rgb_camera': '/head_front_camera/rgb/image_raw',
                    'camera_frame_id': 'head_front_camera_color_optical_frame'
                }.items()
            )
        ]),
        
        # 6s: Transformadas
        TimerAction(period=6.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                     '--frame-id', 'head_front_camera_color_optical_frame',
                     '--child-frame-id', 'default_cam'],
                output='screen'
            )
        ]),
        
        # 9s: Gestor de Personas
        TimerAction(period=9.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('hri_person_manager'),
                    '/launch/person_manager.launch.py'
                ]),
                launch_arguments={
                    'robot_reference_frame': 'base_footprint',
                    'reference_frame': 'base_footprint'
                }.items()
            )
        ]),
        
        # 12s: Engagement
        TimerAction(period=12.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('hri_engagement'),
                    '/launch/hri_engagement.launch.py'
                ])
            )
        ]),
        
        # 15s: LEDs
        TimerAction(period=15.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('engagement_action'),
                    '/launch/robot_led_control.launch.py'
                ])
            )
        ]),
        
        # EXTENSIONES (tus nodos adicionales)
        
        # # 18s: Text to Speech
        # TimerAction(period=18.0, actions=[
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             get_package_share_directory('text_to_speech'),
        #             '/launch/text_to_speech.launch.py'
        #         ])
        #     )
        # ]),
        
        # 21s: TTS Node
        TimerAction(period=21.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('engagement_action'),
                    '/launch/tts_node.launch.py'
                ])
            )
        ]),
        
        # # 24s: Navigation
        # TimerAction(period=24.0, actions=[
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             get_package_share_directory('engagement_action'),
        #             '/launch/navigation_node.launch.py'
        #         ])
        #     )
        # ]),
        
        # 27s: Voice Interaction
        TimerAction(period=27.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('engagement_action'),
                    '/launch/voice_interaction.launch.py'
                ])
            )
        ]),
        
        # 30s: BT Coordinador (cuando TODO esté funcionando)
        TimerAction(period=30.0, actions=[
            Node(
                package='tiago_behavior_tree_cpp',
                executable='tiago_behavior_tree_cpp_node',
                name='tiago_bt_coordinator',
                output='screen',
                parameters=[{
                    'tick_rate': 5.0,
                    'debug_mode': True,
                    'tree_file': 'tiago_main_tree.xml'
                }],
                respawn=True,
                respawn_delay=3.0
            )
        ]),
        
    ])