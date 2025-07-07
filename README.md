# ROS4HRI Engagement Detection System

This repository contains a complete system for human detection, tracking, and engagement analysis using an Asus Xtion RGB-D camera with ROS 2.

## Overview

This project implements a human engagement detection system based on the ROS4HRI framework. It can detect when a person is looking at and engaging with a robot, and trigger appropriate responses (speech, notifications, etc.).

## Components

- **Asus Xtion Driver**: Camera interface for RGB and depth data
- **Face Detection**: Detection and tracking of human faces
- **Body Detection**: Full-body pose estimation and tracking
- **Person Manager**: Associates face and body detections to track complete persons
- **Engagement Detection**: Analyzes human attention and engagement based on gaze and proximity
- **Engagement Action**: Custom node that responds when a person engages with the robot

## Dependencies

This project requires the following ROS packages:

```bash
# Core ROS4HRI packages
git clone https://github.com/ros4hri/hri_msgs.git
git clone https://github.com/ros4hri/hri_actions_msgs.git
git clone https://github.com/ros4hri/hri_engagement.git
git clone https://github.com/ros4hri/hri_person_manager.git
git clone https://github.com/ros4hri/hri_face_detect.git
git clone https://github.com/ros4hri/hri_fullbody.git
git clone https://github.com/mgonzs13/ros2_asus_xtion.git

```

## Installation

```bash
# Create a workspace
mkdir -p ~/ros2_ws/ros4hri_ws/src
cd ~/ros2_ws/ros4hri_ws/src

# Clone this repository
git clone https://github.com/krisia13/ros4hri-engagement.git

# Clone dependencies (see list above)

# Build
cd ~/ros2_ws/ros4hri_ws
colcon build
source install/setup.bash
```

## Configuration

Create a configuration file:
```bash
mkdir -p ~/.pal/config
nano ~/.pal/config/ros4hri-tutorials.yml
```

Add these settings:
```yaml
/hri_face_detect:
   remappings:
      image: /camera/rgb/image_raw
      camera_info: /camera/rgb/camera_info
   ros__parameters:
      image_compressed: true

/hri_fullbody:
   remappings:
      image: /camera/rgb/image_raw
      camera_info: /camera/rgb/camera_info
   ros__parameters:
      image_compressed: true

/hri_engagement:
  ros__parameters:
    # Default engagement parameters
    engagement_timeout: 2.0
    gaze_engagement_threshold: 0.5
    proximity_threshold: 2

/hri_person_manager:
  ros__parameters:
    # Enable built-in association without the matcher
    use_auto_association: true
    # Be more lenient with associations
    association_threshold: 0.4
    # Increase timeouts to maintain tracking
    face_timeout: 2.0
    body_timeout: 2.0
```

## Running the System

Launch the components in the following order:

### 1. Start the Camera Driver
```bash
ros2 launch asus_xtion asus_xtion.launch.py
```

### 2. Launch Face Detection
```bash
ros2 launch hri_face_detect face_detect.launch.py filtering_frame:=camera_rgb_optical_frame rgb_camera:=/camera/rgb
```

### 3. Launch Body Detection
```bash
ros2 launch hri_fullbody hri_fullbody.launch.py rgb_camera:=/camera/rgb
```

### 4. Set up Required Transforms
```bash
# Both of these transforms are required for the system to work correctly
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id default_cam
```

### 5. Start Person Manager and Engagement Detection
```bash
ros2 launch hri_person_manager person_manager.launch.py
ros2 launch hri_engagement hri_engagement.launch.py
```

### 6. Launch the Engagement Action Node
```bash
ros2 launch engagement_action engagement_action.launch.py
```

## Visualization in RViz

```bash
rviz2
```

In RViz:
1. Set the Fixed Frame to `map`
2. Add the following displays:
   - Humans plugin
   - Skeletons3D plugin
   - TF_HRI plugin
   - TF (to see coordinate frames)

## Monitoring Engagement

To monitor engagement levels for a detected person:

1. List all active topics to find the current person ID:
```bash
ros2 topic list | grep persons
```

2. Check the engagement status for a specific person:
```bash
ros2 topic echo /humans/persons/anonymous_person_XXXXX/engagement_status
```
Replace `anonymous_person_XXXXX` with the actual ID shown in the topic list.

3. Watch for intent messages:
```bash
ros2 topic echo /intents
```



# Configuración para Tiago Robot

Esta sección contiene la configuración específica para ejecutar el sistema de detección de engagement en el robot Tiago.

## Archivo de Configuración

Crea un archivo de configuración en `~/.pal/config/ros4hri-tutorials.yml` con el siguiente contenido:

```yaml
/hri_face_detect:
   remappings:
      image: /head_front_camera/rgb/image_raw
      camera_info: /head_front_camera/rgb/camera_info
   ros__parameters:
      image_compressed: false
      camera_frame_id: head_front_camera_color_optical_frame
      filtering_frame: base_footprint

/hri_fullbody:
   remappings:
      image: /head_front_camera/rgb/image_raw
      camera_info: /head_front_camera/rgb/camera_info
   ros__parameters:
      image_compressed: false
      camera_frame_id: head_front_camera_color_optical_frame

/hri_engagement:
  ros__parameters:
    reference_frame: "base_footprint"  # Punto de referencia del robot, alineado con la dirección de la mirada
    max_distance: 4.0  # Aumentado a 4 metros para mayor alcance
    field_of_view: 80.0  # Aumentado a 80 grados para un campo de atención más amplio
    engagement_threshold: 0.4  # Reducido para hacer más fácil considerar a alguien como "engaged"
    observation_window: 5.0  # Reducido para respuesta más rápida a cambios
    rate: 10.0  # Mantenido como en la documentación

/hri_person_manager:
  ros__parameters:
    reference_frame: map
    robot_reference_frame: base_footprint
    use_auto_association: true
    association_threshold: 0.4
    face_timeout: 2.0
    body_timeout: 2.0
```

## Lanzamiento del Sistema en Tiago

Ejecuta los siguientes comandos en orden, cada uno en una terminal separada:

### 1. Detección Facial
```bash
ros2 launch hri_face_detect face_detect.launch.py \
  filtering_frame:=base_footprint \
  rgb_camera:=/head_front_camera/rgb/image_raw \
  camera_frame_id:=head_front_camera_color_optical_frame
```

### 2. Detección Corporal
```bash
ros2 launch hri_fullbody hri_fullbody.launch.py \
  rgb_camera:=/head_front_camera/rgb/image_raw \
  camera_frame_id:=head_front_camera_color_optical_frame
```

### 3. Configuración de Transformadas
```bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id head_front_camera_color_optical_frame \
  --child-frame-id default_cam
```

### 4. Gestor de Personas
```bash
ros2 launch hri_person_manager person_manager.launch.py \
  robot_reference_frame:=base_footprint \
  reference_frame:=map
```

### 5. Detección de Engagement
```bash
ros2 launch hri_engagement hri_engagement.launch.py
```

### 6. Cambio LEDs
```bash
ros2 launch engagement_action robot_led_control.launch.py 
```