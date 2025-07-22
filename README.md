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
- **Voice Interaction**: Sistema de interacción por voz que detecta comandos y gira hacia la fuente del sonido

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

# ReSpeaker para interacción de voz
git clone https://github.com/igonzf/respeaker_ros.git
```

## Installation

```bash
# Create a workspace
mkdir -p ~/ros2_ws/ros4hri_ws/src
cd ~/ros2_ws/ros4hri_ws/src

# Clone this repository
git clone https://github.com/krisia13/ros4hri-engagement.git

# Clone dependencies (see list above)

# Install voice interaction dependencies
pip install SpeechRecognition numpy
sudo apt-get install python3-pyaudio portaudio19-dev
sudo apt-get install ros-humble-nav2-msgs ros-humble-tf2-ros

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

## Modulos de Tiago

En http://10.68.0.1/
- Parar todos los módulos relacionados con hri
- Parar locatization y navigation
- Mantener led_manager_node y mm11_node (importante sino el cambio de color no funciona)

## Lanzamiento del Sistema en Tiago

Ejecuta los siguientes comandos en orden, cada uno en una terminal separada:

### 1. Detección Facial
```bash
ros2 launch hri_face_detect face_detect.launch.py \
  filtering_frame:=base_footprint \
  rgb_camera:=/head_front_camera/rgb/image_raw \
  camera_frame_id:=head_front_camera_color_optical_frame
```

### 2. Detección Corporal (no hace nada)
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
  reference_frame:=base_footprint
```

### 5. Detección de Engagement
```bash
ros2 launch hri_engagement hri_engagement.launch.py
```

### 6. Cambio LEDs
```bash
ros2 launch engagement_action robot_led_control.launch.py 
```

### 7. Visualización RViz
```bash
ros2 launch tiago_navigation tiago_nav2.launch.py 
```

### 8. Text to Speech-1
```bash
ros2 launch text_to_speech text_to_speech.launch.py
```

### 9. Text to Speech-2
```bash
ros2 launch engagement_action tts_node.launch.py 
```

### 10. Navigation Node
```bash
ros2 launch engagement_action navigation_node.launch.py 
```

### 11. Sistema de Interacción por Voz
```bash
ros2 launch engagement_action voice_interaction.launch.py
```

## Sistema de Interacción por Voz

El sistema incluye un módulo de interacción por voz que utiliza el micrófono ReSpeaker para:

### Funcionalidades
- **Detección de actividad de voz (VAD)**: Detecta automáticamente cuando alguien habla
- **Localización de fuente sonora (DOA)**: Determina la dirección de donde viene la voz
- **Reconocimiento de voz**: Convierte speech a texto usando Google Speech Recognition
- **Rotación automática**: El robot gira hacia la persona que le habla
- **Control de LEDs**: Feedback visual durante la interacción

### Palabras de Activación
El sistema responde a las siguientes palabras clave:
- `tiago`
- `hola`
- `oye` 
- `robot`
- `hola tiago`
- `ey tiago`

### Configuración del ReSpeaker
El launch file `voice_interaction.launch.py` incluye automáticamente:
- **respeaker_node**: Para captura y procesamiento de audio
- **voice_interaction_node**: Para detección de interacciones y control del robot

### Tópicos Publicados
- `/speech_text`: Texto reconocido de la voz
- `/interaction_detected`: Señal de interacción detectada
- `/status_led`: Control de LEDs del ReSpeaker

### Mapeo de Direcciones
El sistema mapea las direcciones del ReSpeaker a comandos de rotación:
- **0° (delante)**: No girar
- **90° (izquierda)**: Girar +90°
- **180° (detrás)**: Girar 180°
- **270° (derecha)**: Girar -90°

## Análisis de Voces TTS

El sistema text_to_speech soporta diferentes engines y configuraciones de voz. Tras realizar pruebas exhaustivas, se determinaron las mejores configuraciones:

### Engines Disponibles
```bash
int32 ESPEAK=1    # eSpeak synthesis engine
int32 SPD_SAY=2   # Speech Dispatcher 
int32 FESTIVAL=3  # Festival speech synthesis
int32 GTTS=4      # Google Text-to-Speech
```

### Resultados del Análisis

**Mejores opciones:**
- **Engine 4 (GTTS)**: Calidad de voz excelente, pronunciación natural y clara. No hay distinción m/f, suena siempre la misma voz femenina

**Opciones descartadas:**
- **Engine 1 (ESPEAK)**: Segunda mejor opción, voz más robótica
- **Engine 2 (SPD_SAY)**: Calidad de voz pobre, sonido robótico
- **Engine 3 (FESTIVAL)**: Pronunciación deficiente, voz poco natural


## Ajuste de Velocidades de Navegación

Para controlar la velocidad del robot durante la navegación, se han modificado los parámetros en el archivo `nav2_params.yaml`. Esto es especialmente útil para navegación en entornos con personas o para demostraciones.

### Parámetros Modificados

En la sección `controller_server` → `FollowPath` se han ajustado los siguientes valores para reducir la velocidad en un 50%:

```yaml
FollowPath:
  plugin: dwb_core::DWBLocalPlanner
  
  # VELOCIDADES LINEALES (reducidas de 0.3 a 0.15)
  max_vel_x: 0.15        # Velocidad máxima hacia adelante
  max_speed_xy: 0.15     # Velocidad máxima general
  
  # VELOCIDADES ANGULARES (reducidas de 0.6 a 0.3)
  max_vel_theta: 0.3     # Velocidad máxima de rotación
  
  # ACELERACIONES SUAVIZADAS
  acc_lim_x: 1.5         # Aceleración lineal (de 2.5 a 1.5)
  decel_lim_x: -1.5      # Deceleración lineal (de -2.5 a -1.5)
  acc_lim_theta: 2.0     # Aceleración angular (de 3.2 a 2.0)
  decel_lim_theta: -2.0  # Deceleración angular (de -3.2 a -2.0)
```

### Velocidades en Behavior Server

También se han ajustado las velocidades para comportamientos específicos (giros, retrocesos):

```yaml
behavior_server:
  ros__parameters:
    max_rotational_vel: 0.5  # Reducido de 1.0 a 0.5
    min_rotational_vel: 0.2  # Reducido de 0.4 a 0.2
```

## Configuración del Micrófono ReSpeaker

El sistema utiliza un micrófono ReSpeaker USB para captura de audio y procesamiento de voz.

### Dependencia ReSpeaker

```bash
# Clonar el repositorio ReSpeaker
cd ~/ros2_ws/ros4hri_ws/src
git clone https://github.com/igonzf/respeaker_ros.git
```

### Configuración de Permisos USB

Para que el usuario pueda acceder al dispositivo ReSpeaker sin permisos de administrador:

1. **Crear reglas udev**:
```bash
sudo nano /etc/udev/rules.d/99-respeaker.rules
```

2. **Añadir el siguiente contenido**:
```bash
SUBSYSTEM=="usb", ATTR{idProduct}=="0018", ATTR{idVendor}=="2886", MODE:="0666"
SUBSYSTEM=="usb", ATTRS{idProduct}=="0018", ATTRS{idVendor}=="2886", MODE:="0666"
```

3. **Aplicar las reglas**:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. **Desconectar y volver a conectar el ReSpeaker**

### Ejecución Automática

El sistema de interacción por voz se ejecuta automáticamente con el comando:

```bash
ros2 launch engagement_action voice_interaction.launch.py
```

Este launch file incluye automáticamente:
- **respeaker_node**: Nodo del micrófono ReSpeaker
- **voice_interaction_node**: Nodo de procesamiento de voz e interacción

**Nota**: Ya no es necesario ejecutar `ros2 run respeaker_ros respeaker_node` por separado, ya que está incluido en el launch file.


Para comprobar el reconocimiento de voz en tu sistema, sigue estos pasos:

1. **Ejecuta el nodo que publica el audio del micrófono:**
   ```sh
   ros2 run respeaker_ros respeaker_node
   ```

2. **Ejecuta el nodo que convierte el audio en texto usando Vosk:**
   ```sh
   ros2 run engagement_action vosk_asr_node
   ```

3. **Visualiza el texto reconocido en tiempo real:**
   ```sh
   ros2 topic echo /voz_texto
   ```

Así podrás ver en la terminal el texto que reconoce el sistema cuando hablas cerca del micrófono.