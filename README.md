# ROS4HRI Engagement Detection System

This project implements a complete system for human detection, tracking, and engagement analysis using ROS 2. Development was carried out in two main phases:

## Phase 1: Initial Testing with Asus Xtion

In the first stage, the system was developed and validated using the Asus Xtion camera in a lab environment. All core ROS4HRI modules were tested: face detection, body detection, person management, and engagement analysis.

## Phase 2: Migration to Behavior Trees and Tiago Camera

After validating the system with Asus Xtion, it was migrated to run on the Tiago robot, using its integrated camera and a Behavior Tree-based architecture.


## Dependencies

This project requires the following ROS packages:

```bash
# Core ROS4HRI packages (required for human detection, tracking, and engagement)
git clone https://github.com/ros4hri/hri_msgs.git
git clone https://github.com/ros4hri/hri_actions_msgs.git
git clone https://github.com/ros4hri/hri_engagement.git
git clone https://github.com/ros4hri/hri_person_manager.git
git clone https://github.com/ros4hri/hri_face_detect.git
git clone https://github.com/ros4hri/hri_fullbody.git

# Asus Xtion camera driver (only needed for Phase 1: initial lab testing)
git clone https://github.com/mgonzs13/ros2_asus_xtion.git

# ReSpeaker voice interaction module (required for voice-based engagement and control)
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



# Configuration for Tiago Robot

This section contains the specific configuration required to run the engagement detection system on the Tiago robot.

## Configuration File

Create a configuration file at `~/.pal/config/ros4hri-tutorials.yml` with the following content:

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
    reference_frame: "base_footprint"  # Robot reference point, aligned with gaze direction
    max_distance: 4.0  # Increased to 4 meters for greater range
    field_of_view: 80.0  # Increased to 80 degrees for a wider attention field
    engagement_threshold: 0.4  # Lowered to make it easier to consider someone "engaged"
    observation_window: 5.0  # Reduced for faster response to changes
    rate: 10.0  # Kept as in the documentation

/hri_person_manager:
  ros__parameters:
    reference_frame: map
    robot_reference_frame: base_footprint
    use_auto_association: true
    association_threshold: 0.4
    face_timeout: 2.0
    body_timeout: 2.0
```

## Tiago Modules

On http://10.68.0.1/
- Stop all modules related to HRI
- Stop localization and navigation
- Keep led_manager_node and mm11_node (important, otherwise the color change will not work)

## Launching the System on Tiago

Execute the following commands in order, each in a separate terminal:

### 1. Face Detection
```bash
ros2 launch hri_face_detect face_detect.launch.py \
  filtering_frame:=base_footprint \
  rgb_camera:=/head_front_camera/rgb/image_raw \
  camera_frame_id:=head_front_camera_color_optical_frame
```

### 2. Body Detection
```bash
ros2 launch hri_fullbody hri_fullbody.launch.py \
  rgb_camera:=/head_front_camera/rgb/image_raw \
  camera_frame_id:=head_front_camera_color_optical_frame
```

### 3. Transform Configuration
```bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id head_front_camera_color_optical_frame \
  --child-frame-id default_cam
```

### 4. Person Manager
```bash
ros2 launch hri_person_manager person_manager.launch.py \
  robot_reference_frame:=base_footprint \
  reference_frame:=base_footprint
```

### 5. Engagement Detection
```bash
ros2 launch hri_engagement hri_engagement.launch.py
```

### 6. LED Control
```bash
ros2 launch engagement_action robot_led_control.launch.py 
```

### 7. RViz Visualization
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

### 11. Voice Interaction System
```bash
ros2 launch engagement_action voice_interaction.launch.py
```

## Voice Interaction System
The system includes a voice interaction module that uses the ReSpeaker Mic Array v2.0.


### USB Permissions Configuration

To allow the user to access the ReSpeaker device without administrator privileges:

1. **Create udev rules**:
```bash
sudo nano /etc/udev/rules.d/99-respeaker.rules
```

2. **Add the following content**:
```bash
SUBSYSTEM=="usb", ATTR{idProduct}=="0018", ATTR{idVendor}=="2886", MODE:="0666"
SUBSYSTEM=="usb", ATTRS{idProduct}=="0018", ATTRS{idVendor}=="2886", MODE:="0666"
```

3. **Apply the rules:**:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. **Disconnect and reconnect the ReSpeaker device**

### Testing Voice Recognition Functionality
To check voice recognition on your system, follow these steps:

1. **Run the node that publishes audio from the microphone:**
   ```sh
   ros2 launch engagement_action voice_interaction.launch.py
   ```

2. **Run the node that converts audio to text using Vosk:**
   ```sh
   ros2 run engagement_action vosk_asr_node
   ```

3. **View the recognized text in real time:**
   ```sh
   ros2 topic echo /voz_texto
   ```

This way, you can see the recognized text in the terminal when you speak near the microphone.

**Note**: `ros2 run respeaker_ros respeaker_node` it is included in the launch file.


## TTS Voice Analysis

The text-to-speech system supports several engines and voice configurations. After testing, the following options were evaluated:

### Available Engines
```bash
int32 ESPEAK=1    # eSpeak synthesis engine
int32 SPD_SAY=2   # Speech Dispatcher 
int32 FESTIVAL=3  # Festival speech synthesis
int32 GTTS=4      # Google Text-to-Speech
```

### Analysis Results

**Best option:**
- **Engine 4 (GTTS)**: Excellent voice quality, natural and clear pronunciation. No male/female distinction, always uses the same female voice.

**Discarded options:**
- **Engine 1 (ESPEAK)**: Second best, but more robotic voice.
- **Engine 2 (SPD_SAY)**: Poor voice quality, robotic sound.
- **Engine 3 (FESTIVAL)**: Poor pronunciation, unnatural voice.


## Navigation Speed Adjustment

To control the robot's speed during navigation, several parameters in the `nav2_params.yaml` file have been modified. This is especially useful for navigation in environments with people or for demonstration purposes.

### Modified Parameters

In the `controller_server` → `FollowPath` section, the following values have been adjusted to reduce the speed by 50%:

```yaml
FollowPath:
  plugin: dwb_core::DWBLocalPlanner
  
  # LINEAR SPEEDS (reduced from 0.3 to 0.15)
  max_vel_x: 0.15        # Maximum forward speed
  max_speed_xy: 0.15     # Maximum overall speed
  
  # ANGULAR SPEEDS (reduced from 0.6 to 0.3)
  max_vel_theta: 0.3     # Maximum rotational speed
  
  # SMOOTHED ACCELERATIONS
  acc_lim_x: 1.5         # Linear acceleration (from 2.5 to 1.5)
  decel_lim_x: -1.5      # Linear deceleration (from -2.5 to -1.5)
  acc_lim_theta: 2.0     # Angular acceleration (from 3.2 to 2.0)
  decel_lim_theta: -2.0  # Angular deceleration (from -3.2 to -2.0)
```

### Behavior Server Speeds

Speeds for specific behaviors (turns, reverses) have also been adjusted:

```yaml
behavior_server:
  ros__parameters:
    max_rotational_vel: 0.5  # Reduced from 1.0 to 0.5
    min_rotational_vel: 0.2  # Reduced from 0.4 to 0.2
```



## Final System Execution on Tiago

To launch the complete engagement detection and interaction system on Tiago, open four separate terminals and execute the following commands (one per terminal):

### Recommended Launch Order:

1. **Nav2 Navigation:**
   ```sh
   ros2 launch tiago_navigation tiago_nav2.launch.py
   ```

2. **Text-to-Speech:**
   ```sh
   ros2 launch text_to_speech text_to_speech.launch.py
   ```

3. **Automatic Speech Recognition (ASR):**
   ```sh
   ros2 run engagement_action vosk_asr_node
   ```

4. **Behavior Tree:**
   ```sh
   ros2 launch tiago_behavior_tree_cpp tiago_behavior_tree.launch.py
   ```

