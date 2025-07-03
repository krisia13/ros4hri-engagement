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

## Known Issues

Some ROS4HRI packages have memory corruption issues:
- `hri_face_body_matcher`: Memory corruption when launched
- `hri_face_identification`: Memory corruption when loading face database

These packages are not required for basic engagement detection and are excluded from the recommended workflow.

## License

This project is licensed under the Apache License 2.0.