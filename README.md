# GeoBot-Final Project for HRI
INFO 5356-030: Introduction to Human-Robot Interaction

## Overview
This ROS2 package implements an interactive quiz robot that uses LiDAR sensing to detect participants and navigate to their positions. The system combines deliberative planning, reactive sensing, and coordinated execution in a three-layer control architecture.

## Package Structure

```
src/
├── package.xml                      # ROS2 package manifest
├── setup.py                         # Python package configuration
├── robot_control_architecture_pkg/
│   ├── robot_quiz_lidar_node.py    # Main control node (FSM + LiDAR)
│   └── simple_tts_node.py          # Text-to-speech node
└── Other files                       # For compiling
```

## File Descriptions


### `robot_quiz_lidar_node.py` 
- **Subscribers**: `/scan` (LaserScan), `/quiz_correct` (String)
- **Publishers**: `/cmd_vel` (Twist), `/say_text` (String)
- **Finite State Machine**: 7-state cycle (IDLE → ORIENT → ADVANCE → STOP_AT_GOAL → TURN_BACK → RETURN → FACE_FORWARD)
- **Key Functions**:
  - `plan_for_answer()`: Calculates motion parameters based on LiDAR detection
  - `estimate_distance()`: Detects participants in target zones
  - `loop()`: Executes state transitions and publishes motion commands

### `simple_tts_node.py`
- Subscribes to `/say_text` topic
- Synthesizes speech using `pyttsx3` library
- Runs independently from main control loop

### `setup.py` & package.xml
- Package metadata (name, description, maintainer)
- Entry points for executables: `robot_quiz_lidar_node`, `simple_tts_node`
- Launch file configuration
- Defines package name, description, and maintainer information
- Declares dependencies: rclpy, sensor_msgs, geometry_msgs, std_msgs


## System Architecture

### Three-Layer Control Architecture

**Deliberative Layer (Top)**
- 7-state FSM defines sequential quiz interaction behavior
- States: IDLE → ORIENT → ADVANCE → STOP_AT_GOAL → TURN_BACK → RETURN → FACE_FORWARD → IDLE

**Coordination Layer (Middle)**
- `plan_for_answer()`: Receives answers, queries reactive layer for distance, calculates timing parameters
- `loop()`: Generates velocity commands, triggers state transitions when `phase_end` is reached

**Reactive Layer (Bottom)**
- Subscribes to `/scan` for real-time LiDAR data
- `estimate_distance()`: Detects participants in 30° sectors around target directions (A/B/C/D)

### Data Flow
```
External Input (/quiz_correct) ──┐
                                  ├──> Deliberative FSM ──> Coordination Layer ──> /cmd_vel (motion)
LiDAR (/scan) ───> Reactive Layer ┘                      └──> /say_text (speech)
```


## Demo Video
📹 [Watch the demo](https://drive.google.com/file/d/1EbMJ2bmjRJsywzv14wcY85f6ChZG1JWt/view?usp=sharing)

## Running the System

```bash
# Terminal 1: Launch robot control node
ros2 run quiz_robot robot_quiz_lidar_node

# Terminal 2: Launch TTS node
ros2 run quiz_robot simple_tts_node

# Terminal 3: Send quiz answers
ros2 topic pub /quiz_correct std_msgs/String "data: 'A'"
```


## Contributors
- Xinwei Xie: xx374@cornell.edu
- Rui Chen: rc986@cornell.edu
- Zifan Yang: zy489@cornell.edu
