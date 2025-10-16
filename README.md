# ROS2 Rover — URC 2026 Simulation (Turtlesim + Camera)

## Overview
This repository implements a modular ROS2 architecture that simulates URC-like rover missions using **Turtlesim** for mobility and a camera node (webcam or dummy) for vision. It demonstrates Science, Delivery, Equipment Servicing, and Autonomous Navigation missions using ROS2 paradigms (topics, services, actions, parameters).

Tested on **ROS2 Jazzy** (docs used: Jazzy release pages and Python action tutorial). :contentReference[oaicite:2]{index=2}

## Repo layout
(see listing in README top-level in repository)

## How to build (Jazzy)
1. Place both packages (`ros2_rover_interfaces`, `ros2_rover`) into `~/ros2_ws/src/`.
2. From workspace root:
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_rover_interfaces ros2_rover
source install/setup.bash
