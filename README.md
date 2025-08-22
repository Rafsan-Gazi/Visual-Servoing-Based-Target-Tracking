# Visual Servoing-Based LED Target Tracking

This project implements a ROS 1 pipeline to detect, track, and drive toward a blinking LED marker using a calibrated camera and real-time visual feedback. The system was developed as part of a graduate-level robotics course at Miami University, Spring 2025.

---

## Overview

- **Platform:** TurtleBot3 (Burger) running on ROS Noetic
- **Objective:** Detect a blinking LED and autonomously drive the robot to stop within ~10 cm of the target
- **Core Technologies:** ROS 1, OpenCV, TF, Homography, Proportional Control

---

## Key Features

### Visual Perception & LED Tracking

- Subscribes to `/raspicam_node/image/compressed` for camera input
- Captures LED ON/OFF images using synchronized frame sampling
- Filters high-intensity pixels and performs blinking frequency analysis
- Detects LED centroid using contour detection and bounding box filtering
- Maps image coordinates to 2D world frame using homography
- Publishes LED coordinates to `/led_world_coord` topic

### Autonomous Navigation & Control

- Listens to LED location topic and calculates angular error
- Implements a proportional controller to generate `cmd_vel` messages
- Commands linear/angular velocity to rotate and move toward LED
- Stops the robot once within a predefined distance threshold from the target
- Operates reliably under varied lighting conditions

---

## Project Files

```bash
.
├── CameraLab8.py          # Captures LED ON/OFF images from ROS topic
├── distanceLab9.py        # Maps LED image coordinates to real-world coordinates
├── trackLab10.py          # Tracks LED centroid across blinking frames
├── trackLab10Still.py     # Static test version of LED tracker
├── trackLab10B.py         # Improved LED tracking logic
├── motor_control.py       # Robot control node based on LED angular error
├── lab11.py               # Final integration of perception and control
├── utilsLab9.py           # Utility functions for homography and filtering
├── README.md              # This file
