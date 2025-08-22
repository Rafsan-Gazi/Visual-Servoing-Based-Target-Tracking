
# Visual Servoing-Based LED Target Tracking with TurtleBot3.

This project implements a ROS 1 pipeline to detect, track, and drive toward a blinking LED marker using a calibrated camera and real-time visual feedback.

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
```

---

## Results

- TurtleBot3 successfully tracked and approached the blinking LED marker
- Demonstrated consistent stopping behavior within 10 cm of the LED
- Modular node structure enabled flexible testing and debugging
- Visual servoing robust to noise, background clutter, and LED blink timing

---

## Dependencies

- **ROS Noetic**
- **Python 3**
- `rospy`, `cv_bridge`, `numpy`, `opencv-python`
- `rosserial`, `tf`, `sensor_msgs`, `geometry_msgs`, `image_transport`

---

## How to Run

1. Launch the robot and bring up the camera node:
   ```bash
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   rosrun raspicam_node raspicam_node
   ```

2. Start image capture and LED frame sampling:
   ```bash
   rosrun your_package_name CameraLab8.py
   ```

3. Track LED location and publish world coordinates:
   ```bash
   rosrun your_package_name trackLab10B.py
   ```

4. Launch the navigation/control node:
   ```bash
   rosrun your_package_name motor_control.py
   ```

---

## Author

This project was completed by **Gazi Abdullah Mashud**.

