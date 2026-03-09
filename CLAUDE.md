# Robotics Project Overview

This project is a Python-based robotics control system running on a Raspberry Pi 4 Model B. It features a modular, multi-threaded architecture that handles hardware motor control, IMU sensor fusion for 2D odometry, a live camera feed, and a real-time web-based plotting interface.

## 1. Hardware Specifications
* **Brain**: Raspberry Pi 4 Model B (Accessed via SSH, headless).
* **Motors**: Pololu 12V motors with encoders.
* **Motor Controller**: RoboClaw (Connected via Serial `/dev/ttyAMA0`, Address `0x80`).
* **IMU**: LSM6DSV16X (Connected via I2C, Bus 1, Address `0x6B`).
* **Camera**: Standard USB Web Camera (`/dev/video0`).
* **Power Distribution**:
    * **Source**: 3-cell Li-ion battery pack (~12V).
    * **Motors**: Powered directly from the 12V pack via the RoboClaw.
    * **Raspberry Pi**: Powered via a 5V (3A+) Buck Converter stepping down the 12V pack.

## 2. Software Architecture
* `main.py`: The central orchestrator. Starts all threads (Camera, IMU, Plotter). Injects 2D odometry logic into the robot control loop via monkey-patching `handle_command`. Tracks position where X = Forward and Y = Lateral.
* `robot_controller.py`: Contains `RobotInterface`, `MotorController`, and `MovementController`. Handles keyboard input, encoder reading, and distance driving commands using a step size of `0.256` meters.
* `roboclaw.py`: The low-level Python 3 compatible serial driver for the motor controller.
* `imu_thread.py`: Background thread that continuously polls the IMU, applies calibration offsets, and integrates gyro rates to maintain global Euler angles (Roll, Pitch, Yaw).
* `imu_driver.py`: Low-level I2C hardware driver for the LSM6DSV16X. Configured for ±2g accel and ±125dps gyro at 120Hz.
* `camera_server.py`: Aggressive OpenCV camera driver streaming via a Flask server on **Port 5000**.
* `web_plotter.py`: Headless Matplotlib server using Flask on **Port 5001**. Draws the robot's real-time X/Y path. Operates in `Agg` mode to prevent GUI thread blocking on the headless Pi.

## 3. Known Hardware Quirks & Solutions
* **Camera Initialization Errors (`select() timeout`)**: The Raspberry Pi legacy camera stack frequently hangs. `camera_server.py` forces the hardware to 640x480 resolution and MJPG compression to prevent bandwidth lockups. It also uses a `v4l2-ctl` subprocess kickstart on boot. **Requirement**: "Legacy Camera Support" must be enabled in `raspi-config`.

## 4. Coding Standards & Conventions
* **Comments**: When writing code in any programming language, all comments must be explicitly written in English.
* **UI/UX**: The main terminal interface must remain fast and non-blocking. Heavy tasks (like Matplotlib rendering or OpenCV encoding) must always be offloaded to daemon threads or separate Flask servers.
