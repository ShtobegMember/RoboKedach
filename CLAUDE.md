# "RoboKedach" Robotics Project Context & Architecture Guide

## 1. Project Background
This project is an industrial-grade robotics platform currently transitioning from a standalone Python multiprocessing architecture to a distributed Robot Operating System (ROS) architecture. The ultimate goal is to achieve live SLAM (Simultaneous Localization and Mapping), utilizing an optic fiber tether to separate hardware I/O from heavy computational tasks (Heads-Up Display, SLAM, kinematics).

**Architectural Strictness:** ROS is strictly quarantined to handle the LIDAR sensor, IMU, and SLAM (Simultaneous Localization and Mapping). All other hardware operations (Motors, Camera) are managed entirely by pure Python scripts. The IMU's I2C bus is exclusively owned by ROS — it feeds Cartographer for SLAM quality rather than being read by a standalone Python driver. Furthermore, basic dead-reckoning odometry (via encoders and IMU) has been permanently abandoned; all spatial tracking and mapping will be handled exclusively by the LIDAR SLAM integration.

## 2. Hardware Specifications

### Core Computing & Networking
* **Robot Brain (SBC)**: Raspberry Pi 4 Model B.
* **Base Station (Computer)**: Windows 11 64-bit, with Ubuntu 24.04 on WSL.
* **Network Tether**: 1 Gbps Optic Fiber link connecting the Pi and Base Station.
* **ROS Version**: ROS 2 Kilted Kaiju.

### Power Distribution
* **Power Source**: 3-cell Li-ion battery pack (~12V).
* **Motor Power**: Direct 12V from battery to motor controller.
* **Pi Power**: 5V (3A+) Buck Converter stepping down from 12V.

### Sensors & Actuators
* **Motors**: Pololu 131 12V motors with encoders (8400 ticks per cycle).
* **Locomotion (Whegs)**: The robot utilizes "whegs" (half-circle legs) instead of standard continuous-rotation wheels. This mechanical design requires strict constant-phase synchronization between the left and right motors to maintain a stable gait. Consequently, motor movement logic should use precise, encoder-based positional tracking (driving specific fractions of a rotation cycle) rather than simple open-loop velocity commands.
* **Motor Controller**: RoboClaw. Connected via Serial `ttyAMA0`, Baud Rate: `38400`, Address: `0x80`.
* **IMU**: LSM6DSV16X. Connected via I2C Bus `1`, Address `0x6B`. Configured for atomic burst reads at 120Hz, ±2g accelerometer, and ±125dps gyroscope.
* **Voltage/Current Monitor**: INA226. Connected via I2C Bus `3` (GPIO 4 SDA / GPIO 5 SCL), Address `0x40`. Shunt resistor: R010 (10 mΩ). Monitors battery bus voltage and system current draw. Streamed to the PC at 2 Hz.
* **Camera**: Standard USB Web Camera on `/dev/video0`. Forced to `MJPG` compression and `640x480` resolution at `30` FPS to bypass Raspberry Pi USB bandwidth bottlenecks. Frames are served at full `640x480` (no downscale) with JPEG quality `80` over the 1 Gbps fiber link.
* **LIDAR**: RPLIDAR-C1, Connected via USB.

---

## 3. System Architecture: "Hardware on the Pi, Brains on the Computer"
The software is strictly decoupled to preserve the Raspberry Pi's CPU for hardware timing. The Pi acts as the "Nervous System" routing raw data, while the Base Station PC acts as the "Cerebral Cortex" executing all heavy logic.

### A. The Raspberry Pi (Hardware Nodes)
The Pi runs completely headless. **No heavy math, GUI rendering, or complex odometry calculations are permitted on the Pi.**
* **Pure Python Nodes**:
    * **Motor Engine**: Wraps the `roboclaw.py` driver. Listens to the optic fiber via Python sockets for velocity/movement commands from the Base Station.
    * **Camera Server**: Uses `camera_server.py` to serve the MJPEG video stream directly over an HTTP port, completely bypassing ROS to ensure zero-latency video.
    * **Voltage Monitor Streamer**: Runs as a daemon thread inside the IMU process. Reads bus voltage and current from the INA226 on I2C bus 3 and streams the data to the Base Station via TCP (port `65434`) at 2 Hz.
* **ROS Node**:
    * **LIDAR + IMU Driver**: Launched via `ros2 launch robot_bringup record_c1.launch.py`. Spins the LIDAR and polls the IMU, publishing `LaserScan` and `Imu` messages to the ROS network. Managed as a `multiprocessing.Process` by `rpi_main2.py` with auto-restart on failure and a 5-second cooldown before (re)start to let USB/I2C devices fully release.

### B. The Base Station Computer (Computational Nodes)
* **Pure Python Components**:
    * **The Brain (PyQt Dashboard)** (`pc_main2.py`): The Heads-Up Display (HUD).
        * Pulls the live video stream via HTTP to paint the background.
        * Captures user teleoperation inputs (keyboard/gamepad) and sends them to the Pi via Python sockets.
        * Overlays a transparent status bar showing camera, motor, power, and SLAM status.
        * Displays a Power Monitor HUD panel (voltage + current) with a battery health color indicator (green/yellow/red based on 3S Li-ion cell thresholds).
        * Manages the WSL/ROS2 SLAM lifecycle via a `SLAMWorker` QThread — auto-launches RViz2 + Cartographer on startup, provides a Start/Stop SLAM button to toggle live bag recording.
* **ROS Components** (launched inside WSL by the PyQt Dashboard):
    * **RViz2**: Visualization window, launched automatically alongside the dashboard.
    * **Cartographer SLAM**: Subscribes to the LIDAR scans and IMU data from the Pi. Computes SLAM to generate a live map.
    * **Bag Recording**: Optionally records `/scan`, `/imu/data`, `/tf`, `/tf_static` topics, toggled by the dashboard's Start/Stop SLAM button.

---

## 4. Coding Standards & Agent Directives
When modifying or generating code for this project, the coding agent MUST adhere to the following rules:

1.  **Language & Documentation**: All code comments, regardless of the programming language used, must be written in English.
2.  **Concurrency / Separation of Concerns**: Maintain strict boundaries between I/O bound tasks and CPU bound tasks. Use `multiprocessing` for CPU-heavy tasks to bypass the Python GIL.
3.  **Hardware Interactions**: Hardware reads (like I2C IMU polling) must remain atomic (e.g., Burst Reads) to prevent data tearing. Error handling must account for dropped serial packets gracefully without crashing the main loop.

---

## 5. Folders Context & Guide

### Non-Product Folders
* The folders `IMU_BMI270`, `IMU_LSM6DSV16X`, and `VM_INA226` are independent of each other and are used to test the IMU & VM components only.
* The folder `drafts` contains some old codes and tests, ignore it.

### Product Folders
* The folder `MVP` contains the finished code for the Minimum-Viable-Product of the project. It runs by itself on the previous version of the robot, and is the base for the next iteration and final product.
* The folders `PC` and `RPI` inside `GUI` contain the in-the-works code for the PC and Raspberry-Pi sides in the final product. The `v2` files (`pc_main2.py`, `rpi_main2.py`) are the current active versions — they integrate SLAM lifecycle management and remove standalone IMU handling in favor of ROS-managed IMU. The `v1` files (`pc_main.py`, `rpi_main.py`) include IMU, camera, motor control, and INA226 voltage/current monitoring. When SLAM integration is finished, all will be merged into a final version.

### ROS WSL Launcher (`GUI/ros_wsl.py`)
A standalone Python script that launches the entire ROS2 SLAM pipeline from Windows by spawning processes inside WSL (Ubuntu 24.04). It is **not** a ROS node itself — it uses `subprocess.Popen` to run `wsl` commands that start ROS2 processes inside the Linux environment.

* **Two modes** controlled by a parameter to `main()`:
    * `"playback"` — replays a pre-recorded rosbag (`.mcap`) with `--clock` for offline SLAM testing.
    * `"live"` — launches sensor drivers and records a new rosbag session.
* **Session lifecycle**: RViz2 acts as the session anchor. All other processes (Cartographer SLAM, bag playback/recording) are launched alongside it. When the user closes the RViz2 window, the script runs `pkill ros2` inside WSL to terminate the Linux-side ROS2 processes, then `.terminate()`s the remaining Windows-side `wsl.exe` wrappers.
* **Integration note**: The dashboard's `SLAMWorker` in `pc_main2.py` reimplements the WSL subprocess launching logic directly rather than calling `ros_wsl.py`. This script remains useful as a standalone debugging/playback tool.
