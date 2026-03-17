# "RoboKedach" Robotics Project Context & Architecture Guide

## 1. Project Background
This project is an industrial-grade robotics platform currently transitioning from a standalone Python multiprocessing architecture to a distributed Robot Operating System (ROS) architecture. The ultimate goal is to achieve live SLAM (Simultaneous Localization and Mapping), utilizing an optic fiber tether to separate hardware I/O from heavy computational tasks (Heads-Up Display, SLAM, kinematics).

**Architectural Strictness:** ROS is strictly quarantined to handle the LIDAR sensor and SLAM (Simultaneous Localization and Mapping). All other hardware operations (Motors, IMU, Camera) are managed entirely by pure Python scripts. Furthermore, basic dead-reckoning odometry (via encoders and IMU) has been permanently abandoned; all spatial tracking and mapping will be handled exclusively by the future LIDAR SLAM integration.

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
* **Camera**: Standard USB Web Camera on `/dev/video0`. Forced to `MJPG` compression and `640x480` resolution at `30` FPS to bypass Raspberry Pi USB bandwidth bottlenecks.
* **LIDAR**: RPLIDAR-C1, Connected via USB.

---

## 3. System Architecture: "Hardware on the Pi, Brains on the Computer"
The software is strictly decoupled to preserve the Raspberry Pi's CPU for hardware timing. The Pi acts as the "Nervous System" routing raw data, while the Base Station PC acts as the "Cerebral Cortex" executing all heavy logic.

### A. The Raspberry Pi (Hardware Nodes)
The Pi runs completely headless. **No heavy math, GUI rendering, or complex odometry calculations are permitted on the Pi.**
* **Pure Python Nodes**:
    * **Motor Engine**: Wraps the `roboclaw.py` driver. Listens to the optic fiber via Python sockets for velocity/movement commands from the Base Station.
    * **IMU Engine**: Uses `imu_driver.py` to poll the I2C bus for raw pitch/roll orientation data.
    * **Camera Server**: Uses `camera_server.py` to serve the MJPEG video stream directly over an HTTP port, completely bypassing ROS to ensure zero-latency video.
* **ROS Node**:
    * **LIDAR Driver**: The *only* ROS node on the Pi. It simply spins the laser and publishes raw `LaserScan` or `PointCloud2` messages to the ROS network.

### B. The Base Station Computer (Computational Nodes)
* **Pure Python Components**:
    * **The Brain (PyQt Dashboard)**: The Heads-Up Display (HUD).
        * Pulls the live video stream via HTTP to paint the background.
        * Captures user teleoperation inputs (keyboard/gamepad) and sends them to the Pi via Python sockets.
        * Overlays transparent telemetry gauges reading from the Pi.
* **ROS Components**:
    * **ROS Master (`roscore`)**: Manages the ROS directory from the stable Base Station.
    * **SLAM Engine**: Subscribes to the LIDAR point clouds from the Pi. Computes the SLAM algorithms to generate a live map and precise X/Y coordinates, which the PyQt Dashboard will ingest and display via PyQtGraph.

---

## 4. Coding Standards & Agent Directives
When modifying or generating code for this project, the coding agent MUST adhere to the following rules:

1.  **Language & Documentation**: All code comments, regardless of the programming language used, must be written in English.
2.  **Concurrency / Separation of Concerns**: Maintain strict boundaries between I/O bound tasks and CPU bound tasks. Use `multiprocessing` for CPU-heavy tasks to bypass the Python GIL.
3.  **Hardware Interactions**: Hardware reads (like I2C IMU polling) must remain atomic (e.g., Burst Reads) to prevent data tearing. Error handling must account for dropped serial packets gracefully without crashing the main loop.

---

## 5. Folders Context & Guide

### Non-Product Folders
* The folders `IMU_BMI270` and `IMU_LSM6DSV16X` are independent of each other and are used to test the IMU drivers only.
* The folder `drafts` contains some old codes and test, ignore it.

### Product Folders
* The folder `MVP` contains the finished code for the Minimum-Viable-Product of the project. It runs by itself on the previous version of the robot, and is the base for the next iteration and final product.
* The folders `PC` and `RPI` inside `GUI` contain the in-the-works code for the PC and Raspberry-Pi sides in the final product. It is not done and requires more integration of the MVP code.
