# "RoboKedach" Robotics Project — Context & Architecture Guide

## 1. Project Overview
RoboKedach is a tethered robotics platform that performs live SLAM (Simultaneous Localization and Mapping). An optic fiber tether connects a Raspberry Pi (on the robot) to a Windows Base Station PC. The Pi handles all hardware I/O — motors, camera, LIDAR, IMU, voltage monitoring — while the PC runs the operator dashboard (HUD), RViz2, and Cartographer SLAM inside WSL.

**Architectural Principle:** ROS 2 is strictly quarantined to the LIDAR, IMU, and SLAM pipeline. All other subsystems (motors, camera, voltage monitoring) use pure Python over TCP/HTTP sockets — no ROS involvement. The IMU's I2C bus is exclusively owned by its ROS2 driver node; no standalone Python code reads it. On the Pi, subsystems run as separate `multiprocessing` processes for true parallel hardware I/O; on the PC, the PyQt6 dashboard uses `QThread` workers for concurrent network I/O and SLAM management alongside the UI.

## 2. Hardware Specifications

### Core Computing & Networking
* **Robot Brain**: Raspberry Pi 4 Model B (headless).
* **Base Station**: Windows 11 64-bit PC, with Ubuntu 24.04 on WSL2.
* **Network Tether**: 1 Gbps Optic Fiber link. Pi IP: `192.168.1.2`, PC IP: `192.168.1.1`.
* **ROS Version**: ROS 2 Kilted Kaiju (Cyclone DDS middleware).

### Power Distribution
* **Power Source**: 3-cell Li-ion battery pack (~12V).
* **Motor Power**: Direct 12V from battery to motor controller.
* **Pi Power**: 5V 5A Buck Converter stepping down from 12V.
* **USB Power**: All 4 USB ports on the RPi 4 share a single VL805 USB controller with one shared 5V power rail — no per-port power isolation. Low-resistance wiring (short, thick cables) between the buck converter and the Pi is critical to keep the USB 5V rail stable when camera and LIDAR draw simultaneously. Voltage sag on this rail causes the RPLIDAR-C1 to fail its internal health check (status 2).

### Sensors & Actuators
* **Motors**: Pololu 131 12V motors with encoders (8400 ticks per cycle).
* **Locomotion (Whegs)**: The robot uses "whegs" (half-circle legs) instead of standard wheels. This requires strict constant-phase synchronization between left and right motors. Motor movement logic uses precise encoder-based positional tracking (driving specific fractions of a rotation cycle) rather than open-loop velocity commands.
* **Motor Controller**: RoboClaw. Serial `/dev/ttyAMA0`, baud `38400`, address `0x80`.
* **IMU**: LSM6DSV16X on I2C Bus `1`, address `0x6B`. Atomic burst reads at 120Hz, ±2g accelerometer, ±125dps gyroscope. Exclusively managed by its ROS2 driver node — feeds Cartographer for SLAM quality.
* **Voltage/Current Monitor**: INA226 on I2C Bus `3` (GPIO 4 SDA / GPIO 5 SCL), address `0x40`. Shunt resistor: R010 (10 mOhm). Monitors battery bus voltage and system current draw.
* **Camera**: USB webcam on `/dev/video0`. Forced to MJPG compression, `640x480` at 30 FPS (RPi USB bandwidth constraint). Mounted upside-down (180 degree flip in software). Served over HTTP with JPEG quality 80 via Flask.
* **LIDAR**: RPLIDAR-C1 via USB serial adapter on `/dev/ttyUSB0`, baudrate `460800`. The C1 firmware has an internal health check that refuses to scan (status 2) if the 5V supply sags.

---

## 3. System Architecture: "Hardware on the Pi, Brains on the Computer"

### DDS Networking
Both sides export a Cyclone DDS configuration via the `CYCLONEDDS_URI` environment variable. This pins all DDS traffic to the fiber interface and restricts multicast to discovery-only (SPDP) packets — data transport uses unicast. Without this configuration, DDS multicasts discovery AND data on ALL network interfaces; under WSL2 mirrored networking this floods the Windows network stack and freezes the PC.

* **Pi** (`rpi_main.py`): `NetworkInterfaceAddress` = `192.168.1.2`
* **PC** (`pc_main.py`): `NetworkInterfaceAddress` = `192.168.1.1`
* **Both sides**: `AllowMulticast` = `spdp`, `ROS_DOMAIN_ID` = `1`

### A. The Raspberry Pi (`rpi_main.py`)
The Pi runs headless. **No GUI, heavy math, or SLAM computation runs on the Pi.**

`rpi_main.py` is the process manager. It spawns four `multiprocessing.Process` workers and monitors them in a health loop. If a process dies, it auto-restarts up to 3 times; after that, the process is abandoned with an error log.

**Startup order:** VMStreamer → MotorEngine → CameraServer → LIDARNode.

* **Pure Python Processes**:
    * **VMStreamer**: Reads bus voltage and current from the INA226 on I2C bus 3. Streams packed float pairs to the PC via TCP (port `65434`) at 2 Hz. Auto-reconnects if the PC is unreachable.
    * **MotorEngine**: TCP server (port `65433`) receiving movement commands from the PC. Wraps `roboclaw.py` via `robot_controller.py`. Supports full-rotation and quarter-rotation commands, speed adjustment, encoder reset, and spacebar abort. The abort mechanism works over the network by monkey-patching `robot_controller.get_key()`. After each blocking move, stale commands are drained from the queue.
    * **CameraServer**: Runs a Flask HTTP server (port `5000`) serving an MJPEG stream. A dedicated reader thread owns the camera exclusively — web clients grab frames from a shared buffer. Includes a `v4l2-ctl` hardware kickstart on startup to clear stale driver state.
* **ROS Node**:
    * **LIDARNode**: Launches `ros2 launch robot_bringup record_c1.launch.py` in a bash subprocess, exporting `CYCLONEDDS_URI` and `ROS_DOMAIN_ID` in the same shell. Publishes `LaserScan` and `Imu` messages to the ROS network. Has a 5-second cooldown before each (re)start to let USB/I2C devices fully release. Runs `sudo chmod 666 /dev/ttyUSB0` before launch.

### B. The Base Station PC (`pc_main.py`)
* **The Dashboard (PyQt6 HUD)**:
    * Full-screen camera feed background pulled via HTTP from the Pi's Flask server.
    * Keyboard teleoperation: arrow keys for movement (full rotation), Shift+arrow for 90-degree turns, +/- for speed, Space for abort, R for encoder reset. Supports continuous movement while a key is held — re-sends the command on each `READY` acknowledgment from the Pi.
    * HUD overlay: bottom status bar (camera, motor, speed, SLAM status) and a Power Monitor panel showing voltage + current with battery health color indicator (green >= 11.1V, yellow >= 10.2V, red below).
    * **SLAMWorker** (QThread): Auto-launches RViz2 and Cartographer inside WSL on dashboard startup. Builds WSL commands via `WSL_ROS_PREAMBLE` which sources ROS + workspace setups and exports `CYCLONEDDS_URI`. All subprocesses use `stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL` to prevent pipe buffer deadlocks. Monitors process health; on shutdown, runs `pkill ros2` inside WSL and terminates Windows-side `wsl.exe` wrappers.
    * **Bag recording**: Scaffolding exists (commented out) for a Start/Stop SLAM button that toggles rosbag recording of `/scan`, `/imu/data`, `/tf`, `/tf_static`. Not yet active.

* **ROS Components** (launched inside WSL via `SLAMWorker`):
    * **RViz2**: Visualization, launched automatically with a saved config (`mapper.rviz`).
    * **Cartographer SLAM**: `ros2 launch my_robot_slam online_slam.launch.py`. Subscribes to LIDAR scans and IMU data from the Pi, generates a live map.

### C. ROS Launch File (on Pi, not in this repo)
Located at `~/ros2_ws/src/robot_bringup/launch/record_c1.launch.py` on the Raspberry Pi. Launches:
1. `robot_state_publisher` with the robot URDF.
2. LSM6DSV16X IMU driver node (with `imu_config.yaml`).
3. RPLIDAR via `sllidar_c1_launch.py` from the `sllidar_ros2` package. C1-specific defaults (baudrate 460800, channel type) are handled internally.

---

## 4. Coding Standards & Agent Directives
When modifying or generating code for this project, the coding agent MUST adhere to the following rules:

1. **Language & Documentation**: All code comments must be written in English.
2. **Concurrency / Separation of Concerns**: Maintain strict boundaries between I/O bound and CPU bound tasks. Use `multiprocessing` for CPU-heavy tasks to bypass the Python GIL.
3. **Hardware Interactions**: Hardware reads (like I2C IMU polling) must remain atomic (e.g., burst reads) to prevent data tearing. Error handling must account for dropped serial packets gracefully without crashing the main loop.
4. **Subprocess Management**: Never use `subprocess.PIPE` for stdout/stderr unless the parent actively reads from the pipes. Use `subprocess.DEVNULL` for fire-and-forget subprocesses to prevent pipe buffer deadlocks (the ~64KB OS pipe buffer fills and the child process blocks forever).
5. **DDS Configuration**: Any new ROS2 launch point (on Pi or PC) must export `CYCLONEDDS_URI` with `NetworkInterfaceAddress` pinned to the fiber IP and `AllowMulticast` set to `spdp`. Omitting this floods all network interfaces and freezes the system under WSL2 mirrored networking.

---

## 5. Folder Guide

### Non-Product Folders
* **`Sensors/`** — Standalone test scripts for individual sensor components: `read_BMI270.py`, `read_LSM6DSV16X.py`, `read_INA26.py`, `imu_deg_calc.py`. Independent of each other and the product code.
* **`drafts/`** — Old codes & tests - ignore.

### Product Folders
* **`MVP/`** — Finished Minimum-Viable-Product code. Runs standalone on the previous version of the robot. Contains: `main.py`, `robot_controller.py`, `roboclaw.py`, `camera_server.py`, `imu_driver.py`, `imu_thread.py`, `web_plotter.py`. Serves as the base for the current iteration.
* **`FINAL/pc_main.py`** — Base Station code (current active version). SLAM + DDS + HUD dashboard.
* **`FINAL/RPI/`** — Raspberry Pi code (current active version).
    * `rpi_main.py` — ROS LIDAR/IMU + DDS.
    * `camera_server.py` — MJPEG streaming server.
    * `robot_controller.py` — Motor control, encoder-based movement, keyboard interface.
    * `roboclaw.py` — Low-level RoboClaw serial driver.
* **`GUI/`** — Previous version (standalone IMU, no SLAM). Closed / archived.
    * `GUI/PC/pc_main_gui.py` — Old PC dashboard.
    * `GUI/RPI/rpi_main_gui.py` — Old RPi main controller.
    * `GUI/RPI/imu_driver.py` — Standalone IMU driver (not used in FINAL; FINAL uses the ROS IMU node).
