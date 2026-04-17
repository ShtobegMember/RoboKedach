# "RoboKedach" Robotics Project — Context & Architecture Guide

## 1. Project Overview
RoboKedach is a tethered robotics platform that performs live SLAM (Simultaneous Localization and Mapping). An optic fiber tether connects a Raspberry Pi (on the robot) to a Windows Base Station PC. The Pi handles all hardware I/O — motors, camera, LIDAR, IMU, voltage monitoring — while the PC runs the operator dashboard (HUD), RViz2, and Cartographer SLAM inside WSL.

**Architectural Principle:** ROS 2 is strictly quarantined to the LIDAR, IMU, and SLAM pipeline. All other subsystems (motors, camera, voltage monitoring, heading tracking) use pure Python over TCP/HTTP sockets — no ROS involvement. I2C Bus 1 (IMU) is a **shared resource** with exclusive-access arbitration: either the heading tracker process or the ROS2 IMU driver node may use it, never both simultaneously — `rpi_main.py` enforces this mutex. On the Pi, subsystems run as separate `multiprocessing` processes for true parallel hardware I/O; on the PC, the PyQt6 dashboard uses `QThread` workers for concurrent network I/O and SLAM management alongside the UI.

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
* **IMU**: LSM6DSV16X on I2C Bus `1`, address `0x6B`. Atomic burst reads at 120Hz, ±2g accelerometer, ±250dps gyroscope. Hardware LPF on gyro (~30 Hz cutoff). Shared between the pre-SLAM heading tracker (`heading_tracker.py`) and the ROS2 IMU driver node — exclusive access enforced by `rpi_main.py` (never both at once). The heading tracker resets the IMU to factory defaults on exit so the ROS2 node inherits a clean state.
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

`rpi_main.py` is the process manager. It spawns three `multiprocessing.Process` workers on startup and monitors them in a health loop. If a process dies, it auto-restarts up to 3 times; after that, the process is abandoned with an error log. Two additional processes are launched on demand from PC commands: HeadingTracker (pre-SLAM heading) and LIDARNode (SLAM sensors).

**Startup order:** VMStreamer → MotorEngine → CameraServer. HeadingTracker and LIDARNode start on PC command.

**I2C Bus 1 Arbitration:** HeadingTracker and LIDARNode both need I2C Bus 1 (the IMU). `rpi_main.py` enforces mutual exclusion — it refuses to start one while the other is running. The operational sequence is: heading tracker runs first (during pipe descent), finishes and releases the bus, then LIDARNode launches for SLAM.

* **Pure Python Processes**:
    * **VMStreamer**: Reads bus voltage and current from the INA226 on I2C bus 3. Streams packed float pairs to the PC via TCP (port `65434`) at 2 Hz. Auto-reconnects if the PC is unreachable.
    * **MotorEngine**: TCP server (port `65433`) receiving movement commands from the PC. Wraps `roboclaw.py` via `robot_controller.py`. Supports full-rotation and quarter-rotation commands, uniform and differential speed adjustment, encoder reset, abort (via command, disconnect, or heartbeat timeout), and routing of heading tracker and SLAM commands. A background `socket_reader` thread parses incoming commands and routes them: movement commands go to a queue and set a `continue_move` flag for held-key continuous movement; `ABORT`/`STOP_MOVE` clear the flag; `HEARTBEAT` updates the watchdog timestamp. A separate `heading_result_relay` thread forwards heading tracker results (calibration status, final heading) back to the PC over the same TCP connection. After each blocking move, stale commands are drained from the queue.
    * **CameraServer**: Runs a Flask HTTP server (port `5000`) serving an MJPEG stream. A dedicated reader thread owns the camera exclusively — web clients grab frames from a shared buffer. Includes a `v4l2-ctl` hardware kickstart on startup to clear stale driver state.
    * **HeadingTracker**: Launched on demand via `START_HEADING_TRACK` command. Runs the Mahony AHRS filter on the LSM6DSV16X IMU through three phases: calibration → tracking → settling. Communicates with MotorEngine via `multiprocessing.Queue` pairs (commands in, results out). On completion, sends the final heading to the PC and resets the IMU to factory defaults before exiting, releasing I2C Bus 1 for the LIDARNode.
* **ROS Node**:
    * **LIDARNode**: Launched on demand via `START_SLAM` command from the PC (only after HeadingTracker has finished). Runs `ros2 launch robot_bringup record_c1.launch.py` in a bash subprocess, exporting `CYCLONEDDS_URI` and `ROS_DOMAIN_ID` in the same shell. Publishes `LaserScan` and `Imu` messages to the ROS network. Has a 3-second cooldown before each (re)start to let USB/I2C devices fully release. Runs `sudo chmod 666 /dev/ttyUSB0` before launch.

### B. The Base Station PC (`pc_main.py`)
* **The Dashboard (PyQt6 HUD)**:
    * Camera feed background (left 2/3 of screen) pulled via HTTP from the Pi's Flask server. RViz2 occupies the remaining right 1/3.
    * Keyboard teleoperation: Up/Left/Right arrows for movement (full rotation), Shift+arrow for 90-degree turns, +/- for uniform speed, Shift+/- for differential speed (left/right independent), Space for abort, R for encoder reset. Backward movement is disabled. Supports continuous movement while a key is held — `keyRelease` sends `STOP_MOVE` to end continuous mode after the current rotation completes.
    * A **heartbeat timer** (500ms) sends `HEARTBEAT` to the Pi; the Pi's motor engine aborts movement if no heartbeat arrives within 2 seconds (watchdog safety for disconnect/crash).
    * HUD overlay: bottom status bar (camera, motor, heading, speed, SLAM status), a Power Monitor panel (top-left) showing voltage + current with battery health color indicator (green >= 11.1V, yellow >= 10.2V, red below), and a Differential Speed panel (bottom-left) with three vertical bars (Left, Average, Right) showing the current speed settings.
    * **MotorCommandWorker** (QThread): TCP client to the Pi's motor server. Parses incoming messages: `SPEED:L,R`, `BUSY`, `READY`, `HEADING_CALIBRATED`, `HEADING:<float>`, `HEADING_ERROR:...`, and `ERROR:...`. Emits Qt signals for each.
    * **SLAMWorker** (QThread): Auto-launches RViz2 and Cartographer inside WSL on dashboard startup. Builds WSL commands via `WSL_ROS_PREAMBLE` which sources ROS + workspace setups and exports `CYCLONEDDS_URI`. All subprocesses use `stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL` to prevent pipe buffer deadlocks. Monitors process health; on shutdown, runs `pkill ros2` inside WSL and terminates Windows-side `wsl.exe` wrappers. Also publishes the North TF (static transform from `north` to `map` frame) once the heading is received. Four HUD buttons (top-right):
        * **Start Heading Track button**: Sends `START_HEADING_TRACK` to the Pi, triggering IMU calibration. Disables after press; status flows through `HEADING_CALIBRATED` → enables Landed button.
        * **Landed button**: Sends `HEADING_LANDED` to the Pi. Disabled until calibration completes. When the Pi returns `HEADING:<float>`, the heading is displayed, SLAMWorker publishes the North TF, and Start SLAM is enabled.
        * **Start SLAM button**: Sends `START_SLAM` to the Pi, triggering LIDARNode launch. Disabled until heading is received. One-shot — disables after press, re-enables only if SLAM core processes die.
        * **Record Bag button**: Toggles `ros2 bag record` of `/scan`, `/imu/data`, `/tf`, `/tf_static`. Only enabled after Start SLAM is pressed. Each recording gets a timestamped output name (`bag_YYYY-MM-DD_HH-MM-SS`) saved to `~/bags/`, allowing multiple recordings per session. Stop sends `SIGINT` via `pkill -INT` inside WSL for graceful flush, with a 5-second timeout fallback to force-terminate.

* **ROS Components** (launched inside WSL via `SLAMWorker`):
    * **RViz2**: Visualization, launched automatically on dashboard startup with a saved config (`mapper.rviz`).
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
6. **I2C Bus 1 Arbitration**: The heading tracker and the ROS2 LIDARNode (which includes the IMU driver) both use I2C Bus 1. They must never run simultaneously. Any new code that accesses Bus 1 must respect the existing arbitration in `rpi_main.py`. The heading tracker must reset the IMU to factory defaults on exit (`CTRL3_C = 0x01`) so the next consumer inherits a clean state.

---

## 5. Folder Guide

### Non-Product Folders
* **`Sensors/`** — Standalone test scripts for individual sensor components: `read_BMI270.py`, `read_LSM6DSV16X.py`, `read_INA26.py`, `imu_deg_calc.py`. Independent of each other and the product code.
* **`CONFIGS/`** — ROS2 configuration files deployed on the Pi and in the WSL workspace: Cartographer `.lua` tuning, robot URDF models. These are reference/backup copies — the live versions live in `~/ros2_ws/` on the Pi and `~/cartographer_ws/` in WSL.
* **`drafts/`** — Old codes & tests - ignore.

### Product Folders
* **`PRODUCT/pc_main.py`** — Base Station code (**current active version**). Heading tracking + SLAM + DDS + HUD dashboard.
* **`PRODUCT/RPI/`** — Raspberry Pi code (**current active version**).
    * `rpi_main.py` — Process manager: VMStreamer, MotorEngine, CameraServer, HeadingTracker, LIDARNode.
    * `heading_tracker.py` — Mahony AHRS heading tracker for pre-SLAM pipe descent.
    * `camera_server.py` — MJPEG streaming server.
    * `robot_controller.py` — Motor control, encoder-based movement, keyboard interface.
    * `roboclaw.py` — Low-level RoboClaw serial driver.
* **`MVP/`** — Finished Minimum-Viable-Product code. Runs standalone on the previous version of the robot. Contains: `main.py`, `robot_controller.py`, `roboclaw.py`, `camera_server.py`, `imu_driver.py`, `imu_thread.py`, `web_plotter.py`. Historical reference.
* **`FINAL/`** — Previous version (SLAM + DDS, no heading tracker). Superseded by PRODUCT.
* **`GUI/`** — Older version (standalone IMU, no SLAM). Closed / archived.

---

## 6. Precision Robotic Heading Optimization (The Heading/IMU Pipeline)

### Problem Description & Context
**The Mission:** RoboKedach must map environments starting up to 50 meters underground. The robot is manually aligned to sheer True North at the surface, and then lowered **face-down through a vertical pipe** for approximately 2 minutes. Once it hits the bottom, it transitions to horizontal crawling and initiates SLAM. The gap between the known heading at the surface (North) and the SLAM initialization underground means we must mathematically track the cumulative heading spin during the 2-minute freefall.

**Error Budget:** The product requirement is a 5% position error over 100 meters of SLAM mapping. A heading error of `theta` degrees produces a lateral error of `100 * sin(theta)` meters. Factoring in SLAM's internal topological error, our strict maximum permissible heading drift is **+/- 3 degrees**.

**Specifications & Limitations:**
*   **The Face-Down Gimbal Lock**: Because the robot descends face-down, gravity aligns with the robot's X-axis instead of Z. Therefore, the accelerometer is physically "blind" to the yaw tracking axis. We must rely entirely on Gyroscopic integration for heading calculation.
*   **Magnetometer Dead Zone**: The robot operates inside dense metal pipes, meaning magnetic compasses (Magnetometers) are continuously bombarded by ferromagnetic interference and are completely useless.
*   **Hardware Constraint**: The heading must be synthesized entirely from 6-axis data (Gyroscope + Accelerometer) using the LSM6DSV16X mechanical sensor mounted right-side up inside the chassis.
*   **Thermal/Time Drift**: Over a 2-minute integration spanning a 50-meter drop, the gyroscope bias shifts due to the intense temperature gradient between the surface and the underground environment, resulting in mathematical rotation drift. 
*   **The Bump Problem**: A 6-axis IMU uses Gravity (measured via the accelerometer) as an absolute "anchor" to stabilize Pitch and Roll. However, when the robot hits the pipe wall or swings on the tether, the accelerometer measures *linear/centripetal acceleration* alongside gravity. If the filter blindly trusts these "fake gravity" spikes, the resulting heading and pitch are catastrophically tilted.
*   **The Data Bottleneck**: The Python `smbus2` library running on the Raspberry Pi lacks real-time OS (RTOS) capabilities. Trying to extract data faster than ~120 Hz overloads the I2C pipeline, causing Python to drop bytes or choke on stale data.

**Requirements:**
*   Must achieve sub-degree heading precision and complete lack of idle drift when suspended.
*   Must be robust against intense sudden environmental jolts.
*   Must integrate cleanly into a ROS TF (Transform) architecture so the UI and map grid align absolutely to physiological "North".

### Current Solution & Implementation
To fulfill these requirements, we implemented a completely custom, Continuous-Gain Mahony Attitude and Heading Reference System (AHRS) filter operating at 120 Hz. The system logic is seamlessly distributed across the Pi hardware logic and PC ROS management.

#### 1. Hardware Configuration (`heading_tracker.py`)
*   **Gyroscope**: Configured to $\pm 250$ dps range to maximize ADC quantization resolution ($0.00875^\circ$/s per LSB).
*   **Accelerometer**: Configured to $\pm 2g$ to maximize resolution mapping the physical gravity vector.
*   **ODR (Output Data Rate)**: $120$ Hz strictly enforced limit. This allows Python exactly 8 milliseconds of breathing room to successfully retrieve atomic I2C reads without encountering a bus overload.
*   **Software-Defined *dt***: Because Linux Python loops experience CPU scheduler jitter, we dynamically calculate exact delta time (`dt`) via `time.time()` for every single iteration, making the continuous numeric integration completely immune to OS throttling.

#### 2. The Algorithm (Custom Mahony AHRS + RK4)
We operate a classic Mahony complementary filter augmented with several aerospace-grade upgrades:
*   **RK4 Integration**: Instead of crude Euler integration (`angle = angle + rate * dt`), we utilize mathematically rigorous Runge-Kutta 4th Order quaternion integration, virtually eliminating discrete-step rounding errors at high rotation speeds.
*   **Continuous Gain Scaling (Jolt Handling)**: Rather than instantly shutting off trust during a bump, we use a continuous mathematical scaling curve on the Acceleration vector:
    *   **Deadband (0 - 0.05g deviation)**: 100% trust applied. Treats data as normal robotic background mechanical vibration.
    *   **Attenuation (0.05g - 0.20g deviation)**: Trust linearly tapers down from 100% to 0%. The algorithm generously grants Pitch correction but smoothly desensitizes it to prevent violent tilting during scraping against the wall.
    *   **Hard Cutoff (> 0.20g deviation)**: 0% trust applied. The filter operates entirely blind on the gyro, assuming the physical jolt renders the "downward" gravity vector a lie.
    *   *Note*: This scaling is algorithmically applied to BOTH the proportional ($K_p$) and integral ($K_i$) gains to prevent the script from accidentally memorizing a jolt as permanent "gyro bias".

#### 3. ZUPT (Zero Velocity Update)
To achieve zero-drift when idle (especially crucial for precise pre-descent calibration testing):
*   If the exact rotational speed is less than $0.8^\circ$/s (`ZUPT_GYRO_THRESH`), and the acceleration magnitude perfectly rests within $5\%$ of $1G$ (`ZUPT_ACCEL_THRESH`), the algorithm physically freezes the raw gyro inputs to strictly `0.0`. This prevents random-walk sensor noise from trickling into the integrator and creating phantom drift.

#### 4. The ROS TF Injection (`pc_main.py`)
Finally, Cartographer requires a spatial frame to bind the SLAM output to physical reality.
*   When the operator completes the tracking phase and clicks "Landed", the Pi runs the settling phase, computes the final heading, and transmits it over TCP as `HEADING:<degrees>`.
*   The `HUDWindow` receives it, calls `SLAMWorker.publish_north_tf(heading_deg)`, which spawns a `static_transform_publisher` ROS2 node inside WSL. This node publishes a static transform from the `north` frame to the Cartographer `map` frame using `yaw = radians(heading - 90)`, firmly locking the UI's Cartesian rendering grid permanently to physical absolute North.

#### 5. Operational Sequence (Full Pipeline)
1. Operator aligns robot to True North at the surface.
2. Clicks **Start Heading Track** → Pi calibrates IMU (stationary, ~4s).
3. Pi returns `HEADING_CALIBRATED` → operator lowers robot into the pipe (tracking phase).
4. Robot lands underground → operator clicks **Landed** → Pi settles, computes heading, returns `HEADING:<deg>`.
5. PC publishes the North TF, enables **Start SLAM**.
6. Operator clicks **Start SLAM** → Pi launches LIDARNode (LIDAR + ROS IMU driver).
7. Cartographer begins mapping; operator drives via keyboard; optionally records bags.
