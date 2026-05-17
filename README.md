
----
RoboKedach is a tethered robotics platform designed for vertical pipe descent and 2D SLAM mapping. The system uses a Raspberry Pi for hardware interfacing and a Windows PC for visualization and high-level computation.

## 2. Distributed Architecture
The project is split across three logical machines, connected via a fiber optic tether (Subnet `192.168.1.0/24`).

### A. Raspberry Pi (RPI) - "The Body"
*   **Role**: Real-time hardware I/O and sensor data streaming.
*   **Code**: `PRODUCT/RPI/rpi_main.py` (Process Manager).
*   **Key Components**:
    *   **MotorEngine**: Controls RoboClaw and wheels.
    *   **CameraServer**: MJPEG stream from USB camera.
    *   **VMStreamer**: Battery/Current monitoring via INA226.
    *   **HeadingTracker**: Mahony AHRS filter for pipe descent.
    *   **LIDARNode**: ROS2 drivers for LIDAR-C1 and LSM6DSV IMU.

### B. Windows Desktop System (WDS) - "The Command Center"
*   **Role**: Graphical User Interface, Master Controller, and Networking Bridge.
*   **Code**: `PRODUCT/pc_main.py` (PyQt6 HUD).
*   **Responsibilities**:
    *   Automates RPI startup via SSH and WSL startup via local subprocesses.
    *   Coordinates the 120Hz DDS network traffic (Cyclone DDS).
    *   Translates keyboard inputs into robotic movement.

### C. WSL2 (Ubuntu 24.04) - "The Brain"
*   **Role**: ROS2 Environment for heavy computational tasks.
*   **Key Components**:
    *   **Google Cartographer**: Performs SLAM using data streamed from the RPI.
    *   **RViz2**: Visualizes the map, robot pose, and LIDAR points.
    *   **Bag Recorder**: Saves sensor data to `.mcap` files for post-processing.

---

## 3. The Configuration File (`config.json`)
The system is driven by a shared `config.json`. Ensure this file is identical on both the PC and the RPI.

*   **`network`**:
    *   `rpi_ip` (`192.168.1.2`) / `pc_ip` (`192.168.1.1`): Tether static IPs.
    *   `ssh`: Credentials for the PC to automatically launch the Pi software.
    *   `motor_port` / `vm_port` / `camera_port`: TCP/HTTP ports for internal comms.
*   **`wsl`**:
    *   `distro`: Name of the WSL distribution (e.g., `Ubuntu-24.04`).
    *   `path`: Local workspace path inside WSL.
    *   `core_commands`: The ROS2 launch strings for SLAM and RViz.
*   **`hardware`**:
    *   `motors`: Encoder resolution (`8400`), speed limits, and `leg_offsets` (used for the HUD leg animation).
    *   `roboclaw`: Serial port and baudrate settings.
    *   `ina226`: I2C address and bus for power monitoring.

---

## 4. Operation Guide

### Step 1: Physical Setup
1.  Connect the fiber tether between the RPI and the Windows PC.
2.  Power on the robot (12V battery).
3.  Ensure the Windows Ethernet interface is set to Static IP `192.168.1.1`.

### Step 2: Launching the App
1.  On the Windows PC, run the dashboard: `python PRODUCT/pc_main.py`.
2.  The dashboard will automatically:
    *   SSH into the Pi to start the process manager.
    *   Open RViz2 and the SLAM core inside WSL.
    *   Initialize the live HUD overlay.

### Step 3: Deployment (Pipe Descent)
1.  **Align North**: Physically align the robot to True North at the surface.
2.  **Track Heading**: Click **"Track Heading"**. Keep the robot still for 4 seconds during calibration.
3.  **Descent**: Lower the robot into the vertical pipe. The HUD will show "Tracking...".
4.  **Landing**: When the robot hits the bottom and is horizontal, click **"Landed"**. This locks the map grid to absolute North.

### Step 4: Mapping
1.  **Start SLAM**: Click **"Start SLAM"**. This launches the LIDAR on the RPI.
2.  **Drive**: Use arrow keys to navigate.
    *   `Up`: Full rotation forward.
    *   `Shift + Left/Right`: 90° pivot turns.
    *   `Space`: Emergency Stop.
    *   `+/-`: Increase/Decrease speed.
3.  **Record**: Toggle **"Record Bag"** to save mapping data.

---

## 5. Hardware Specifications (Technical Reference)
* **Brain**: Raspberry Pi 4 Model B.
* **Network**: 1 Gbps Optic Fiber via Cyclone DDS (Unicast transport).
* **DDS Config**: Pinned to the fiber interface to prevent WSL networking from freezing the PC.
* **Motors**: Pololu 131 12V motors with 8400 tick encoders.
* **IMU**: LSM6DSV16X (I2C Bus 1). Atomic burst reads at 120Hz.
* **LIDAR**: RPLIDAR-C1. High baudrate (460,800).
* **Power Monitor**: INA226 (I2C Bus 3). Monitors battery sag to prevent LIDAR brownouts.

---