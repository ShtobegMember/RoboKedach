"""
lidar_node.py - Subprocess wrapper for the ROS2 SLAM hardware driver.
Spawns the sllidar node when triggered.
"""

import subprocess
import time
from core.config_loader import CONFIG

def run_lidar_process():
    """
    Launches the sllidar_ros2 node for the RPLIDAR.
    This process is triggered on-demand by the START_SLAM command received
    from the base station.
    """
    print("LIDAR: Initializing sllidar_ros2 node...")
    
    # 1. Load values dynamically from config.json
    #    Defaults to /dev/ttyUSB0 and laser_frame if missing from config
    lidar_cfg = CONFIG.get("hardware", {}).get("lidar", {})
    serial_port = lidar_cfg.get("port", "/dev/ttyUSB0")
    frame_id = lidar_cfg.get("frame_id", "laser_frame")
    
    # 2. Inject parameters into the ROS2 launch command
    cmd = [
        "ros2", "launch", "sllidar_ros2", "sllidar_launch.py",
        f"serial_port:={serial_port}",
        f"frame_id:={frame_id}"
    ]
    
    try:
        # Popen is used here instead of run() to allow the node to stream data 
        # continuously until the parent process terminates it.
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL, # Prevent ROS2 spam in the main console
            stderr=subprocess.PIPE,
            text=True
        )
        
        print(f"LIDAR: ROS2 Node started successfully on {serial_port} ({frame_id}).")
        
        # Block until the process is terminated externally (or crashes)
        _, stderr = process.communicate()
        
        if process.returncode != 0:
            print(f"LIDAR: Node exited with code {process.returncode}")
            if stderr:
                print(f"LIDAR: Error log: {stderr.strip()}")
                
    except FileNotFoundError:
        print("LIDAR: Error - 'ros2' command not found. Ensure ROS2 environment is sourced.")