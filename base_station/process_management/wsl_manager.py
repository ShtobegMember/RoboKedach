"""
wsl_manager.py - WSL and Remote Process Controller.
Handles launching Cartographer via local WSL subprocesses, toggling rosbag 
recordings, publishing static TF frames, and utilizing Paramiko for remote SSH shutdowns.
"""

import os
import time
import math
import logging
import subprocess
import paramiko
from core.config_loader import CONFIG

class WSLManager:
    def __init__(self):
        self.logger = logging.getLogger("WSL_MGR")
        self.wsl_distro = CONFIG["wsl"]["distro"]
        self.ros_domain_id = CONFIG["wsl"]["ros_domain_id"]
        
        # Pull the workspace path and commands dynamically from config.json
        self.workspace_path = CONFIG["wsl"].get("path", "~/cartographer_ws")
        self.core_commands = CONFIG["wsl"].get("core_commands", {})
        
        # Keep track of subprocesses
        self.tf_proc = None
        self.rviz_proc = None
        self.cartographer_proc = None
        self.rosbag_proc = None

    def _get_wsl_base_cmd(self):
        """Returns the base command prefix to execute bash commands inside WSL."""
        return [
            "wsl.exe", "-d", self.wsl_distro, "bash", "-c"
        ]

    def _wrap_ros_cmd(self, command_str: str) -> str:
        """Wraps a ROS2 command with the necessary sourcing and environment variables."""
        # Use the dynamic workspace path instead of hardcoding ~/cartographer_ws
        return (
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.workspace_path}/install/setup.bash && "
            f"export ROS_DOMAIN_ID={self.ros_domain_id} && "
            f"{command_str}"
        )

    def publish_north_tf(self, heading_deg: float):
        """Publishes the initial map-to-odom static transform based on the IMU heading."""
        if self.tf_proc:
            self.tf_proc.terminate()
            
        self.logger.info(f"Publishing static transform for heading {heading_deg}°")
        yaw_rad = math.radians(heading_deg)
        
        cmd = self._wrap_ros_cmd(
            f"ros2 run tf2_ros static_transform_publisher 0 0 0 {yaw_rad} 0 0 map odom"
        )
        
        self.tf_proc = subprocess.Popen(
            self._get_wsl_base_cmd() + [cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def start_core_slam_nodes(self):
        """Launch RViz2 and Cartographer nodes via WSL."""
        self.logger.info("Starting SLAM Cartographer nodes...")
        
        # Safely fetch commands from config, providing a fallback just in case
        carto_raw = self.core_commands.get("Cartographer", "ros2 launch my_robot_slam online_slam.launch.py")
        rviz_raw = self.core_commands.get("RViz2", "ros2 run rviz2 rviz2 -d ~/cartographer_ws/src/my_robot_slam/rviz/mapper.rviz")

        cartographer_cmd = self._wrap_ros_cmd(carto_raw)
        rviz_cmd = self._wrap_ros_cmd(rviz_raw)

        self.cartographer_proc = subprocess.Popen(
            self._get_wsl_base_cmd() + [cartographer_cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(2) # Give cartographer time to establish
        
        self.rviz_proc = subprocess.Popen(
            self._get_wsl_base_cmd() + [rviz_cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def toggle_rosbag(self, record: bool):
        """Starts or stops the rosbag recording."""
        if record and not self.rosbag_proc:
            self.logger.info("Starting rosbag recording...")
            bag_cmd = self._wrap_ros_cmd("ros2 bag record -a")
            self.rosbag_proc = subprocess.Popen(
                self._get_wsl_base_cmd() + [bag_cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            return True
        elif not record and self.rosbag_proc:
            self.logger.info("Stopping rosbag...")
            # Safely SIGINT the recording inside WSL
            kill_cmd = self._get_wsl_base_cmd() + ["pkill -SIGINT -f 'ros2 bag record'"]
            subprocess.run(kill_cmd)
            self.rosbag_proc.wait(timeout=5)
            self.rosbag_proc = None
            return False

    def stop_all_local(self):
        self.logger.info("Stopping local WSL processes...")
        if self.rosbag_proc:
            self.toggle_rosbag(False)
        if self.rviz_proc:
            self.rviz_proc.terminate()
        if self.cartographer_proc:
            self.cartographer_proc.terminate()
        if self.tf_proc:
            self.tf_proc.terminate()