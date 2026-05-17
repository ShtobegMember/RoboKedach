#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "Scaffolding the new RoboKedach project structure..."

# 1. Create all directories
mkdir -p robokedach_project/config
mkdir -p robokedach_project/core
mkdir -p robokedach_project/base_station/gui
mkdir -p robokedach_project/base_station/network
mkdir -p robokedach_project/base_station/process_management
mkdir -p robokedach_project/robot/hardware
mkdir -p robokedach_project/robot/subsystems
mkdir -p robokedach_project/robot/control

# 2. Create Root files
touch robokedach_project/requirements.txt
touch robokedach_project/README.md

# 3. Create Config files
touch robokedach_project/config/config.json

# 4. Create Core files
touch robokedach_project/core/__init__.py
touch robokedach_project/core/config_loader.py
touch robokedach_project/core/network_utils.py

# 5. Create Base Station (PC) files
touch robokedach_project/base_station/__init__.py
touch robokedach_project/base_station/pc_main.py

# base_station/gui
touch robokedach_project/base_station/gui/__init__.py
touch robokedach_project/base_station/gui/hud_window.py
touch robokedach_project/base_station/gui/hud_painter.py

# base_station/network
touch robokedach_project/base_station/network/__init__.py
touch robokedach_project/base_station/network/camera_client.py
touch robokedach_project/base_station/network/vm_client.py
touch robokedach_project/base_station/network/motor_client.py

# base_station/process_management
touch robokedach_project/base_station/process_management/__init__.py
touch robokedach_project/base_station/process_management/wsl_manager.py

# 6. Create Robot (RPi) files
touch robokedach_project/robot/__init__.py
touch robokedach_project/robot/rpi_main.py

# robot/hardware
touch robokedach_project/robot/hardware/__init__.py
touch robokedach_project/robot/hardware/roboclaw.py
touch robokedach_project/robot/hardware/ina226.py
touch robokedach_project/robot/hardware/lsm6dsv16x.py

# robot/subsystems
touch robokedach_project/robot/subsystems/__init__.py
touch robokedach_project/robot/subsystems/motor_server.py
touch robokedach_project/robot/subsystems/vm_streamer.py
touch robokedach_project/robot/subsystems/camera_server.py
touch robokedach_project/robot/subsystems/heading_tracker.py
touch robokedach_project/robot/subsystems/lidar_node.py

# robot/control
touch robokedach_project/robot/control/__init__.py
touch robokedach_project/robot/control/movement_controller.py

echo "Project structure created successfully!"

# Display the resulting tree if the 'tree' command is installed
if command -v tree &> /dev/null; then
    tree robokedach_project
else
    echo "Done! (Install 'tree' to see the visual output in your terminal)."
fi