"""
pc_main.py - Base Station Entry Point.
Initializes the PyQt6 application, instantiates the network workers and process 
managers, wires their data signals to the HUDWindow, and manages clean shutdowns.
"""

import sys
import logging
from PyQt6.QtWidgets import QApplication

# Import core configurations and subsystems
from core.config_loader import CONFIG
from core.network_utils import setup_windows_network, is_windows_network_ready
from base_station.network.camera_client import MJPEGStreamWorker
from base_station.network.vm_client import VMServerWorker
from base_station.network.motor_client import MotorCommandWorker
from base_station.process_management import wsl_manager, rpi_deployer
from base_station.gui.hud_window import HUDWindow

class ColoredFormatter(logging.Formatter):
    """Custom formatter to add colors to console logs based on level."""
    COLORS = {
        'DEBUG': "\033[94m",    # Blue
        'INFO': "\033[92m",     # Green
        'WARNING': "\033[93m",  # Yellow
        'ERROR': "\033[91m",    # Red
        'CRITICAL': "\033[1;91m" # Bold Red
    }
    RESET = "\033[0m"

    def format(self, record):
        color = self.COLORS.get(record.levelname, self.RESET)
        record.levelname = f"{color}{record.levelname}{self.RESET}"
        return super().format(record)

def main():
    # Initialize professional colored logging
    handler = logging.StreamHandler(sys.stdout)
    fmt = ColoredFormatter(
        fmt='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    handler.setFormatter(fmt)
    
    logging.basicConfig(level=logging.INFO, handlers=[handler])
    logger = logging.getLogger("PC_MAIN")

    logger.info("Initializing Base Station...")
    app = QApplication(sys.argv)

    # 0. Configure windows connection
    pc_network_interface_name = CONFIG["network"]["pc_network_interface_name"]
    pc_network_name = CONFIG["network"]["pc_network_name"]
    pc_ip = CONFIG["network"]["pc_ip"]
    pc_subnet_mask = CONFIG["network"]["pc_subnet_mask"]
    if not is_windows_network_ready(pc_network_interface_name, pc_ip):
        setup_windows_network(pc_network_interface_name, pc_network_name, pc_ip, pc_subnet_mask)
    else:
        logger.info(f"Windows network interface '{pc_network_interface_name}' is already correctly configured.")


    # 1. Deploy and Start Robot Code
    # This checks for local code changes, pushes them over SFTP to the Pi, 
    # and restarts the Pi's main script before the GUI even loads.
    logger.info("Connecting to RPi for deployment...")
    rpi_deploy_mgr = rpi_deployer.RPiDeployer()
    
    # Shared connection to fix 'AssertionError: connect called twice'
    ssh_conn = rpi_deploy_mgr._connect()
    rpi_deploy_mgr.sync_code(ssh=ssh_conn)
    rpi_deploy_mgr.start_rpi_main(ssh=ssh_conn)
    # Note: ssh_conn is left open for the background log streamer thread

    # 2. Initialize Subsystems
    wsl_mgr = wsl_manager.WSLManager()
    
    cam_worker = MJPEGStreamWorker()
    vm_worker = VMServerWorker()
    motor_worker = MotorCommandWorker()

    # 3. Initialize the GUI Window
    # We pass the managers into the window so button clicks and keystrokes 
    # can trigger network commands and local processes.
    window = HUDWindow(wsl_manager=wsl_mgr, motor_client=motor_worker)

    # 4. Connect Worker Signals to GUI Slots
    # Camera Data
    cam_worker.frame_ready.connect(window.update_frame)
    cam_worker.connection_status.connect(
        lambda status: window.update_connection_status("CAM", status)
    )
    
    # Telemetry Data
    vm_worker.data_received.connect(window.update_telemetry)
    vm_worker.connection_status.connect(
        lambda status: window.update_connection_status("SYS", status)
    )
    
    # Motor Status
    motor_worker.connection_status.connect(
        lambda status: window.update_connection_status("MOT", status)
    )

    motor_worker.heading_calibrated.connect(window.on_heading_calibrated)
    motor_worker.heading_received.connect(window.on_heading_received)

    # 5. Start Background Threads
    logger.info("Starting background worker threads...")
    cam_worker.start()
    vm_worker.start()
    motor_worker.start()

    # 6. Define Graceful Teardown Routine
    def cleanup():
        logger.info("Initiating graceful shutdown sequence...")
        
        # Signal threads to stop
        cam_worker.stop()
        vm_worker.stop()
        motor_worker.stop()
        
        # Wait for threads to join safely
        logger.debug("Waiting for network workers to terminate...")
        cam_worker.wait(2000)
        vm_worker.wait(2000)
        motor_worker.wait(2000)
        
        # Stop any active ROS2/WSL processes (like RViz or bag recordings)
        logger.info("Stopping local WSL processes...")
        wsl_mgr.stop_all_local()
        
        # Trigger an SSH shutdown to the Pi to ensure motors stop safely
        logger.info("Sending remote kill command to RPi...")
        wsl_mgr.remote_kill_rpi()
        
        logger.info("Shutdown complete. Goodbye!")

    # Bind the cleanup function to the application quit event
    app.aboutToQuit.connect(cleanup)

    # 7. Display the GUI and start the Qt Event Loop
    logger.info("System Ready. Launching HUD window.")
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()