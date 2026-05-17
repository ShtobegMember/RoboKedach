"""
pc_main.py - Base Station Entry Point.
Initializes the PyQt6 application, instantiates the network workers and process 
managers, wires their data signals to the HUDWindow, and manages clean shutdowns.
"""

import sys
import logging
import traceback
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

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

def handle_exception(exc_type, exc_value, exc_traceback):
    """Global hook to catch unhandled exceptions and log them before the app dies."""
    logger = logging.getLogger("FATAL")
    logger.critical("Unhandled Exception detected:", exc_info=(exc_type, exc_value, exc_traceback))
    # Continue with standard sys.excepthook
    sys.__excepthook__(exc_type, exc_value, exc_traceback)

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
    
    # Install the exception hook
    sys.excepthook = handle_exception

    try:
        logger.info("Step 1: Initializing QApplication...")
        app = QApplication(sys.argv)
        # Force the app to stay alive even if windows are hidden momentarily
        app.setQuitOnLastWindowClosed(True)

        # 0. Configure windows connection
        pc_network_interface_name = CONFIG["network"]["pc_network_interface_name"]
        pc_network_name = CONFIG["network"]["pc_network_name"]
        pc_ip = CONFIG["network"]["pc_ip"]
        pc_subnet_mask = CONFIG["network"]["pc_subnet_mask"]
        
        logger.info(f"Checking network interface: {pc_network_interface_name}")
        if not is_windows_network_ready(pc_network_interface_name, pc_ip):
            logger.warning("Network not ready. Attempting configuration (may require Admin)...")
            setup_windows_network(pc_network_interface_name, pc_network_name, pc_ip, pc_subnet_mask)
            logger.info("Network configuration command sent.")
        else:
            logger.info(f"Windows network interface '{pc_network_interface_name}' is already correctly configured.")


        # 1. Deploy and Start Robot Code
        logger.info("Step 2: Deploying code to RPi...")
        # This checks for local code changes, pushes them over SFTP to the Pi, 
        # and restarts the Pi's main script before the GUI even loads.
        logger.info("Connecting to RPi for deployment...")
        rpi_deploy_mgr = rpi_deployer.RPiDeployer()
        
        # Shared connection to fix 'AssertionError: connect called twice'
        logger.info(f"Establishing SSH connection to {rpi_deploy_mgr.host}...")
        ssh_conn = rpi_deploy_mgr._connect()
        rpi_deploy_mgr.sync_code(ssh=ssh_conn)
        logger.info("Starting remote process...")
        rpi_deploy_mgr.start_rpi_main(ssh=ssh_conn)
        # Note: ssh_conn is left open for the background log streamer thread

        # 2. Initialize Subsystems
        logger.info("Step 3: Initializing Subsystems and Workers...")
        wsl_mgr = wsl_manager.WSLManager()
        
        cam_worker = MJPEGStreamWorker()
        vm_worker = VMServerWorker()
        motor_worker = MotorCommandWorker()

        # 3. Initialize the GUI Window
        logger.info("Step 4: Creating HUD Window...")
        # We pass the managers into the window so button clicks and keystrokes 
        # can trigger network commands and local processes.
        window = HUDWindow(wsl_manager=wsl_mgr, motor_client=motor_worker)

        # 4. Connect Worker Signals to GUI Slots
        # Camera Data
        # Using QueuedConnection to prevent 0xc0000005 Access Violations in Qt6Gui.dll
        cam_worker.frame_ready.connect(window.update_frame, Qt.ConnectionType.QueuedConnection)
        cam_worker.connection_status.connect(
            lambda status: window.update_connection_status("CAM", status), Qt.ConnectionType.QueuedConnection
        )
        
        # Telemetry Data
        vm_worker.data_received.connect(window.update_telemetry, Qt.ConnectionType.QueuedConnection)
        vm_worker.connection_status.connect(
            lambda status: window.update_connection_status("SYS", status), Qt.ConnectionType.QueuedConnection
        )
        
        # Motor Status
        motor_worker.connection_status.connect(
            lambda status: window.update_connection_status("MOT", status), Qt.ConnectionType.QueuedConnection
        )

        motor_worker.heading_calibrated.connect(window.on_heading_calibrated, Qt.ConnectionType.QueuedConnection)
        motor_worker.heading_received.connect(window.on_heading_received, Qt.ConnectionType.QueuedConnection)

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
        
        # Allow the OS to process the 'show' event before starting threads
        QApplication.processEvents()

        # 5. Start Background Threads AFTER window is shown
        logger.info("Starting background worker threads...")
        cam_worker.start()
        vm_worker.start()
        motor_worker.start()

        logger.info("Step 5: Entering Main Event Loop (app.exec)...")
        # Using a try block here to catch if something calls sys.exit() 
        # which raises a SystemExit exception.
        try:
            exit_code = app.exec()
            logger.info(f"Event loop finished normally. Exit code: {exit_code}")
        except SystemExit as se:
            logger.warning(f"SystemExit detected! Code: {se.code}")
            exit_code = se.code

    except Exception as e:
        logger.critical(f"FATAL: Application failed during initialization: {e}")
        traceback.print_exc()
        exit_code = 1
    
    logger.info(f"Process finalizing with exit code: {exit_code}")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()