"""
rpi_main.py - Raspberry Pi Central Process Manager.
Acts purely as a supervisor. Spawns subsystem workers (VMStreamer, MotorEngine, 
CameraServer, HeadingTracker) on startup. Monitors health with auto-restart 
capabilities and manages on-demand processes like the LIDAR node.
"""

import time
import sys
import logging
import multiprocessing
import queue

# Import centralized configuration
from core.config_loader import CONFIG

# Import isolated subsystems
from robot.subsystems.vm_streamer import run_vm_streamer
from robot.subsystems.motor_server import run_motor_engine
from robot.subsystems.camera_server import run_server as run_camera_server
from robot.subsystems.heading_tracker import run_heading_tracker
from robot.subsystems.lidar_node import run_lidar_process

# ========================== Configuration ==========================
MAX_RESTARTS = 3

class ColoredFormatter(logging.Formatter):
    """Custom formatter to add colors to console logs for the RPi."""
    COLORS = {
        'INFO': "\033[96m",     # Cyan (Distinct from PC Green)
        'WARNING': "\033[93m",  # Yellow
        'ERROR': "\033[91m",    # Red
    }
    RESET = "\033[0m"

    def format(self, record):
        color = self.COLORS.get(record.levelname, self.RESET)
        record.levelname = f"{color}{record.levelname}{self.RESET}"
        return super().format(record)

# Setup Logging
logger = logging.getLogger("RPI_MAIN")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(ColoredFormatter('%(asctime)s - [%(levelname)s] - %(message)s', datefmt='%H:%M:%S'))
logger.addHandler(handler)

# Global process tracker dictionaries
procs = {}
restart_counts = {}

def start_process(name, target, args=()):
    """Helper to spawn, register, and track a multiprocessing worker."""
    p = multiprocessing.Process(target=target, args=args, daemon=True)
    p.start()
    procs[name] = (p, target, args)
    logger.info(f"Started {name} (PID {p.pid})")

def main():
    logger.info("Initializing RoboKedach RPi Supervisor...")

    # 1. Initialize Multiprocessing Queues for Inter-Process Communication
    trigger_queue = multiprocessing.Queue()
    heading_cmd_queue = multiprocessing.Queue()
    heading_res_queue = multiprocessing.Queue()

    # 2. Start default startup processes
    # Pass the trigger_queue into the motor server so it can send commands back up!
    start_process("VMStreamer", run_vm_streamer, ())
    start_process("MotorEngine", run_motor_engine, (trigger_queue, heading_res_queue))
    start_process("CameraServer", run_camera_server, ())

    try:
        while True:
            # --- NEW COMMAND ROUTER ---
            # Process any commands forwarded by the Motor Server
            while not trigger_queue.empty():
                try:
                    cmd = trigger_queue.get_nowait()
                except queue.Empty:
                    break
                    
                if cmd == "START_SLAM":
                    if "LIDARNode" not in procs or not procs["LIDARNode"][0].is_alive():
                        logger.info("MAIN: START_SLAM received. Launching LIDAR Node...")
                        start_process("LIDARNode", run_lidar_process, ())
                    else:
                        logger.warning("MAIN: LIDAR Node is already running.")

                elif cmd == "START_HEADING_TRACK":
                    if "HeadingTracker" not in procs or not procs["HeadingTracker"][0].is_alive():
                        logger.info("MAIN: Launching Heading Tracker...")
                        start_process("HeadingTracker", run_heading_tracker, (heading_cmd_queue, heading_res_queue))
                    logger.info("MAIN: Triggering Phase 1 Calibration.")
                    heading_cmd_queue.put("CALIBRATE")

                elif cmd == "HEADING_LANDED":
                    logger.info("MAIN: HEADING_LANDED received. Triggering Phase 3.")
                    heading_cmd_queue.put("LANDED")
            # --------------------------

            # --- PROCESS HEALTH MONITOR ---
            for name, (proc, target, args) in list(procs.items()):
                # Ignore stateful/on-demand processes.
                if not proc.is_alive() and name not in ["HeadingTracker", "LIDARNode"]:
                    count = restart_counts.get(name, 0)
                    
                    if count < MAX_RESTARTS:
                        restart_counts[name] = count + 1
                        logger.warning(f"{name} died (exit code {proc.exitcode}). "
                                       f"Restarting ({count + 1}/{MAX_RESTARTS})...")
                        start_process(name, target, args)
                        
                    elif count == MAX_RESTARTS:
                        # Increment once more to prevent spamming the console
                        restart_counts[name] = count + 1
                        logger.error(f"{name} failed {MAX_RESTARTS} times. "
                                     f"Giving up — check hardware.")
                        
            time.sleep(0.1) # Higher tick rate for better responsiveness

    except KeyboardInterrupt:
        logger.info("Shutting down...")
        
        # Send kill signals
        for name, (proc, _, _) in procs.items():
            logger.info(f"Stopping {name}...")
            proc.terminate()
            
        # Wait for graceful exit
        for _, (proc, _, _) in procs.items():
            proc.join(timeout=3)
            
        logger.info("All processes stopped. Goodbye!")

if __name__ == '__main__':
    main()