"""
motor_server.py - TCP server for remote motor commands.
Receives movement fraction pairs (or high-level signals) from the base station 
and forwards them to the logical movement controller.
Streams encoder, speed, and heading data back to the base station.
"""

import socket
import struct
import multiprocessing
import select
import time
import queue
import threading

from core.config_loader import CONFIG
from robot.control.movement_controller import RobotConfig, MotorController, MovementController

def run_motor_engine(command_queue: multiprocessing.Queue = None, heading_res_queue: multiprocessing.Queue = None):
    """
    Listens for remote motor commands and executes them.
    Passes string triggers up to the main process via the command_queue.
    Streams telemetry back to the connected client.
    """
    
    # Initialize hardware config for the RoboClaw
    rc_cfg = CONFIG["hardware"]["roboclaw"]
    robot_cfg = RobotConfig(
        port=rc_cfg["port"],
        baud_rate=rc_cfg["baud_rate"],
        address=rc_cfg["address"]
    )
    
    try:
        motor_ctrl = MotorController(robot_cfg)
        movement_ctrl = MovementController(motor_ctrl)
    except Exception as e:
        print(f"MOTOR_SRV: Failed to initialize controllers: {e}")
        return

    port = CONFIG["network"]["motor_port"]
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(1)
    print(f"MOTOR_SRV: Listening on 0.0.0.0:{port}")

    def telemetry_streamer(client_socket, is_connected_event):
        """Background thread to stream encoders, speeds, and heading status back to the PC."""
        last_enc_time = time.time()
        
        while is_connected_event.is_set():
            try:
                # 1. Send Encoder and Speed Data at ~10Hz
                if time.time() - last_enc_time > 0.1:
                    try:
                        # Fetch values from the roboclaw. 
                        # Update these method names if your motor_ctrl uses different ones!
                        enc1, enc2 = motor_ctrl.read_encoders()
                        spd1, spd2 = motor_ctrl.read_speeds()
                        
                        client_socket.sendall(f"ENC:{enc1},{enc2}\n".encode('utf-8'))
                        client_socket.sendall(f"SPD:{spd1},{spd2}\n".encode('utf-8'))
                    except Exception:
                        pass # Ignore temporary hardware read errors
                        
                    last_enc_time = time.time()

                # 2. Check for Heading Updates from the IMU Tracker
                if heading_res_queue:
                    try:
                        res = heading_res_queue.get_nowait()
                        if res[0] == 'STATUS' and res[1] == 'HEADING_CALIBRATED':
                            client_socket.sendall(b"HEADING_CALIBRATED\n")
                        elif res[0] == 'HEADING_RES':
                            client_socket.sendall(f"HEADING_RES:{res[1]}\n".encode('utf-8'))
                    except queue.Empty:
                        pass
                
                time.sleep(0.01)
                
            except (ConnectionResetError, BrokenPipeError, socket.error):
                break # Client disconnected, exit thread

    try:
        while True:
            client, addr = server.accept()
            print(f"MOTOR_SRV: Connection from {addr}")
            
            # Announce readiness to the PC
            client.sendall(b"MOTOR_READY\n")
            
            is_connected = threading.Event()
            is_connected.set()
            
            # Start the background telemetry streaming thread
            telemetry_thread = threading.Thread(target=telemetry_streamer, args=(client, is_connected), daemon=True)
            telemetry_thread.start()
            
            try:
                while True:
                    # Use select to make recv non-blocking
                    ready_to_read, _, _ = select.select([client], [], [], 0.1)
                    if ready_to_read:
                        data = client.recv(1024)
                        if not data:
                            break
                        # Split by newline to prevent TCP concatenation bugs
                        messages = data.decode('utf-8', errors='ignore').split('\n')
                        
                        for msg in messages:
                            msg = msg.strip()
                            if not msg:
                                continue
                            
                            # Route Movement Commands
                            if msg.startswith("MOVE:"):
                                try:
                                    parts = msg.split(":")[1].split(",")
                                    # hud_window sends: MOVE:<left_fraction>,<right_fraction>
                                    f_left, f_right = float(parts[0]), float(parts[1])
                                    
                                    # Use the new non-blocking method!
                                    movement_ctrl.set_speeds(f_left, f_right)
                                    
                                except Exception as e:
                                    print(f"MOTOR_SRV: Failed to parse move '{msg}': {e}")
                                    
                            # Route System Triggers
                            else:
                                valid_triggers = ['START_SLAM', 'START_HEADING_TRACK', 'HEADING_LANDED']
                                if msg in valid_triggers:
                                    print(f"MOTOR_SRV: Trigger received: {msg}")
                                    if command_queue:
                                        command_queue.put(msg)
                                    
            except (ConnectionResetError, BrokenPipeError):
                print("MOTOR_SRV: Client disconnected.")
            finally:
                # Cleanup the client session
                is_connected.clear()
                telemetry_thread.join(timeout=1.0)
                client.close()
                motor_ctrl.stop_all()
                
    except KeyboardInterrupt:
        print("MOTOR_SRV: Stopping.")
    finally:
        server.close()
        try:
            motor_ctrl.stop_all()
        except:
            pass