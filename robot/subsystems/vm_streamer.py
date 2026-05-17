"""
vm_streamer.py - VM telemetry server over TCP.
Reads voltage and current from the INA226 hardware and streams it
to the base station dashboard.
"""

import time
import socket
from core.config_loader import CONFIG
from core.network_utils import pack_vm_data
from robot.hardware.ina226 import INA226

def run_vm_streamer():
    """Continuously reads INA226 and sends packed binary data over a TCP socket."""
    ina_cfg = CONFIG["hardware"]["ina226"]
    ina = INA226(
        bus_num=ina_cfg["bus"], 
        address=ina_cfg["address"], 
        shunt_ohms=ina_cfg["shunt_ohms"]
    )
    
    try:
        ina.initialize()
        print("VM: INA226 initialized successfully.")
    except Exception as e:
        print(f"VM: INA226 initialization failed: {e}. Hardware missing?")

    port = CONFIG["network"]["vm_port"]
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(1)
    print(f"VM: Listening on 0.0.0.0:{port}")

    try:
        while True:
            client, addr = server.accept()
            print(f"VM: Client connected from {addr}")
            try:
                while True:
                    v = ina.get_voltage()
                    i = ina.get_current()
                    
                    # Use the shared packing function from Phase 1
                    data = pack_vm_data(v, i)
                    client.sendall(data)
                    time.sleep(1.0)  # Stream at 1Hz
            except (ConnectionResetError, BrokenPipeError):
                print("VM: Client disconnected.")
            finally:
                client.close()
    except KeyboardInterrupt:
        print("VM: Stopping.")
    finally:
        server.close()
        ina.close()