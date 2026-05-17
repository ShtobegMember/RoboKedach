"""
vm_client.py - TCP Telemetry Receiver.
Listens for the packed binary struct (Voltage, Current) from the Pi's INA226.
"""

import socket
from PyQt6.QtCore import QThread, pyqtSignal
from core.config_loader import CONFIG
from core.network_utils import unpack_vm_data, VM_DATA_SIZE

class VMServerWorker(QThread):
    # Emits (Voltage, Current) as floats
    data_received = pyqtSignal(float, float)
    connection_status = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.is_running = True
        self.rpi_ip = CONFIG["network"]["rpi_ip"]
        self.port = CONFIG["network"]["vm_port"]

    def run(self):
        while self.is_running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2.0)
                    s.connect((self.rpi_ip, self.port))
                    self.connection_status.emit(True)
                    s.settimeout(None) # Block normally once connected
                    
                    while self.is_running:
                        data = s.recv(VM_DATA_SIZE)
                        if not data or len(data) != VM_DATA_SIZE:
                            break
                        
                        voltage, current = unpack_vm_data(data)
                        self.data_received.emit(voltage, current)
                        
            except (socket.timeout, ConnectionRefusedError, socket.error):
                self.connection_status.emit(False)
                if self.is_running:
                    self.msleep(1000)

    def stop(self):
        self.is_running = False