import socket
import struct
import queue
import select # Added for non-blocking read
from PyQt6.QtCore import QThread, pyqtSignal
from core.config_loader import CONFIG

class MotorCommandWorker(QThread):
    connection_status = pyqtSignal(bool)
    
    # --- ADD MISSING SIGNALS ---
    speed_update = pyqtSignal(int, int)
    encoder_update = pyqtSignal(int, int)
    motor_ready = pyqtSignal()
    heading_calibrated = pyqtSignal()
    heading_received = pyqtSignal(float)
    # ---------------------------

    def __init__(self):
        super().__init__()
        self.is_running = True
        self.rpi_ip = CONFIG["network"]["rpi_ip"]
        self.port = CONFIG["network"]["motor_port"]
        self.command_queue = queue.Queue()
    
    def send_movement(self, fraction1: float, fraction2: float):
        # Format as string and append newline delimiter
        cmd_str = f"MOVE:{fraction1},{fraction2}\n"
        self.command_queue.put(cmd_str.encode('utf-8'))

    def send_trigger(self, trigger_str: str):
        # Append newline delimiter
        cmd_str = f"{trigger_str}\n"
        self.command_queue.put(cmd_str.encode('utf-8'))

    def run(self):
        while self.is_running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(2.0)
                    s.connect((self.rpi_ip, self.port))
                    self.connection_status.emit(True)
                    s.setblocking(False) # Change to non-blocking
                    
                    while self.is_running:
                        # 1. Send outgoing commands if any exist
                        try:
                            while not self.command_queue.empty():
                                cmd_data = self.command_queue.get_nowait()
                                s.sendall(cmd_data)
                        except queue.Empty:
                            pass

                        # 2. Check for incoming telemetry/triggers
                        ready_to_read, _, _ = select.select([s], [], [], 0.05)
                        if ready_to_read:
                            data = s.recv(1024)
                            if not data:
                                break # Connection closed
                            
                            self._parse_incoming_data(data)
                            
            except (socket.timeout, ConnectionRefusedError, socket.error):
                self.connection_status.emit(False)
                if self.is_running:
                    self.msleep(1000)

    def _parse_incoming_data(self, data: bytes):
        """Parse data sent back from the RPi and emit corresponding signals."""
        try:
            msg = data.decode('utf-8').strip()
            if msg == "HEADING_CALIBRATED":
                self.heading_calibrated.emit()
            elif msg.startswith("HEADING_RES:"):
                # Format: "HEADING_RES:125.4"
                val = float(msg.split(":")[1])
                self.heading_received.emit(val)
            elif msg.startswith("ENC:"):
                # Format: "ENC:4200,8400"
                parts = msg.split(":")[1].split(",")
                self.encoder_update.emit(int(parts[0]), int(parts[1]))
            elif msg.startswith("SPD:"):
                # Format: "SPD:64,64"
                parts = msg.split(":")[1].split(",")
                self.speed_update.emit(int(parts[0]), int(parts[1]))
            elif msg == "MOTOR_READY":
                self.motor_ready.emit()
        except Exception:
            pass # Ignore malformed packets

    def stop(self):
        self.is_running = False