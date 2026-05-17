"""
camera_client.py - MJPEG HTTP Stream Receiver.
Runs on a background QThread so network latency doesn't freeze the GUI.
Emits a pyqtSignal with a QImage whenever a new frame is decoded.
"""

import cv2
import numpy as np
import urllib.request
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QImage
from core.config_loader import CONFIG

class MJPEGStreamWorker(QThread):
    # Emits a new QImage when a frame is successfully fetched and decoded
    frame_ready = pyqtSignal(QImage)
    # Emits a boolean indicating stream connection status
    connection_status = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.is_running = True
        rpi_ip = CONFIG["network"]["rpi_ip"]
        port = CONFIG["network"]["camera_port"]
        self.stream_url = f"http://{rpi_ip}:{port}/"

    def run(self):
        bytes_data = bytes()
        while self.is_running:
            try:
                # Open stream with a timeout to prevent hanging forever
                stream = urllib.request.urlopen(self.stream_url, timeout=3.0)
                self.connection_status.emit(True)
                
                while self.is_running:
                    chunk = stream.read(1024 * 4) # Read in 4KB chunks
                    if not chunk:
                        break
                        
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8') # JPEG start
                    b = bytes_data.find(b'\xff\xd9') # JPEG end
                    
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        # Decode image
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            # Convert BGR to RGB for Qt
                            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            h, w, ch = rgb_image.shape
                            bytes_per_line = ch * w
                            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                            self.frame_ready.emit(qt_image)
                            
            except Exception as e:
                self.connection_status.emit(False)
                if self.is_running:
                    self.msleep(1000) # Wait before trying to reconnect

    def stop(self):
        self.is_running = False