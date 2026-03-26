"""
pc_main.py - Base Station HUD Dashboard.
Displays live camera feed as background, IMU telemetry & voltage/current overlay,
and captures arrow keys for remote motor control.
"""

import sys
import socket
import struct
import time
import math
import threading
import urllib.request
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QPixmap, QImage, QPainter, QFont, QColor, QPen
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel


# ========================== Configuration ==========================
RPI_IP = "192.168.1.2"
CAMERA_URL = f"http://{RPI_IP}:5000/"

IMU_PORT = 65432
MOTOR_PORT = 65433
VM_PORT = 65434


# ========================== MJPEG Stream Worker ==========================
class MJPEGStreamWorker(QThread):
    """Fetches MJPEG frames from the RPi camera HTTP server and emits QImages."""

    frame_received = pyqtSignal(QImage)
    status_update = pyqtSignal(str)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self.is_running = True

    def run(self):
        while self.is_running:
            try:
                self.status_update.emit("Connecting to camera...")
                stream = urllib.request.urlopen(self.url, timeout=5)
                self.status_update.emit("Camera connected")

                buf = b""
                while self.is_running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break

                    buf += chunk

                    # Find JPEG boundaries (SOI: 0xFFD8, EOI: 0xFFD9)
                    soi = buf.find(b'\xff\xd8')
                    eoi = buf.find(b'\xff\xd9')

                    if soi != -1 and eoi != -1 and eoi > soi:
                        jpg = buf[soi:eoi + 2]
                        buf = buf[eoi + 2:]

                        image = QImage()
                        if image.loadFromData(jpg):
                            self.frame_received.emit(image)

            except Exception as e:
                self.status_update.emit(f"Camera error: {e}")
                if self.is_running:
                    time.sleep(2)

    def stop(self):
        self.is_running = False
        self.quit()


# ========================== IMU Server Worker ==========================
class IMUServerWorker(QThread):
    """TCP server that receives IMU data packets from the RPi."""

    data_received = pyqtSignal(float, float, float, float, float, float)
    status_update = pyqtSignal(str)

    def __init__(self, host="0.0.0.0", port=IMU_PORT):
        super().__init__()
        self.host = host
        self.port = port
        self.is_running = True

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind((self.host, self.port))
            server.listen()
            server.settimeout(1.0)
            self.status_update.emit(f"IMU: Listening on port {self.port}")

            while self.is_running:
                try:
                    conn, addr = server.accept()
                    self._handle_client(conn, addr)
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.is_running:
                        self.status_update.emit(f"IMU server error: {e}")
        finally:
            server.close()

    def _handle_client(self, conn, addr):
        self.status_update.emit(f"IMU: RPi connected from {addr[0]}")

        try:
            conn.sendall(b"START\n")

            while self.is_running:
                data = conn.recv(24)
                if not data:
                    break
                if len(data) == 24:
                    ax, ay, az, gx, gy, gz = struct.unpack('<6f', data)
                    self.data_received.emit(ax, ay, az, gx, gy, gz)

        except ConnectionResetError:
            self.status_update.emit("IMU: Connection lost")
        except Exception as e:
            self.status_update.emit(f"IMU error: {e}")
        finally:
            conn.close()
            self.status_update.emit("IMU: Waiting for reconnection...")

    def stop(self):
        self.is_running = False
        self.quit()


# ========================== VM Server Worker ==========================
class VMServerWorker(QThread):
    """TCP server that receives INA226 voltage/current from the RPi."""

    data_received = pyqtSignal(float, float)   # voltage, current
    status_update = pyqtSignal(str)

    def __init__(self, host="0.0.0.0", port=VM_PORT):
        super().__init__()
        self.host = host
        self.port = port
        self.is_running = True

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind((self.host, self.port))
            server.listen()
            server.settimeout(1.0)
            self.status_update.emit(f"VM: Listening on port {self.port}")

            while self.is_running:
                try:
                    conn, addr = server.accept()
                    self._handle_client(conn, addr)
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.is_running:
                        self.status_update.emit(f"VM server error: {e}")
        finally:
            server.close()

    def _handle_client(self, conn, addr):
        self.status_update.emit(f"VM: RPi connected from {addr[0]}")

        try:
            while self.is_running:
                data = conn.recv(8)  # 2 floats = 8 bytes
                if not data:
                    break
                if len(data) == 8:
                    voltage, current = struct.unpack('<2f', data)
                    self.data_received.emit(voltage, current)

        except ConnectionResetError:
            self.status_update.emit("VM: Connection lost")
        except Exception as e:
            self.status_update.emit(f"VM error: {e}")
        finally:
            conn.close()
            self.status_update.emit("VM: Waiting for reconnection...")

    def stop(self):
        self.is_running = False
        self.quit()


# ========================== Motor Command Worker ==========================
class MotorCommandWorker(QThread):
    """TCP client that sends motor commands to and receives status from the RPi."""

    status_update = pyqtSignal(str)
    speed_update = pyqtSignal(int)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.is_running = True
        self._sock = None
        self._lock = threading.Lock()

    def run(self):
        while self.is_running:
            try:
                self.status_update.emit("Motor: Connecting...")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.host, self.port))
                sock.settimeout(1.0)

                with self._lock:
                    self._sock = sock

                self.status_update.emit("Motor: Connected")

                buf = ""
                while self.is_running:
                    try:
                        data = sock.recv(1024)
                        if not data:
                            break
                        buf += data.decode('utf-8')
                        while '\n' in buf:
                            line, buf = buf.split('\n', 1)
                            msg = line.strip()
                            if msg.startswith("SPEED:"):
                                self.speed_update.emit(int(msg[6:]))
                            elif msg == "BUSY":
                                self.status_update.emit("Motor: Moving...")
                            elif msg == "READY":
                                self.status_update.emit("Motor: Ready")
                            elif msg.startswith("ERROR:"):
                                self.status_update.emit(f"Motor: {msg}")
                    except socket.timeout:
                        continue

            except (ConnectionRefusedError, OSError):
                self.status_update.emit("Motor: Disconnected")
            except Exception as e:
                self.status_update.emit(f"Motor: {e}")

            with self._lock:
                self._sock = None
            if self.is_running:
                time.sleep(2)

    def send_command(self, cmd):
        """Thread-safe command send. Called from the GUI thread."""

        with self._lock:
            if self._sock:
                try:
                    self._sock.sendall(f"{cmd}\n".encode())
                except Exception:
                    pass

    def stop(self):
        self.is_running = False
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
        self.quit()


# ========================== HUD Overlay ==========================
class HUDOverlay(QLabel):
    """Transparent widget that paints IMU telemetry, motor status, and power data."""

    def __init__(self, parent):
        super().__init__(parent)
        self.hud = parent
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setStyleSheet("background: transparent;")

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        d = self.hud.imu_data

        # Colors
        panel_bg = QColor(0, 0, 0, 140)
        green = QColor(0, 255, 0, 220)
        white = QColor(255, 255, 255, 240)
        yellow = QColor(255, 220, 0, 220)
        border = QPen(green, 1)

        title_font = QFont("Consolas", 11, QFont.Weight.Bold)
        data_font = QFont("Consolas", 10)
        status_font = QFont("Consolas", 9)

        # --- Accelerometer Panel (top-left) ---
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(10, 10, 220, 130, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(22, 34, "ACCELEROMETER  m/s\u00b2")

        p.setFont(data_font)
        y = 60
        for label, key in [("X", "ax"), ("Y", "ay"), ("Z", "az")]:
            p.setPen(green)
            p.drawText(28, y, f"{label}:")
            p.setPen(white)
            p.drawText(55, y, f"{d[key]:+8.3f}")

            # Mini bar gauge
            bar_val = max(-10, min(10, d[key]))
            bar_x = 160
            bar_w = int((bar_val / 10) * 30)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(green)
            if bar_w >= 0:
                p.drawRect(bar_x, y - 10, bar_w, 8)
            else:
                p.drawRect(bar_x + bar_w, y - 10, -bar_w, 8)
            p.setBrush(panel_bg)

            y += 25

        # --- Gyroscope Panel (top-right) ---
        gx = w - 230
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(gx, 10, 220, 130, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(gx + 12, 34, "GYROSCOPE  deg/s")

        p.setFont(data_font)
        y = 60
        for label, key in [("X", "gx"), ("Y", "gy"), ("Z", "gz")]:
            deg_val = math.degrees(d[key])
            p.setPen(green)
            p.drawText(gx + 18, y, f"{label}:")
            p.setPen(white)
            p.drawText(gx + 45, y, f"{deg_val:+8.3f}")

            # Mini bar gauge
            bar_val = max(-125, min(125, deg_val))
            bar_bx = gx + 150
            bar_w = int((bar_val / 125) * 30)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(green)
            if bar_w >= 0:
                p.drawRect(bar_bx, y - 10, bar_w, 8)
            else:
                p.drawRect(bar_bx + bar_w, y - 10, -bar_w, 8)
            p.setBrush(panel_bg)

            y += 25

        # --- Power Monitor Panel (below accelerometer) ---
        pm_y = 150
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(10, pm_y, 220, 80, 8, 8)

        p.setFont(title_font)
        p.setPen(yellow)
        p.drawText(22, pm_y + 24, "POWER MONITOR")

        p.setFont(QFont("Consolas", 13, QFont.Weight.Bold))
        vm = self.hud.vm_data
        p.setPen(white)
        p.drawText(28, pm_y + 50, f"{vm['voltage']:6.2f} V")
        p.drawText(28, pm_y + 70, f"{vm['current']:6.3f} A")

        # Voltage color indicator
        v = vm['voltage']
        if v > 0:
            if v >= 11.1:
                v_color = green         # Healthy (3.7V+ per cell)
            elif v >= 10.2:
                v_color = yellow        # Low warning (3.4V per cell)
            else:
                v_color = QColor(255, 50, 50, 220)  # Critical (<3.4V per cell)

            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(v_color)
            p.drawRect(190, pm_y + 38, 12, 12)
            p.setBrush(panel_bg)

        # --- Status Bar (bottom) ---
        p.setBrush(QColor(0, 0, 0, 160))
        p.setPen(border)
        p.drawRoundedRect(10, h - 40, w - 20, 30, 5, 5)

        p.setFont(status_font)
        p.setPen(green)
        p.drawText(20, h - 20,
                   f"CAM: {self.hud.camera_status}  |  {self.hud.imu_status}"
                   f"  |  {self.hud.motor_status}  SPD: {self.hud.motor_speed}/127"
                   f"  |  {self.hud.vm_status}")

        p.end()


# ========================== Main HUD Window ==========================
class HUDWindow(QMainWindow):
    """Main window — camera feed background with IMU + motor + power HUD overlay."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboKedach HUD")
        self.resize(800, 600)
        self.setStyleSheet("background-color: black;")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # State
        self.imu_data = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
        self.vm_data = {'voltage': 0.0, 'current': 0.0}
        self.camera_status = "Disconnected"
        self.imu_status = "Disconnected"
        self.motor_status = "Disconnected"
        self.motor_speed = 0
        self.vm_status = "VM: Disconnected"

        # Camera background
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("background-color: black;")
        self.setCentralWidget(self.camera_label)

        # HUD overlay (paints on top of the camera)
        self.overlay = HUDOverlay(self)
        self.overlay.raise_()

        # Camera stream worker
        self.cam_worker = MJPEGStreamWorker(CAMERA_URL)
        self.cam_worker.frame_received.connect(self._on_frame)
        self.cam_worker.status_update.connect(self._on_cam_status)
        self.cam_worker.start()

        # IMU socket server worker
        self.imu_worker = IMUServerWorker()
        self.imu_worker.data_received.connect(self._on_imu_data)
        self.imu_worker.status_update.connect(self._on_imu_status)
        self.imu_worker.start()

        # Motor command client worker
        self.motor_worker = MotorCommandWorker(RPI_IP, MOTOR_PORT)
        self.motor_worker.status_update.connect(self._on_motor_status)
        self.motor_worker.speed_update.connect(self._on_motor_speed)
        self.motor_worker.start()

        # VM (voltage monitor) server worker
        self.vm_worker = VMServerWorker()
        self.vm_worker.data_received.connect(self._on_vm_data)
        self.vm_worker.status_update.connect(self._on_vm_status)
        self.vm_worker.start()

        # Periodic repaint for VM panel (ensures updates even when IMU/camera are idle)
        self.vm_timer = QTimer(self)
        self.vm_timer.timeout.connect(self.overlay.update)
        self.vm_timer.start(500)

    # --- Key Input ---
    def keyPressEvent(self, event):
        """Capture arrow keys and send motor commands to the RPi."""
        
        if event.isAutoRepeat():
            return

        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        key = event.key()

        cmd = None
        if key == Qt.Key.Key_Up:
            cmd = "FWD90" if shift else "FWD"
        # elif key == Qt.Key.Key_Down:
        #     cmd = "BWD90" if shift else "BWD"
        elif key == Qt.Key.Key_Left:
            cmd = "LEFT90" if shift else "LEFT"
        elif key == Qt.Key.Key_Right:
            cmd = "RIGHT90" if shift else "RIGHT"
        elif key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            cmd = "SPEED_UP"
        elif key == Qt.Key.Key_Minus:
            cmd = "SPEED_DOWN"
        elif key == Qt.Key.Key_Space:
            cmd = "ABORT"
        elif key == Qt.Key.Key_R:
            cmd = "RESET_ENC"

        if cmd:
            self.motor_worker.send_command(cmd)
        else:
            super().keyPressEvent(event)

    # --- Slots ---
    def _on_frame(self, image):
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled)

    def _on_cam_status(self, msg):
        self.camera_status = msg
        self.overlay.update()

    def _on_imu_data(self, ax, ay, az, gx, gy, gz):
        self.imu_data = {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz}
        self.overlay.update()

    def _on_imu_status(self, msg):
        self.imu_status = msg
        self.overlay.update()

    def _on_motor_status(self, msg):
        self.motor_status = msg
        self.overlay.update()

    def _on_motor_speed(self, speed):
        self.motor_speed = speed
        self.overlay.update()

    def _on_vm_data(self, voltage, current):
        self.vm_data = {'voltage': voltage, 'current': current}
        self.overlay.update()

    def _on_vm_status(self, msg):
        self.vm_status = msg
        self.overlay.update()

    # --- Events ---
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay.setGeometry(0, 0, self.width(), self.height())

    def closeEvent(self, event):
        self.vm_timer.stop()
        self.cam_worker.stop()
        self.imu_worker.stop()
        self.motor_worker.stop()
        self.vm_worker.stop()
        self.cam_worker.wait(3000)
        self.imu_worker.wait(3000)
        self.motor_worker.wait(3000)
        self.vm_worker.wait(3000)
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HUDWindow()
    window.show()
    sys.exit(app.exec())
