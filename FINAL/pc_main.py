"""
pc_main2.py - Base Station HUD Dashboard with SLAM lifecycle management.
Displays live camera feed as background, captures arrow keys for remote
motor control, and manages WSL/ROS2 SLAM pipeline (RViz2 + Cartographer).
"""

import sys
import socket
import struct
import subprocess
import time
import threading
import urllib.request
from PyQt6.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt6.QtGui import QPixmap, QImage, QPainter, QFont, QColor, QPen
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton


# ========================== Configuration ==========================
RPI_IP = "192.168.1.2"
CAMERA_URL = f"http://{RPI_IP}:5000/"

MOTOR_PORT = 65433
VM_PORT = 65434

WSL_DISTRO = "Ubuntu-24.04"
WSL_PATH = "~/cartographer_ws"

# Pin Cyclone DDS to the fiber interface only, unicast data to prevent network flood
PC_FIBER_IP = "192.168.1.1"  # PC's fiber adapter IP — verify with ipconfig
CYCLONEDDS_CFG = (
    '<CycloneDDS><Domain><General>'
    f'<NetworkInterfaceAddress>{PC_FIBER_IP}</NetworkInterfaceAddress>'
    '<AllowMulticast>spdp</AllowMulticast>'
    '</General></Domain></CycloneDDS>'
)

WSL_ROS_PREAMBLE = (
    "source /opt/ros/kilted/setup.bash && "
    "source ~/ros2_libs/install/setup.bash && "
    "source ~/cartographer_ws/install/setup.bash && "
    "export ROS_DOMAIN_ID=1 && "
    f"export CYCLONEDDS_URI='{CYCLONEDDS_CFG}' && "
)

SLAM_CORE_CMDS = {
    "RViz2": "ros2 run rviz2 rviz2 -d ~/cartographer_ws/src/my_robot_slam/rviz/mapper.rviz",
    "Cartographer": "ros2 launch my_robot_slam online_slam.launch.py",
}

# SLAM_LIVE_CMDS = {
#     "Bag Record": "ros2 bag record -o clean_session_v1 /scan /imu/data /tf /tf_static",
# }


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
    motor_ready = pyqtSignal()

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
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
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
                                self.motor_ready.emit()
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


# ========================== SLAM Worker ==========================
class SLAMWorker(QThread):
    """Manages WSL/ROS2 SLAM subprocesses (RViz2, Cartographer, bag recording)."""
    status_update = pyqtSignal(str)
    core_running = pyqtSignal(bool)
    # live_running = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.is_running = True
        self._core_procs = {}   # name -> Popen
        # self._live_procs = {}   # name -> Popen
        # self._start_live_flag = threading.Event()
        # self._stop_live_flag = threading.Event()
        self._core_ok = False

    def _build_wsl_cmd(self, command):
        return ["wsl", "-d", WSL_DISTRO, "--cd", WSL_PATH, "bash", "-ic",
                WSL_ROS_PREAMBLE + command]

    def run(self):
        # Spawn core processes (RViz2 + Cartographer)
        try:
            for name, cmd in SLAM_CORE_CMDS.items():
                self.status_update.emit(f"SLAM: Launching {name}...")
                proc = subprocess.Popen(
                    self._build_wsl_cmd(cmd),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                self._core_procs[name] = proc
        except (OSError, FileNotFoundError) as e:
            self.status_update.emit(f"SLAM: Failed to launch — {e}")
            self.core_running.emit(False)
            return

        self._core_ok = True
        self.core_running.emit(True)
        self.status_update.emit("SLAM: Core OK")

        # Poll loop
        while self.is_running:
            # Check if core processes died
            for name, proc in list(self._core_procs.items()):
                ret = proc.poll()
                if ret is not None:
                    self.status_update.emit(f"SLAM: {name} exited with code {ret}")
                    del self._core_procs[name]
                    self._core_ok = False
                    self.core_running.emit(False)

            # # Handle start_live request
            # if self._start_live_flag.is_set():
            #     self._start_live_flag.clear()
            #     if self._core_ok:
            #         try:
            #             for name, cmd in SLAM_LIVE_CMDS.items():
            #                 self.status_update.emit(f"SLAM: Starting {name}...")
            #                 proc = subprocess.Popen(
            #                     self._build_wsl_cmd(cmd),
            #                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            #                 )
            #                 self._live_procs[name] = proc
            #             self.live_running.emit(True)
            #             self.status_update.emit("SLAM: Live recording")
            #         except (OSError, FileNotFoundError) as e:
            #             self.status_update.emit(f"SLAM: Live launch failed — {e}")

            # # Handle stop_live request
            # if self._stop_live_flag.is_set():
            #     self._stop_live_flag.clear()
            #     self._terminate_live()

            # # Check if live processes died on their own
            # for name, proc in list(self._live_procs.items()):
            #     ret = proc.poll()
            #     if ret is not None:
            #         self.status_update.emit(f"SLAM: {name} exited with code {ret}")
            #         del self._live_procs[name]

            time.sleep(0.5)

    # def start_live(self):
    #     """Called from the GUI thread to start the live pipeline."""
    #     self._start_live_flag.set()

    # def stop_live(self):
    #     """Called from the GUI thread to stop the live pipeline."""
    #     self._stop_live_flag.set()

    # def _terminate_live(self):
    #     """Terminate live processes only (not core)."""
    #     for name, proc in self._live_procs.items():
    #         if proc.poll() is None:
    #             proc.terminate()
    #     self._live_procs.clear()
    #     self.live_running.emit(False)
    #     self.status_update.emit("SLAM: Live stopped")

    def stop(self):
        """Full shutdown: kill all ROS2 in WSL, terminate all wrappers."""
        self.is_running = False

        # Kill ROS2 processes inside WSL
        subprocess.run(
            ["wsl", "-d", WSL_DISTRO, "bash", "-ic", "pkill ros2"],
            check=False
        )

        # Terminate all Windows-side wsl.exe wrappers
        for proc in self._core_procs.values():
            if proc.poll() is None:
                proc.terminate()

        self.quit()


# ========================== HUD Overlay ==========================
class HUDOverlay(QLabel):
    """Transparent widget that paints motor status over the camera feed."""

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

        # Colors
        panel_bg = QColor(0, 0, 0, 140)
        green = QColor(0, 255, 0, 220)
        white = QColor(255, 255, 255, 240)
        yellow = QColor(255, 220, 0, 220)
        border = QPen(green, 1)

        title_font = QFont("Consolas", 11, QFont.Weight.Bold)
        data_font = QFont("Consolas", 10)
        status_font = QFont("Consolas", 9)

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
                   f"CAM: {self.hud.camera_status}  |  {self.hud.motor_status}"
                   f"  SPD: {self.hud.motor_speed}/127  |  {self.hud.slam_status}")

        p.end()


# ========================== Main HUD Window ==========================
class HUDWindow(QMainWindow):
    """Main window — camera feed background with motor + SLAM HUD overlay."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboKedach HUD")
        self.setStyleSheet("background-color: black;")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Position HUD at left 2/3 of the screen
        screen = QApplication.primaryScreen().availableGeometry()
        hud_w = screen.width() * 2 // 3
        self.setGeometry(screen.x(), screen.y(), hud_w, screen.height() - 35)

        # State
        self.vm_data = {'voltage': 0.0, 'current': 0.0}
        self.camera_status = "Disconnected"
        self.motor_status = "Disconnected"
        self.motor_speed = 0
        self.slam_status = "SLAM: Starting..."
        # self._slam_live = False

        # Continuous movement state
        self._held_move_cmd = None   # Command string for the currently held key
        self._held_key_code = None   # Qt key code of the held movement key

        # Camera background
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("background-color: black;")
        self.setCentralWidget(self.camera_label)

        # HUD overlay (paints on top of the camera)
        self.overlay = HUDOverlay(self)
        self.overlay.raise_()

        # # Start/Stop SLAM button
        # self.slam_btn = QPushButton("Start SLAM", self)
        # self.slam_btn.setEnabled(False)
        # self.slam_btn.setStyleSheet(
        #     "QPushButton {"
        #     "  background-color: rgba(0, 0, 0, 160);"
        #     "  color: #00ff00;"
        #     "  border: 1px solid #00ff00;"
        #     "  border-radius: 5px;"
        #     "  font-family: Consolas;"
        #     "  font-size: 12px;"
        #     "  padding: 8px 20px;"
        #     "}"
        #     "QPushButton:hover { background-color: rgba(0, 255, 0, 40); }"
        #     "QPushButton:disabled { color: #555555; border-color: #555555; }"
        # )
        # self.slam_btn.clicked.connect(self._on_slam_btn)
        # self.slam_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Camera stream worker
        self.cam_worker = MJPEGStreamWorker(CAMERA_URL)
        self.cam_worker.frame_received.connect(self._on_frame)
        self.cam_worker.status_update.connect(self._on_cam_status)
        self.cam_worker.start()

        # Motor command client worker
        self.motor_worker = MotorCommandWorker(RPI_IP, MOTOR_PORT)
        self.motor_worker.status_update.connect(self._on_motor_status)
        self.motor_worker.speed_update.connect(self._on_motor_speed)
        self.motor_worker.motor_ready.connect(self._on_motor_ready)
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

        # SLAM worker (auto-launches core on start)
        self.slam_worker = SLAMWorker()
        self.slam_worker.status_update.connect(self._on_slam_status)
        self.slam_worker.core_running.connect(self._on_core_running)
        # self.slam_worker.live_running.connect(self._on_live_running)
        self.slam_worker.start()

    # # --- SLAM Button ---
    # def _on_slam_btn(self):
    #     if self._slam_live:
    #         self.slam_worker.stop_live()
    #     else:
    #         self.slam_worker.start_live()

    # --- Key Input ---
    def keyPressEvent(self, event):
        """Capture arrow keys and send motor commands to the RPi."""
        if event.isAutoRepeat():
            return

        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        key = event.key()

        cmd = None
        is_movement = False

        if key == Qt.Key.Key_Up:
            cmd = "FWD90" if shift else "FWD"
            is_movement = True
        # elif key == Qt.Key.Key_Down:
        #     cmd = "BWD90" if shift else "BWD"
        #     is_movement = True
        elif key == Qt.Key.Key_Left:
            cmd = "LEFT90" if shift else "LEFT"
            is_movement = True
        elif key == Qt.Key.Key_Right:
            cmd = "RIGHT90" if shift else "RIGHT"
            is_movement = True
        elif key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            cmd = "SPEED_UP"
        elif key == Qt.Key.Key_Minus:
            cmd = "SPEED_DOWN"
        elif key == Qt.Key.Key_Space:
            cmd = "ABORT"
            # Abort cancels continuous movement
            self._held_move_cmd = None
            self._held_key_code = None
        elif key == Qt.Key.Key_R:
            cmd = "RESET_ENC"

        if cmd:
            if is_movement:
                self._held_move_cmd = cmd
                self._held_key_code = key
            self.motor_worker.send_command(cmd)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Clear held movement state when the movement key is released."""
        if event.isAutoRepeat():
            return

        if event.key() == self._held_key_code:
            self._held_move_cmd = None
            self._held_key_code = None

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

    def _on_motor_status(self, msg):
        self.motor_status = msg
        self.overlay.update()

    def _on_motor_speed(self, speed):
        self.motor_speed = speed
        self.overlay.update()

    def _on_motor_ready(self):
        """When RPi finishes a rotation and is ready, re-send if key still held."""
        if self._held_move_cmd:
            self.motor_worker.send_command(self._held_move_cmd)

    def _on_slam_status(self, msg):
        self.slam_status = msg
        self.overlay.update()

    def _on_vm_data(self, voltage, current):
        self.vm_data = {'voltage': voltage, 'current': current}
        self.overlay.update()

    def _on_vm_status(self, msg):
        self.vm_status = msg
        self.overlay.update()

    def _on_core_running(self, running):
        pass
        # self.slam_btn.setEnabled(running)
        # if not running:
        #     self._slam_live = False
        #     self.slam_btn.setText("Start SLAM")

    # def _on_live_running(self, running):
    #     self._slam_live = running
    #     self.slam_btn.setText("Stop SLAM" if running else "Start SLAM")

    # --- Events ---
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        # # Position SLAM button at top-right
        # btn_w, btn_h = 140, 36
        # self.slam_btn.setGeometry(self.width() - btn_w - 15, 15, btn_w, btn_h)

    def closeEvent(self, event):
        self.vm_timer.stop()
        self.cam_worker.stop()
        self.motor_worker.stop()
        self.slam_worker.stop()
        self.vm_worker.stop()
        self.cam_worker.wait(3000)
        self.motor_worker.wait(3000)
        self.slam_worker.wait(5000)
        self.vm_worker.wait(3000)
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HUDWindow()
    window.show()
    sys.exit(app.exec())
