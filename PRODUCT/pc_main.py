"""
pc_main.py - Base Station HUD Dashboard.
PyQt6 full-screen overlay with live camera feed, keyboard teleoperation, and
power monitoring. Auto-launches RViz2 + Cartographer in WSL on startup. Two HUD
buttons: Start SLAM (sends START_SLAM to Pi to launch LIDAR/IMU) and Record Bag
(toggles timestamped rosbag recording with graceful SIGINT stop).
"""

import os
import ctypes
import sys
import json
import math
import time
from datetime import datetime

import paramiko
import socket
import struct
import subprocess
import threading
import urllib.request

from PyQt6.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt6.QtGui import QPixmap, QImage, QPainter, QFont, QColor, QPen, QIcon
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton


from network_utils import setup_windows_network, is_windows_network_ready
# ========================== Configuration ==========================
def get_config():
    """Load system configuration from config.json."""
    locations = [
        os.path.join(os.path.dirname(__file__), "config.json"),
        os.path.join(os.path.dirname(__file__), "RPI", "config.json"),
        "config.json"
    ]
    for loc in locations:
        if os.path.exists(loc):
            with open(loc, 'r') as f:
                return json.load(f)
    raise FileNotFoundError("Could not find config.json")

CONFIG = get_config()

# Raspberry Pi SSH credentials
RPI_IP       = CONFIG["network"]["rpi_ip"]
RPI_SSH_HOST = CONFIG["network"]["ssh"]["host"]
RPI_SSH_USER = CONFIG["network"]["ssh"]["user"]
RPI_SSH_PASS = CONFIG["network"]["ssh"]["pass"]

MOTOR_PORT = CONFIG["network"]["motor_port"]
VM_PORT    = CONFIG["network"]["vm_port"]
CAMERA_URL = f"http://{RPI_IP}:{CONFIG['network']['camera_port']}/"

WSL_DISTRO = CONFIG["wsl"]["distro"]
WSL_PATH   = CONFIG["wsl"]["path"]

# Pin Cyclone DDS to the fiber interface only, unicast data to prevent network flood
PC_FIBER_IP = CONFIG["network"]["pc_ip"]
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
    f"export ROS_DOMAIN_ID={CONFIG['wsl']['ros_domain_id']} && "
    f"export CYCLONEDDS_URI='{CYCLONEDDS_CFG}' && "
)

SLAM_CORE_CMDS = CONFIG["wsl"]["core_commands"]

def _bag_record_cmd():
    """Generate a bag record command with a timestamped output name."""

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return f"mkdir -p ~/bags && ros2 bag record -o ~/bags/bag_{stamp} /scan /imu/data /tf /tf_static"


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

            # Diagnostic: Print all local IPs to ensure 192.168.1.1 is active
            try:
                hostname = socket.gethostname()
                local_ips = socket.gethostbyname_ex(hostname)[2]
                print(f"VM Server: Started. Local IPs detected: {local_ips}")
            except Exception:
                pass

            self.status_update.emit(f"VM: Listening on port {self.port}")
            print(f"VM Server: Listening on port {self.port} (IP: {self.host})")
            print("VM Server: Waiting for RPi connection...")

            while self.is_running:
                try:
                    # This will block for 1s (due to settimeout) then throw socket.timeout
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
    speed_update = pyqtSignal(int, int)
    encoder_update = pyqtSignal(int, int)
    motor_ready = pyqtSignal()
    heading_calibrated = pyqtSignal()
    heading_received = pyqtSignal(float)

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
                        try:
                            buf += data.decode('utf-8')
                        except UnicodeDecodeError:
                            continue

                        while buf and '\n' in buf:
                            line, buf = buf.split('\n', 1)
                            msg = line.strip()
                            print(f"Motor: Command recieved - {msg}")
                            if msg.startswith("SPEED:"):
                                try:
                                    left_s, right_s = msg[6:].split(",", 1)
                                    self.speed_update.emit(int(left_s), int(right_s))
                                except ValueError:
                                    pass
                            elif msg == "BUSY":
                                self.status_update.emit("Motor: Moving...")
                            elif msg == "READY":
                                self.status_update.emit("Motor: Ready")
                                self.motor_ready.emit()
                            elif msg.startswith("ENC:"):
                                try:
                                    left_enc, right_enc = msg[4:].split(",", 1)
                                    self.encoder_update.emit(int(left_enc), int(right_enc))
                                except ValueError:
                                    pass
                            elif msg == "HEADING_CALIBRATED":
                                self.heading_calibrated.emit()
                            elif msg.startswith("HEADING:"):
                                try:
                                    deg = float(msg[8:])
                                    self.heading_received.emit(deg)
                                except ValueError:
                                    pass
                            elif msg.startswith("HEADING_ERROR:"):
                                self.status_update.emit(f"Heading: {msg}")
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
        if cmd != "HEARTBEAT":
            print(f"Motor: Sending command -> {cmd}")

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
    live_running = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.is_running = True
        self._core_procs = {}   # name -> Popen
        self._live_procs = {}   # name -> Popen
        self._start_live_flag = threading.Event()
        self._stop_live_flag = threading.Event()
        self._publish_tf_flag = threading.Event()
        self._core_ok = False
        self._heading_deg = None  # Set before launching SLAM core

    def _build_wsl_cmd(self, command):
        return ["wsl", "-d", WSL_DISTRO, "--cd", WSL_PATH, "bash", "-ic", WSL_ROS_PREAMBLE + command]

    def publish_north_tf(self, heading_deg):
        """Dynamically launch the static TF publisher once heading is known."""
        
        self._heading_deg = heading_deg
        self._publish_tf_flag.set()

    def run(self):
        # Auto-launch core processes (RViz2 + Cartographer)
        try:
            for name, cmd in SLAM_CORE_CMDS.items():
                self.status_update.emit(f"SLAM: Launching {name}...")
                proc = subprocess.Popen(
                    self._build_wsl_cmd(cmd),
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    creationflags=0x08000000
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

            # Hold off on launching the North TF until the heading is calculated
            if self._publish_tf_flag.is_set():
                self._publish_tf_flag.clear()
                if self._heading_deg is not None:
                    heading_rad = math.radians(-self._heading_deg - 90)
                    tf_cmd = (f"ros2 run tf2_ros static_transform_publisher "
                              f"--x 0 --y 0 --z 0 "
                              f"--yaw {heading_rad} --pitch 0 --roll 0 "
                              f"--frame-id north --child-frame-id map")
                    self.status_update.emit(
                        f"SLAM: Publishing north TF (heading={self._heading_deg:.2f} deg)...")
                    try:
                        proc = subprocess.Popen(
                            self._build_wsl_cmd(tf_cmd),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                        )
                        self._core_procs["NorthTF"] = proc
                    except Exception as e:
                        self.status_update.emit(f"SLAM: Failed to publish TF — {e}")

            # Handle start_live request
            if self._start_live_flag.is_set():
                self._start_live_flag.clear()
                if self._core_ok:
                    try:
                        cmd = _bag_record_cmd()
                        self.status_update.emit("SLAM: Starting Bag Record...")
                        proc = subprocess.Popen(
                            self._build_wsl_cmd(cmd),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                        )
                        self._live_procs["Bag Record"] = proc
                        self.live_running.emit(True)
                        self.status_update.emit("SLAM: Recording bag")
                    except (OSError, FileNotFoundError) as e:
                        self.status_update.emit(f"SLAM: Live launch failed — {e}")

            # Handle stop_live request
            if self._stop_live_flag.is_set():
                self._stop_live_flag.clear()
                self._terminate_live()
            else:
                # Check if live processes died on their own (skip if we just terminated)
                had_live = bool(self._live_procs)
                for name, proc in list(self._live_procs.items()):
                    ret = proc.poll()
                    if ret is not None:
                        self.status_update.emit(f"SLAM: {name} exited with code {ret}")
                        del self._live_procs[name]
                if had_live and not self._live_procs:
                    self.live_running.emit(False)

            time.sleep(0.5)

    def start_live(self):
        """Called from the GUI thread to start bag recording."""

        self._start_live_flag.set()

    def stop_live(self):
        """Called from the GUI thread to stop the live pipeline."""

        self._stop_live_flag.set()

    def _terminate_live(self):
        """Gracefully stop live processes via SIGINT inside WSL (like Ctrl+C)."""

        for _, proc in self._live_procs.items():
            if proc.poll() is None:
                # Send SIGINT to the actual process inside WSL, not the wrapper
                subprocess.run(
                    ["wsl", "-d", WSL_DISTRO, "bash", "-ic",
                     "pkill -INT -f 'ros2 bag record'"],
                    check=False
                )
                # Give it a moment to flush metadata, then force-kill if stuck
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.terminate()
        self._live_procs.clear()
        self.live_running.emit(False)
        self.status_update.emit("SLAM: Live stopped")

    def stop(self):
        """Full shutdown: kill all ROS2 in WSL, terminate all wrappers."""

        self.is_running = False

        # Kill all ROS2 processes inside WSL (covers both core and bag record)
        subprocess.run(
            ["wsl", "-d", WSL_DISTRO, "bash", "-ic", "pkill ros2"],
            check=False
        )

        # Terminate all Windows-side wsl.exe wrappers
        for proc in list(self._live_procs.values()) + list(self._core_procs.values()):
            if proc.poll() is None:
                proc.terminate()
        self._live_procs.clear()
        self._core_procs.clear()

        self.quit()



# ========================== RPi Remote Worker ==========================
class RPiRemoteWorker(QThread):
    """SSH into the RPi and run rpi_main.py. Streams output to console."""

    status_update = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.is_running = True
        self._ssh = None

    def run(self):
        while self.is_running:
            try:
                self.status_update.emit("RPi: Connecting via SSH...")
                ssh = paramiko.SSHClient()
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh.connect(hostname=RPI_SSH_HOST, username=RPI_SSH_USER,
                            password=RPI_SSH_PASS, timeout=10)
                self._ssh = ssh
                self.status_update.emit("RPi: SSH connected. Launching rpi_main.py...")

                stdin, stdout, stderr = ssh.exec_command(
                    "cd Desktop/PRESENT && python3 rpi_main.py",
                    get_pty=True
                )

                # Stream stdout until the remote process exits or we're told to stop
                for line in iter(stdout.readline, ""):
                    if not self.is_running:
                        break
                    stripped = line.rstrip()
                    if stripped:
                        print(f"[RPi] {stripped}")

                exit_code = stdout.channel.recv_exit_status()
                self.status_update.emit(f"RPi: rpi_main.py exited with code {exit_code}")

            except Exception as e:
                self.status_update.emit(f"RPi SSH: {e}")
            finally:
                if self._ssh:
                    try:
                        self._ssh.close()
                    except Exception:
                        pass
                    self._ssh = None

            if self.is_running:
                time.sleep(5)  # retry after a delay

    def stop(self):
        self.is_running = False
        if self._ssh:
            try:
                self._ssh.close()
            except Exception:
                pass
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

    def _draw_leg_box(self, p, x, y, sz, active, green, white, panel_bg):
        """Draw a single leg indicator box with a down-arrow."""

        color = green if active else white
        p.setPen(QPen(color, 1.5))
        p.setBrush(panel_bg)
        p.drawRoundedRect(x, y, sz, sz, 3, 3)

        # Draw down arrow inside the box
        cx = x + sz // 2
        top_y = y + 5
        bot_y = y + sz - 5

        p.setPen(QPen(color, 2))
        p.drawLine(cx, top_y, cx, bot_y)            # shaft
        p.drawLine(cx, bot_y, cx - 4, bot_y - 5)    # left head
        p.drawLine(cx, bot_y, cx + 4, bot_y - 5)    # right head

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()

        # Colors
        panel_bg = QColor(0, 0, 0, 140)
        green = QColor(0, 255, 0, 220)
        white = QColor(255, 255, 255, 240)
        yellow = QColor(255, 255, 0, 220)
        orange = QColor(255, 165, 0, 220)
        red = QColor(255, 0, 0, 220)
        border = QPen(green, 1)

        title_font = QFont("Consolas", 11, QFont.Weight.Bold)
        status_font = QFont("Consolas", 9)

        # --- Crosshair (center of screen) ---
        cx, cy = w // 2, h // 2
        solid_len = 22    # solid segment length from center
        gap       = 7     # gap around center point

        solid_pen = QPen(QColor(0, 255, 0, 240), 3.5)
        solid_pen.setCapStyle(Qt.PenCapStyle.RoundCap)

        dot_pen = QPen(QColor(0, 255, 0, 170), 2.5)
        dot_pen.setStyle(Qt.PenStyle.CustomDashLine)
        dot_pen.setDashPattern([8, 6])   # 8px dash, 6px gap
        dot_pen.setCapStyle(Qt.PenCapStyle.RoundCap)

        # Solid arms
        p.setPen(solid_pen)
        p.drawLine(cx - solid_len - gap, cy, cx - gap, cy)   # left solid
        p.drawLine(cx + gap, cy, cx + solid_len + gap, cy)   # right solid
        p.drawLine(cx, cy - solid_len - gap, cx, cy - gap)   # top solid
        p.drawLine(cx, cy + gap, cx, cy + solid_len + gap)   # bottom solid

        # Edge tick marks — short lines at each screen edge aligned with crosshair
        tick = 18          # tick length in px
        margin = 20        # distance from screen edge
        p.drawLine(margin, cy, margin + tick, cy)              # left edge
        p.drawLine(w - margin - tick, cy, w - margin, cy)     # right edge
        p.drawLine(cx, margin, cx, margin + tick)              # top edge
        p.drawLine(cx, h - 4*margin - tick, cx, h - 4*margin)     # bottom edge

        # --- Power Monitor Panel (top-left) ---
        pm_y = 10
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(10, pm_y, 190, 80, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(22, pm_y + 24, "POWER MONITOR")

        p.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        vm = self.hud.vm_data
        p.setPen(white)
        p.drawText(22, pm_y + 50, f"{vm['voltage']:6.2f} V")
        p.drawText(22, pm_y + 70, f"{vm['current']:6.2f} A")

        # Voltage battery indicator
        v = vm['voltage']
        if v > 0:
            bx = 150
            by = pm_y + 22
            bw = 26
            bh = 42
            cap_w = 10
            cap_h = 4

            # Outline and cap
            p.setPen(QPen(white, 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(bx, by, bw, bh, 2, 2)
            p.setBrush(white)
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRect(bx + (bw - cap_w) // 2, by - cap_h, cap_w, cap_h)

            if v > 11.7:
                level = 4
                bat_color = green
            elif v > 11.2:
                level = 3
                bat_color = yellow
            elif v > 10.8:
                level = 2
                bat_color = orange
            elif v > 10.5:
                level = 1
                bat_color = red
            else:
                level = 0
                bat_color = red

            if level > 0:
                pad = 2
                seg_gap = 2
                seg_h = (bh - 2 * pad - 3 * seg_gap) // 4
                seg_w = bw - 2 * pad
                p.setBrush(bat_color)
                for i in range(level):
                    seg_x = bx + pad
                    seg_y = by + bh - pad - (i + 1) * seg_h - i * seg_gap
                    p.drawRect(seg_x, seg_y, int(seg_w), int(seg_h))

        p.setBrush(panel_bg)

        # Layout constants for status bar
        sb_h = 50
        sb_y = h - sb_h - 10

        # --- Differential Speed Panel (bottom-left, above status bar) ---
        sp_w = 140
        sp_h = 120
        sp_x = 10
        sp_y = sb_y - 10 - sp_h  # 10px gap above status bar
        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(sp_x, sp_y, sp_w, sp_h, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(sp_x + 12, sp_y + 22, "SPEED")

        # Bar track geometry
        bar_w = 14
        bar_top = sp_y + 35
        bar_bottom = sp_y + sp_h - 26
        bar_h = bar_bottom - bar_top

        # Three evenly-spaced columns (left=L, middle=avg, right=R)
        col_centers = [sp_x + 28, sp_x + 66, sp_x + 104]

        # Range labels (top=127, bottom=10)
        range_font = QFont("Consolas", 8)
        p.setFont(range_font)
        p.setPen(white)
        p.drawText(sp_x + sp_w - 25, bar_top + 6, "127")
        p.drawText(sp_x + sp_w - 20, bar_bottom + 4, "10")

        # Slot values and colors
        lo, hi = self.hud.SPEED_MIN, self.hud.SPEED_MAX
        left_val = self.hud.motor_left_speed
        right_val = self.hud.motor_right_speed
        avg_val = (left_val + right_val) // 2

        green_active = green                       # sides — bright green
        green_muted = QColor(0, 180, 0, 110)       # middle average — grayed-out green

        slots = [
            (col_centers[0], left_val, green_active),
            (col_centers[1], avg_val, green_muted),
            (col_centers[2], right_val, green_active),
        ]

        track_bg = QColor(255, 255, 255, 40)
        for cx, val, fill_color in slots:
            bx = cx - bar_w // 2
            # Track
            p.setPen(QPen(green, 1))
            p.setBrush(track_bg)
            p.drawRect(bx, bar_top, bar_w, bar_h)

            # Fill proportional to (val - lo) / (hi - lo), clamped
            frac = max(0.0, min(1.0, (val - lo) / float(hi - lo)))
            fill_h = int(bar_h * frac)
            if fill_h > 0:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(fill_color)
                p.drawRect(bx + 1, bar_bottom - fill_h + 1, bar_w - 2, fill_h - 1)

            # Numeric value below bar
            p.setPen(white)
            p.setFont(range_font)
            p.drawText(cx - 10, bar_bottom + 16, f"{val:>3d}")

        p.setBrush(panel_bg)

        # --- Robot Legs Panel (bottom-right, above status bar) ---
        lp_w = 140
        lp_h = 180
        lp_x = w - lp_w - 10
        lp_y = sb_y - 10 - lp_h

        p.setBrush(panel_bg)
        p.setPen(border)
        p.drawRoundedRect(lp_x, lp_y, lp_w, lp_h, 8, 8)

        p.setFont(title_font)
        p.setPen(green)
        p.drawText(lp_x + 12, lp_y + 22, "LEGS STATUS")

        # Robot body illustration (centered in panel)
        body_w = 36
        body_h = 120
        body_x = lp_x + (lp_w - body_w) // 2
        body_y = lp_y + 42

        p.setPen(QPen(white, 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(body_x, body_y, body_w, body_h, 6, 6)

        # Camera nub on top
        nub_w = 16
        nub_h = 6
        p.setBrush(white)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(body_x + (body_w - nub_w) // 2, body_y - nub_h, nub_w, nub_h)

        # Forward arrow inside body
        arr_cx = body_x + body_w // 2
        arr_cy = body_y + 22
        p.setPen(QPen(white, 2))
        p.drawLine(arr_cx, arr_cy + 14, arr_cx, arr_cy - 8)           # shaft
        p.drawLine(arr_cx, arr_cy - 8, arr_cx - 6, arr_cy - 1)       # left head
        p.drawLine(arr_cx, arr_cy - 8, arr_cx + 6, arr_cy - 1)       # right head

        # Camera lens (small square + circle) inside body
        lens_y = arr_cy + 22
        lens_sz = 16
        p.setPen(QPen(white, 1.5))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(arr_cx - lens_sz // 2, lens_y, lens_sz, lens_sz, 2, 2)
        p.drawEllipse(arr_cx - 5, lens_y + 3, 10, 10)

        # Cable dot at bottom
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(white)
        p.drawEllipse(arr_cx - 3, body_y + body_h - 12, 6, 6)

        # Cable line hanging below robot body
        cable_top_y = body_y + body_h
        cable_bot_y = lp_y + lp_h - 6
        cable_pen = QPen(white, 1.5)
        cable_pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(cable_pen)
        p.drawLine(arr_cx, cable_top_y, arr_cx, cable_bot_y)

        # --- Leg arrow boxes (3 on each side) ---
        box_sz = 22
        leg_offsets = self.hud.LEG_OFFSETS   # [L1, L2, L3, R1, R2, R3]
        phase_l = self.hud.leg_phase_left
        phase_r = self.hud.leg_phase_right
        leg_cycle = [False, True, False, False]  # white, GREEN, white, white

        # Vertical positions for the 3 rows of legs (front, mid, rear)
        leg_ys = [
            body_y + 8,
            body_y + (body_h - box_sz) // 2,
            body_y + body_h - box_sz - 8,
        ]

        left_x  = body_x - box_sz - 10
        right_x = body_x + body_w + 10

        for i in range(3):
            # Left leg (indices 0-2) — uses right phase
            l_active = leg_cycle[(phase_r - leg_offsets[i]) % 4]
            self._draw_leg_box(p, left_x, leg_ys[i], box_sz, l_active, green, QColor(255, 255, 255, 120), panel_bg)
            # Right leg (indices 3-5) — uses left phase
            r_active = leg_cycle[(phase_l - leg_offsets[i + 3]) % 4]
            self._draw_leg_box(p, right_x, leg_ys[i], box_sz, r_active, green, QColor(255, 255, 255, 120), panel_bg)

        # --- Status Bar (bottom) ---
        p.setBrush(QColor(0, 0, 0, 160))
        p.setPen(border)
        p.drawRoundedRect(10, sb_y, w - 20, sb_h, 5, 5)

        p.setFont(status_font)
        p.setPen(green)

        # Derive simplified status strings
        cam_raw = self.hud.camera_status.lower()
        cam_st = "Connected" if "connected" in cam_raw else "Disconnected"

        vm_raw = getattr(self.hud, 'vm_status', '')
        vm_st = "Connected" if "connected" in vm_raw.lower() else "Disconnected"

        motor_raw = self.hud.motor_status.lower()
        motor_st = "Disconnected" if "disconnected" in motor_raw else "Connected"

        bold_font   = QFont("Consolas", 9, QFont.Weight.Bold)
        normal_font = QFont("Consolas", 9)
        label_gap   = 55   # pixels to skip past the bold label

        # Row 1 — BASE:
        p.setFont(bold_font)
        p.drawText(20, sb_y + 20, "BASE:")
        p.setFont(normal_font)
        p.drawText(20 + label_gap, sb_y + 20, f"COMM: OK  |  {self.hud.heading_status}  |  {self.hud.slam_status}")

        # Row 2 — ROBOT:
        p.setFont(bold_font)
        p.drawText(20, sb_y + 40, "ROBOT:")
        p.setFont(normal_font)
        p.drawText(20 + label_gap, sb_y + 40, f"VM: {vm_st}  |  CAMERA: {cam_st}  |  MOTORS: {motor_st}")

        p.end()


# ========================== Main HUD Window ==========================
def resource_path(relative_path):
        """Get absolute path to resource, works for dev and for PyInstaller"""

        try:
            base_path = sys._MEIPASS
        except Exception:
            base_path = os.path.abspath(".")
        return os.path.join(base_path, relative_path)


class HUDWindow(QMainWindow):
    """Main window — camera feed background with motor + SLAM HUD overlay."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboKedach HUD")
        self.setWindowIcon(QIcon(resource_path("robokedach_icon.ico")))
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
        self.motor_left_speed = CONFIG["hardware"]["motors"]["default_speed"]
        self.motor_right_speed = CONFIG["hardware"]["motors"]["default_speed"]
        self.SPEED_MIN = CONFIG["hardware"]["motors"]["speed_min"]
        self.SPEED_MAX = CONFIG["hardware"]["motors"]["speed_max"]
        self.slam_status = "SLAM: Starting..."
        self._slam_live = False
        self._slam_started = False
        self.heading_status = "Heading: Idle"
        self._heading_deg = None
        self._heading_tracking = False

        # Leg phase animation state
        # Phase offsets per leg — cycle is [white, GREEN, white, white]
        # At phase p, leg with offset o → cycle step = (p - o) % 4
        self.leg_phase_left = 0      # left side quarter-rotation counter (0-3)
        self.leg_phase_right = 0     # right side quarter-rotation counter (0-3)
        self._leg_phase_offset_left = 0   # saved offset on encoder reset
        self._leg_phase_offset_right = 0
        self.LEG_OFFSETS = CONFIG["hardware"]["motors"]["leg_offsets"]

        # Continuous movement state
        self._held_key_code = None   # Qt key code of the held movement key

        # Camera background
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("background-color: black;")
        self.setCentralWidget(self.camera_label)

        # HUD overlay (paints on top of the camera)
        self.overlay = HUDOverlay(self)
        self.overlay.raise_()

        # Feedback popup label
        self.popup_label = QLabel(self)
        self.popup_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.popup_label.setStyleSheet(
            "background-color: rgba(0, 0, 0, 180);"
            "color: #00ff00;"
            "border: 2px solid #00ff00;"
            "border-radius: 10px;"
            "font-family: Consolas;"
            "font-size: 28px;"
            "font-weight: bold;"
            "padding: 15px 30px;"
        )
        self.popup_label.hide()

        self.popup_timer = QTimer(self)
        self.popup_timer.setSingleShot(True)
        self.popup_timer.timeout.connect(self.popup_label.hide)

        # Persistent low battery warning label
        self.bat_warn_label = QLabel("⚠  LOW BATTERY", self)
        self.bat_warn_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.bat_warn_label.setStyleSheet(
            "background-color: rgba(0, 0, 0, 180);"
            "color: #ffff00;"
            "border: 2px solid #ffff00;"
            "border-radius: 10px;"
            "font-family: Consolas;"
            "font-size: 28px;"
            "font-weight: bold;"
            "padding: 15px 30px;"
        )
        self.bat_warn_label.hide()

        # Shared button style
        btn_style = (
            "QPushButton {"
            "  background-color: rgba(0, 0, 0, 160);"
            "  color: #00ff00;"
            "  border: 1px solid #00ff00;"
            "  border-radius: 5px;"
            "  font-family: Consolas;"
            "  font-size: 14px;"
            "  font-weight: bold;"
            "  padding: 6px 10px;"
            "}"
            "QPushButton:hover { background-color: rgba(0, 255, 0, 40); }"
            "QPushButton:disabled { color: #555555; border-color: #555555; }"
        )

        # Heading Track button
        self.heading_btn = QPushButton("Track Heading", self)
        self.heading_btn.setStyleSheet(btn_style)
        self.heading_btn.clicked.connect(self._on_heading_btn)
        self.heading_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Landed button (sends HEADING_LANDED, disabled until heading tracking starts)
        self.landed_btn = QPushButton("Landed", self)
        self.landed_btn.setEnabled(False)
        self.landed_btn.setStyleSheet(btn_style)
        self.landed_btn.clicked.connect(self._on_landed_btn)
        self.landed_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Start SLAM button (disabled until heading is received)
        self.slam_btn = QPushButton("Start SLAM", self)
        self.slam_btn.setEnabled(False)
        self.slam_btn.setStyleSheet(btn_style)
        self.slam_btn.clicked.connect(self._on_slam_btn)
        self.slam_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Record Bag button (start/stop bag recording, disabled until SLAM is running)
        self.bag_btn = QPushButton("Record Bag", self)
        self.bag_btn.setEnabled(False)
        self.bag_btn.setStyleSheet(btn_style)
        self.bag_btn.clicked.connect(self._on_bag_btn)
        self.bag_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # Camera stream worker
        self.cam_worker = MJPEGStreamWorker(CAMERA_URL)
        self.cam_worker.frame_received.connect(self._on_frame)
        self.cam_worker.status_update.connect(self._on_cam_status)
        self.cam_worker.start()

        # Motor command client worker
        self.motor_worker = MotorCommandWorker(RPI_IP, MOTOR_PORT)
        self.motor_worker.status_update.connect(self._on_motor_status)
        self.motor_worker.speed_update.connect(self._on_motor_speed)
        self.motor_worker.encoder_update.connect(self._on_encoder_update)
        self.motor_worker.motor_ready.connect(self._on_motor_ready)
        self.motor_worker.heading_calibrated.connect(self._on_heading_calibrated)
        self.motor_worker.heading_received.connect(self._on_heading_received)
        self.motor_worker.start()

        # VM (voltage monitor) server worker
        self.vm_worker = VMServerWorker()
        self.vm_worker.data_received.connect(self._on_vm_data)
        self.vm_worker.status_update.connect(self._on_vm_status)
        self.vm_worker.start()

        # Periodic repaint for VM panel (ensures updates even when IMU/camera are idle)
        self.vm_timer = QTimer(self)
        self.vm_timer.timeout.connect(self.overlay.update)
        self.vm_timer.start(100)

        # Heartbeat timer — RPi watchdog stops motors if this stops arriving
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(lambda: self.motor_worker.send_command("HEARTBEAT"))
        self.heartbeat_timer.start(500)

        # SLAM worker (auto-launches RViz2 + Cartographer)
        self.slam_worker = SLAMWorker()
        self.slam_worker.status_update.connect(self._on_slam_status)
        self.slam_worker.core_running.connect(self._on_core_running)
        self.slam_worker.live_running.connect(self._on_live_running)
        self.slam_worker.start()

        # RPi remote worker (auto-launches rpi_main.py via SSH)
        self.rpi_worker = RPiRemoteWorker()
        self.rpi_worker.status_update.connect(lambda msg: print(msg))
        self.rpi_worker.start()

    # --- Heading Buttons ---
    def _on_heading_btn(self):
        self.motor_worker.send_command("START_HEADING_TRACK")
        self.heading_btn.setEnabled(False)
        self.heading_btn.setText("Calibrating...")
        self.heading_status = "Heading: Calibrating..."
        self._heading_tracking = True
        self.overlay.update()

    def _on_landed_btn(self):
        self.motor_worker.send_command("HEADING_LANDED")
        self.landed_btn.setEnabled(False)
        self.landed_btn.setText("Computing...")
        self.heading_status = "Heading: Computing..."
        self.overlay.update()

    def _on_heading_calibrated(self):
        self.heading_status = "Heading: Tracking descent..."
        self.heading_btn.setText("Tracking...")
        self.landed_btn.setEnabled(True)
        self.overlay.update()

    def _on_heading_received(self, heading_deg):
        self._heading_deg = heading_deg
        self._heading_tracking = False
        self.heading_status = f"Heading: {heading_deg:+.2f} deg"
        self.landed_btn.setText("Done")
        # Enable Start SLAM now that heading is known
        self.slam_btn.setEnabled(True)
        # Pass heading to SLAM worker for TF publisher
        self.slam_worker.publish_north_tf(heading_deg)
        self.overlay.update()

    # --- SLAM Buttons ---
    def _on_slam_btn(self):
        self.motor_worker.send_command("START_SLAM")
        self._slam_started = True
        self.slam_btn.setEnabled(False)
        self.slam_btn.setText("SLAM Active")
        self.bag_btn.setEnabled(True)

    def _on_bag_btn(self):
        if self._slam_live:
            self.slam_worker.stop_live()
        else:
            self.slam_worker.start_live()

    # --- Key Input ---
    def keyPressEvent(self, event):
        """Capture arrow keys and send motor commands to the RPi."""

        if event.isAutoRepeat():
            return

        print(f"HUD: Key pressed - {event.key()}")
        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        key = event.key()

        cmd = None
        is_movement = False

        if key == Qt.Key.Key_Up:
            cmd = "FWD90" if shift else "FWD"
            is_movement = True
        elif key == Qt.Key.Key_Left:
            cmd = "LEFT90" if shift else "LEFT"
            is_movement = True
        elif key == Qt.Key.Key_Right:
            cmd = "RIGHT90" if shift else "RIGHT"
            is_movement = True
        elif key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            cmd = "DIFF_UP" if shift else "SPEED_UP"
        elif key in (Qt.Key.Key_Minus, Qt.Key.Key_Underscore):
            cmd = "DIFF_DOWN" if shift else "SPEED_DOWN"
        elif key == Qt.Key.Key_Space:
            cmd = "ABORT"
            self._held_key_code = None  # Abort cancels continuous movement
        elif key == Qt.Key.Key_R:
            cmd = "RESET_ENC"
            # Preserve current leg phase as offset so visuals stay continuous
            self._leg_phase_offset_left = self.leg_phase_left
            self._leg_phase_offset_right = self.leg_phase_right
            self.show_popup("ENCODERS RESET")

        if cmd:
            if is_movement:
                self._held_key_code = key
            self.motor_worker.send_command(cmd)
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Clear held movement state and tell RPi to stop after current rotation."""

        if event.isAutoRepeat():
            return

        if event.key() == self._held_key_code:
            self._held_key_code = None
            self.motor_worker.send_command("STOP_MOVE")

    def show_popup(self, text):
        """Display a temporary centered floating feedback message."""
        self.popup_label.setText(text)
        self.popup_label.adjustSize()
        pw, ph = self.popup_label.width(), self.popup_label.height()
        self.popup_label.setGeometry((self.width() - pw) // 2, (self.height() - ph) // 2, pw, ph)
        self.popup_label.show()
        self.popup_label.raise_()
        self.popup_timer.start(1500)

    def _on_motor_ready(self):
        """RPi reports movement complete."""
        print("HUD: Motor READY signal received")
        self.motor_status = "Motor: Ready"
        self.overlay.update()

    def _on_encoder_update(self, enc1, enc2):
        """Update leg phases directly from absolute encoder positions."""
        # 8400 ticks / 4 quarters = 2100 ticks per quarter
        # enc1 tracks physical M2 (logical M1/Left)
        # enc2 tracks physical M1 (logical M2/Right)
        # Both encoders are inverted by multiplier -1 in RPi, so positive = forward
        
        raw_left = ((enc1 // 2100) + 1) % 4
        raw_right = ((enc2 // 2100) + 1) % 4
        self.leg_phase_left = (raw_left + self._leg_phase_offset_left) % 4
        self.leg_phase_right = (raw_right + self._leg_phase_offset_right) % 4
        self.overlay.update()

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

    def _on_motor_speed(self, left_speed, right_speed):
        self.motor_left_speed = left_speed
        self.motor_right_speed = right_speed
        self.overlay.update()

    def _on_slam_status(self, msg):
        self.slam_status = msg
        self.overlay.update()

    def _on_vm_data(self, voltage, current):
        self.vm_data = {'voltage': voltage, 'current': current}
        self.overlay.update()

        # Show/hide persistent low battery warning (orange = <=10.8V)
        if voltage > 0 and voltage <= 10.8:
            if not self.bat_warn_label.isVisible():
                self.bat_warn_label.adjustSize()
                pw, ph = self.bat_warn_label.width(), self.bat_warn_label.height()
                self.bat_warn_label.setGeometry((self.width() - pw) // 2, (self.height() - ph) // 2, pw, ph)
                self.bat_warn_label.show()
                self.bat_warn_label.raise_()
        else:
            self.bat_warn_label.hide()

    def _on_vm_status(self, msg):
        self.vm_status = msg
        self.overlay.update()

    def _on_core_running(self, running):
        if running and not self._slam_started and self._heading_deg is not None:
            self.slam_btn.setEnabled(True)
        if not running:
            self._slam_live = False
            self._slam_started = False
            self.slam_btn.setEnabled(False)
            self.slam_btn.setText("Start SLAM")
            self.bag_btn.setEnabled(False)
            self.bag_btn.setText("Record Bag")

    def _on_live_running(self, running):
        self._slam_live = running
        self.bag_btn.setText("Stop Bag" if running else "Record Bag")

    # --- Events ---
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        # Position buttons at top-right (Heading Track, Landed, Start SLAM, Record Bag)
        btn_w, btn_h, gap = 170, 36, 6
        x = self.width() - btn_w - 15
        y = 15
        self.heading_btn.setGeometry(x, y, btn_w, btn_h)
        y += btn_h + gap
        self.landed_btn.setGeometry(x, y, btn_w, btn_h)
        y += btn_h + gap
        self.slam_btn.setGeometry(x, y, btn_w, btn_h)
        y += btn_h + gap
        self.bag_btn.setGeometry(x, y, btn_w, btn_h)

    def closeEvent(self, event):
        # --- Stop rpi_main.py on the Pi FIRST ---
        self._stop_rpi_remote()

        self.vm_timer.stop()
        self.heartbeat_timer.stop()
        self.cam_worker.stop()
        self.motor_worker.stop()
        self.slam_worker.stop()
        self.vm_worker.stop()
        self.rpi_worker.stop()
        self.cam_worker.wait(3000)
        self.motor_worker.wait(3000)
        self.slam_worker.wait(5000)
        self.vm_worker.wait(3000)
        self.rpi_worker.wait(3000)
        event.accept()

    def _stop_rpi_remote(self):
        """SSH into the Pi and kill rpi_main.py."""

        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(hostname=RPI_SSH_HOST, username=RPI_SSH_USER, password=RPI_SSH_PASS, timeout=5)
            ssh.exec_command("pkill -f rpi_main.py")

            time.sleep(1)  # give it a moment to terminate
            ssh.close()
        
        except Exception as e:
            print(f"RPi stop failed: {e}")


if __name__ == "__main__":    
    # 0. Configure windows connection
    pc_network_interface_name = CONFIG["network"]["pc_network_interface_name"]
    pc_network_name = CONFIG["network"]["pc_network_name"]
    pc_ip = CONFIG["network"]["pc_ip"]
    pc_subnet_mask = CONFIG["network"]["pc_subnet_mask"]
    
    print(f"Checking network interface: {pc_network_interface_name}")
    if not is_windows_network_ready(pc_network_interface_name, pc_ip):
        print("Network not ready. Attempting configuration (may require Admin)...")
        setup_windows_network(pc_network_interface_name, pc_network_name, pc_ip, pc_subnet_mask)
        print("Network configuration command sent.")
    else:
        print(f"Windows network interface '{pc_network_interface_name}' is already correctly configured.")

    # 1. Tell Windows this is a unique app to fix the taskbar icon
    # We check if the OS is Windows ('nt') so it does not crash on Mac/Linux
    if os.name == 'nt':
        myappid = 'RoboKedach.Product.PC_Main.1.0'
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)
    
    # 2. Start the application normally
    app = QApplication(sys.argv)
    window = HUDWindow()
    window.show()
    sys.exit(app.exec())
