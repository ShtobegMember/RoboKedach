"""
hud_window.py - Main GUI Application Window
Constructs the UI layouts, sets up the buttons, handles keyboard teleoperation, 
and manages the floating HUD overlay.
"""

from PyQt6.QtWidgets import QMainWindow, QWidget, QLabel, QPushButton, QApplication
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QIcon

from base_station.gui.hud_painter import HUDPainter

class HUDOverlay(QWidget):
    """Transparent overlay widget that handles all QPainter graphics."""
    def __init__(self, parent_window):
        super().__init__(parent_window)
        self.parent_window = parent_window
        # Make this widget completely transparent to clicks and background
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setStyleSheet("background-color: transparent;")

    def paintEvent(self, event):
        # Pass the parent_window (which holds all the state) to the painter
        HUDPainter.paint_hud(self, self.parent_window)


class HUDWindow(QMainWindow):
    def __init__(self, wsl_manager=None, motor_client=None):
        super().__init__()
        self.wsl_manager = wsl_manager
        self.motor_client = motor_client
        
        self.setWindowTitle("RoboKedach HUD")
        self.setStyleSheet("background-color: black;")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # Position HUD at left 2/3 of the screen
        screen = QApplication.primaryScreen().availableGeometry()
        hud_w = screen.width() * 2 // 3
        self.setGeometry(screen.x(), screen.y(), hud_w, screen.height() - 35)

        # Retrieve Hardware Config limits
        try:
            from core.config_loader import CONFIG
            self.SPEED_MIN = CONFIG["hardware"]["motors"]["speed_min"]
            self.SPEED_MAX = CONFIG["hardware"]["motors"]["speed_max"]
            self.LEG_OFFSETS = CONFIG["hardware"]["motors"]["leg_offsets"]
        except (ImportError, KeyError):
            self.SPEED_MIN = 10
            self.SPEED_MAX = 127
            self.LEG_OFFSETS = [0, 1, 2, 0, 1, 2]

        # System State tracking exactly mirroring original pc_main
        self.vm_data = {'voltage': 0.0, 'current': 0.0}
        self.camera_status = "Disconnected"
        self.vm_status = "Disconnected"
        self.motor_status = "Disconnected"
        
        self.motor_left_speed = self.SPEED_MAX // 2
        self.motor_right_speed = self.SPEED_MAX // 2
        
        self.slam_status = "SLAM: Idle"
        self._slam_live = False
        self._slam_started = False
        self.heading_status = "Heading: Idle"
        self._heading_deg = None
        self._heading_tracking = False

        self.leg_phase_left = 0
        self.leg_phase_right = 0
        self._leg_phase_offset_left = 0
        self._leg_phase_offset_right = 0
        self._held_key_code = None

        self.init_ui()

        # Periodic repaint for HUD
        self.vm_timer = QTimer(self)
        self.vm_timer.timeout.connect(self.overlay.update)
        self.vm_timer.start(100)

    def init_ui(self):
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

        # Buttons
        self.heading_btn = QPushButton("Track Heading", self)
        self.heading_btn.setStyleSheet(btn_style)
        self.heading_btn.clicked.connect(self._on_heading_btn)
        self.heading_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.landed_btn = QPushButton("Landed", self)
        self.landed_btn.setEnabled(False)
        self.landed_btn.setStyleSheet(btn_style)
        self.landed_btn.clicked.connect(self._on_landed_btn)
        self.landed_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.slam_btn = QPushButton("Start SLAM", self)
        self.slam_btn.setEnabled(False)
        self.slam_btn.setStyleSheet(btn_style)
        self.slam_btn.clicked.connect(self._on_slam_btn)
        self.slam_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.bag_btn = QPushButton("Record Bag", self)
        self.bag_btn.setEnabled(False)
        self.bag_btn.setStyleSheet(btn_style)
        self.bag_btn.clicked.connect(self._on_bag_btn)
        self.bag_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    # --- Button Actions ---
    def _on_heading_btn(self):
        if self.motor_client:
            self.motor_client.send_trigger("START_HEADING_TRACK")
        self.heading_btn.setEnabled(False)
        self.heading_btn.setText("Calibrating...")
        self.heading_status = "Heading: Calibrating..."
        self._heading_tracking = True
        self.overlay.update()

    def _on_landed_btn(self):
        if self.motor_client:
            self.motor_client.send_trigger("HEADING_LANDED")
        self.landed_btn.setEnabled(False)
        self.landed_btn.setText("Computing...")
        self.heading_status = "Heading: Computing..."
        self.overlay.update()

    def _on_slam_btn(self):
        print(f"DEBUG: wsl_manager object is: {self.wsl_manager}")
        if self.motor_client:
            self.motor_client.send_trigger("START_SLAM")
        if self.wsl_manager:
            self.wsl_manager.start_core_slam_nodes()
        self._slam_started = True
        self.slam_btn.setEnabled(False)
        self.slam_btn.setText("SLAM Active")
        self.bag_btn.setEnabled(True)

    def _on_bag_btn(self):
        if not self.wsl_manager: return
        if self._slam_live:
            self.wsl_manager.toggle_rosbag(False)
            self._slam_live = False
            self.bag_btn.setText("Record Bag")
        else:
            self.wsl_manager.toggle_rosbag(True)
            self._slam_live = True
            self.bag_btn.setText("Stop Bag")

    def show_popup(self, text):
        self.popup_label.setText(text)
        self.popup_label.adjustSize()
        pw, ph = self.popup_label.width(), self.popup_label.height()
        self.popup_label.setGeometry((self.width() - pw) // 2, (self.height() - ph) // 2, pw, ph)
        self.popup_label.show()
        self.popup_label.raise_()
        self.popup_timer.start(1500)

    # --- Signal Handlers (Called by pc_main) ---
    def on_heading_calibrated(self):
        """Called when the IMU finishes Phase 1 calibration."""
        self.heading_status = "Heading: Tracking descent..."
        self.heading_btn.setText("Tracking...")
        self.landed_btn.setEnabled(True)  # <--- Unlocks the Landed button

    def on_heading_received(self, heading_deg: float):
        """Called when the robot lands and the final heading is computed."""
        self._heading_deg = heading_deg
        self._heading_tracking = False
        self.heading_status = f"Heading: {heading_deg:+.2f} deg"
        
        self.landed_btn.setText("Done")
        self.landed_btn.setEnabled(False)
        
        # --- RESTORED: Publish the TF before starting SLAM ---
        if self.wsl_manager:
            self.wsl_manager.publish_north_tf(heading_deg)
            
        # Enables SLAM now that we have our orientation map transform
        self.slam_btn.setEnabled(True)
        self.overlay.update()

    def update_frame(self, image: QImage):
        pixmap = QPixmap.fromImage(image)
        # Explicitly copy the image to prevent 0xc0000005 access violations
        # if the background worker reuses the underlying buffer.
        pixmap = QPixmap.fromImage(image.copy())
        scaled = pixmap.scaled(
            self.camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled)

    def update_telemetry(self, voltage: float, current: float):
        self.vm_data['voltage'] = voltage
        self.vm_data['current'] = current
        
        if 0 < voltage <= 10.8:
            if not self.bat_warn_label.isVisible():
                self.bat_warn_label.adjustSize()
                pw, ph = self.bat_warn_label.width(), self.bat_warn_label.height()
                self.bat_warn_label.setGeometry((self.width() - pw) // 2, (self.height() - ph) // 2, pw, ph)
                self.bat_warn_label.show()
                self.bat_warn_label.raise_()
        else:
            self.bat_warn_label.hide()

    def update_connection_status(self, system: str, is_connected: bool):
        status_str = "Connected" if is_connected else "Disconnected"
        if system == "CAM":
            self.camera_status = status_str
        elif system == "SYS":
            self.vm_status = status_str
        elif system == "MOT":
            self.motor_status = status_str

    def update_motor_speed(self, left: int, right: int):
        """Update speed values for the differential speed bar panel."""
        self.motor_left_speed = left
        self.motor_right_speed = right

    def update_encoders(self, enc1: int, enc2: int):
        """Update leg animation phases based on encoder ticks."""
        # 8400 ticks / 4 quarters = 2100 ticks per quarter
        raw_left = ((enc1 // 2100) + 1) % 4
        raw_right = ((enc2 // 2100) + 1) % 4
        self.leg_phase_left = (raw_left + self._leg_phase_offset_left) % 4
        self.leg_phase_right = (raw_right + self._leg_phase_offset_right) % 4

    # --- Events ---
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        
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
        
    def keyPressEvent(self, event):
        if event.isAutoRepeat() or not self.motor_client: return
        key = event.key()
        
        cmd_sent = False
        if key in (Qt.Key.Key_W, Qt.Key.Key_Up): 
            self.motor_client.send_movement(1.0, 1.0)
            cmd_sent = True
        elif key in (Qt.Key.Key_S, Qt.Key.Key_Down): 
            self.motor_client.send_movement(-1.0, -1.0)
            cmd_sent = True
        elif key in (Qt.Key.Key_A, Qt.Key.Key_Left): 
            self.motor_client.send_movement(-1.0, 1.0) 
            cmd_sent = True
        elif key in (Qt.Key.Key_D, Qt.Key.Key_Right): 
            self.motor_client.send_movement(1.0, -1.0)
            cmd_sent = True
        elif key == Qt.Key.Key_Space:
            self.motor_client.send_movement(0.0, 0.0)
            self._held_key_code = None
        elif key == Qt.Key.Key_R:
            self._leg_phase_offset_left = self.leg_phase_left
            self._leg_phase_offset_right = self.leg_phase_right
            self.show_popup("ENCODERS RESET")

        if cmd_sent:
            self._held_key_code = key

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or not self.motor_client: return
        if event.key() == self._held_key_code:
            self._held_key_code = None
            self.motor_client.send_movement(0.0, 0.0)