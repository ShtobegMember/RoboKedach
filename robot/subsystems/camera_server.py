"""
camera_server.py - MJPEG camera streaming server with broadcaster pattern.
A dedicated reader thread owns the camera and updates a shared frame buffer.
Web clients grab the latest frame without touching the camera directly,
preventing deadlocks and allowing multiple simultaneous viewers.
"""

import cv2
import threading
import time
import os
import atexit
import subprocess
from flask import Flask, Response

from core.config_loader import CONFIG

app = Flask(__name__)

# --- Global State ---
camera = None
is_running = True

# Shared frame buffer — written by reader thread, read by web clients
current_frame = None
frame_lock = threading.Lock()


def reset_camera_hardware():
    """
    Force-reset the video device at the driver level using config parameters.
    Grabs a single frame via v4l2-ctl to unfreeze a stuck ISP
    (Image Signal Processor), clearing 'select() timeout' locks.
    """
    # 1. Load values from config
    cam_cfg = CONFIG.get("hardware", {}).get("camera", {})
    dev_node = cam_cfg.get("device_node", "/dev/video0")
    width = cam_cfg.get("width", 640)
    height = cam_cfg.get("height", 480)
    fmt = cam_cfg.get("pixelformat", "MJPG")
    
    if os.path.exists(dev_node):
        print(f"CAM: Found {dev_node}. Attempting hardware kickstart...")
        try:
            # 2. Inject dynamically into the v4l2-ctl command
            format_str = f"width={width},height={height},pixelformat={fmt}"
            subprocess.run(
                [
                    "v4l2-ctl", "-d", dev_node, 
                    f"--set-fmt-video={format_str}", 
                    "--stream-mmap", "--stream-count=1"
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2
            )
            print("CAM: Hardware kickstart successful.")
        except Exception as e:
            print(f"CAM: Hardware kickstart failed: {e}")
    else:
        print(f"CAM: Device {dev_node} not found. Hardware disconnected?")


def camera_reader_worker():
    """Continuously reads frames from the camera into a shared buffer."""
    global current_frame, is_running
    
    # Dynamically pull the device node and resolution
    cam_cfg = CONFIG.get("hardware", {}).get("camera", {})
    dev_node = cam_cfg.get("device_node", "/dev/video0")
    width = cam_cfg.get("width", 640)
    height = cam_cfg.get("height", 480)
    
    cap = cv2.VideoCapture(dev_node)
    
    # Try to set MJPG to prevent raw uncompressed stream timeouts
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        print(f"CAM: Fatal error. Cannot open {dev_node}.")
        is_running = False
        return

    print(f"CAM: Camera opened successfully on {dev_node}. Starting stream...")
    
    while is_running:
        ret, frame = cap.read()
        if not ret:
            print("CAM: Read failed (Timeout). Resetting...")
            cap.release()
            time.sleep(1)
            reset_camera_hardware()
            cap = cv2.VideoCapture(dev_node)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            continue
            
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ret:
            with frame_lock:
                current_frame = buffer.tobytes()

    cap.release()
    print("CAM: Reader thread stopped.")


def generate_frames():
    """Generator for the Flask streaming response."""
    global current_frame, is_running

    while is_running:
        frame_data = None

        with frame_lock:
            frame_data = current_frame

        if frame_data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

        time.sleep(0.0333)  # ~30 FPS throttle to save CPU


@app.route('/')
def video_feed():
    """Flask route serving the MJPEG video stream."""
    response = Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return response


def run_server(port=5000):
    """Initialize camera hardware and start both the reader thread and Flask server."""
    print("CAM: Server Thread Starting...")

    # Reset hardware once on startup to clear any stale driver state
    reset_camera_hardware()
    time.sleep(1)  # Let V4L2 driver fully release before OpenCV opens it

    # Dedicated reader thread — owns the camera exclusively
    reader_thread = threading.Thread(target=camera_reader_worker)
    reader_thread.daemon = True
    reader_thread.start()

    # Flask web server — host 0.0.0.0 to allow access from PC
    # Running with threaded=True to handle multiple connections
    # use_reloader=False prevents Flask from spinning up duplicate processes
    actual_port = CONFIG.get("network", {}).get("camera_port", port)
    app.run(host='0.0.0.0', port=actual_port, threaded=True, use_reloader=False)