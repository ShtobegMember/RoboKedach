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

app = Flask(__name__)

# --- Global State ---
camera = None
is_running = True

# Shared frame buffer — written by reader thread, read by web clients
current_frame = None
frame_lock = threading.Lock()


def reset_camera_hardware():
    """
    Force-reset the video device at the driver level.
    Grabs a single frame via v4l2-ctl to unfreeze a stuck ISP
    (Image Signal Processor), clearing 'select() timeout' locks.
    """
    
    if os.path.exists('/dev/video0'):
        print("CAM: Found /dev/video0. Attempting hardware kickstart...")
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", "/dev/video0", "--stream-mmap", "--stream-count=1"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2
            )
            print("CAM: Hardware kickstart successful.")
        except Exception:
            print("CAM: Hardware kickstart skipped (v4l2-ctl not installed or failed).")
    else:
        print("CAM: WARNING! /dev/video0 does not exist.")


def open_camera():
    """
    Open the camera with settings known to work on RPi legacy stack.
    Uses MJPG pixel format at 640x480 to avoid 'select() timeout' errors.
    """

    # Open by device path — avoids index-to-device mismatches
    # when the camera exposes multiple /dev/video* nodes
    cam = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

    if not cam.isOpened():
        return None

    # Set MJPG format BEFORE resolution — order matters for the driver
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # 640x480 is hardware-safe; lower resolutions can crash the driver
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency

    return cam


def release_camera():
    """Release camera resources on program exit."""

    global camera, is_running
    is_running = False
    time.sleep(0.1)  # Give reader thread time to exit its loop

    if camera and camera.isOpened():
        print("CAM: Releasing resource...")
        camera.release()
        camera = None

atexit.register(release_camera)


def camera_reader_worker():
    """
    Dedicated camera reader thread — the ONLY thread that touches the camera.
    Continuously reads frames, processes them, and updates the shared buffer.
    """

    global camera, current_frame, is_running

    while is_running:
        # Reconnect if camera is not available
        if camera is None or not camera.isOpened():
            camera = open_camera()
            if camera is None:
                time.sleep(2)  # Wait before retrying
                continue

        try:
            success, frame = camera.read()
        except cv2.error:
            success = False

        if not success:
            # Read timeout — driver may be stuck, force reconnect
            print("CAM: Read failed (Timeout). Resetting...")
            if camera:
                camera.release()
                camera = None
            time.sleep(1)
            continue

        try:
            # Flip 180° (camera is mounted upside-down)
            frame = cv2.flip(frame, -1)

            # # Draw crosshair overlay at center
            # h, w, _ = frame.shape
            # cx, cy = w // 2, h // 2
            # cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            # cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            # cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)

            # Encode to JPEG and update the shared buffer
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret:
                with frame_lock:
                    current_frame = buffer.tobytes()

        except Exception as e:
            print(f"CAM Logic Error: {e}")
            time.sleep(0.1)


def generate_frames():
    """
    Generator for the MJPEG stream — grabs the latest frame from the shared buffer.
    Yields at ~20 FPS to avoid overloading the web server.
    """

    global is_running

    while is_running:
        frame_data = None

        with frame_lock:
            frame_data = current_frame

        if frame_data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

        time.sleep(0.0333)  # ~30 FPS


@app.route('/')
def video_feed():
    """Flask route serving the MJPEG video stream."""
    
    response = Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return response


def run_server():
    """Initialize camera hardware and start both the reader thread and Flask server."""

    print("CAM: Server Thread Starting...")

    # Reset hardware once on startup to clear any stale driver state
    reset_camera_hardware()
    time.sleep(1)  # Let V4L2 driver fully release before OpenCV opens it

    # Dedicated reader thread — owns the camera exclusively
    reader_thread = threading.Thread(target=camera_reader_worker)
    reader_thread.daemon = True
    reader_thread.start()

    # Flask web server — host 0.0.0.0 to allow access from other machines
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


def start_camera_thread():
    """Start the entire camera server system in a daemon thread."""

    t = threading.Thread(target=run_server)
    t.daemon = True
    t.start()
