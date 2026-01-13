"""
camera_server.py
----------------
Aggressive Camera Driver.
Fixes 'select() timeout' by forcing hardware-supported resolutions (640x480)
and MJPEG compression.
"""

import cv2
import threading
import time
import os
import atexit
import subprocess
from flask import Flask, Response


app = Flask(__name__)

# --- GLOBAL STATE ---
camera = None
lock = threading.Lock()
is_running = True


def reset_camera_hardware():
    """
    Nuclear option: Uses system tools to reset the video device state
    before OpenCV tries to touch it. This clears 'select() timeout' locks.
    """

    if os.path.exists('/dev/video0'):
        print("CAM: Found /dev/video0. Attempting hardware kickstart...")

        try:
            # This 'jogs' the driver by grabbing 1 frame at the driver level
            # It often unfreezes a stuck ISP (Image Signal Processor).
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
    Opens the camera with specific, hard-coded settings that are known
    to work on Raspberry Pi legacy stack.
    """
    # 1. Force index 0. Do not guess.
    idx = 0

    # 2. Open with V4L2 backend
    cam = cv2.VideoCapture(idx, cv2.CAP_V4L2)

    if not cam.isOpened():
        return None

    # 3. CRITICAL: Set Pixel Format to MJPG *BEFORE* setting resolution
    # This fixes the 'select() timeout' by using compressed video
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # 4. CRITICAL: Set Resolution to 640x480
    # Your diagnostic proved 640x480 works. 320x240 likely caused the crash.
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 5. Set FPS
    cam.set(cv2.CAP_PROP_FPS, 30)

    # 6. Buffer size
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    return cam


def release_camera():
    global camera
    with lock:
        if camera and camera.isOpened():
            print("CAM: Releasing resource...")
            camera.release()
            camera = None


atexit.register(release_camera)


def generate_frames():
    global camera

    while is_running:
        with lock:
            if camera is None or not camera.isOpened():
                camera = open_camera()
                if camera is None:
                    # If open fails, wait 2s before hammering the driver again
                    time.sleep(2)
                    continue

            # Attempt Read
            success, frame = camera.read()

        if not success:
            # If read times out, the driver might be stuck.
            print("CAM: Read failed (Timeout). Resetting...")
            with lock:
                if camera:
                    camera.release()
                    camera = None
            time.sleep(1)   # Cooldown
            continue

        try:
            # 1. Flip (Optional: Change to 0 or 1 if upside down)
            frame = cv2.flip(frame, -1)

            # 2. Resize output for bandwidth (Software scaling is safer than Hardware scaling)
            # We capture at 640x480 (Hardware safe) -> Resize to 320x240 (Bandwidth safe)
            frame = cv2.resize(frame, (320, 240))

            # 3. Overlay
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)

            # 4. Encode
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if not ret:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        except Exception as e:
            print(f"CAM Logic Error: {e}")
            time.sleep(0.1)


@app.route('/')
def video_feed():
    response = Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return response


def run_server():
    global camera
    print("CAM: Server Thread Starting...")

    # Run the hardware reset once on startup
    reset_camera_hardware()

    with lock:
        camera = open_camera()
        if camera and camera.isOpened():
            print("CAM: Camera Initialized Successfully (640x480 MJPG)")
        else:
            print("CAM: Warning - Camera open failed on startup. Will retry in loop.")

    # Host 0.0.0.0 is required to see it from another PC
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


def start_camera_thread():
    t = threading.Thread(target=run_server)
    t.daemon = True
    t.start()
