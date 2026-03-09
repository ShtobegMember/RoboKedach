"""
camera_server.py
----------------
Aggressive Camera Driver with Broadcaster Pattern.
Fixes deadlocks and allows multiple simultaneous viewers safely.
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
is_running = True

# Variables for the Broadcaster pattern
current_frame = None
frame_lock = threading.Lock()  # Protects the current_frame variable, NOT the camera


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
    """
    Cleans up the camera resource when the program exits.
    """
    global camera, is_running
    is_running = False  # Tell the reader thread to stop
    time.sleep(0.1)  # Give the reader thread time to see the flag and exit its loop
    
    if camera and camera.isOpened():
        print("CAM: Releasing resource...")
        camera.release()
        camera = None

atexit.register(release_camera)


def camera_reader_worker():
    """
    This thread OWNS the camera. It constantly reads and updates current_frame.
    """
    global camera, current_frame, is_running

    while is_running:
        if camera is None or not camera.isOpened():
            camera = open_camera()
            if camera is None:
                # If open fails, wait 2s before hammering the driver again
                time.sleep(2)
                continue

        # Attempt Read (Only this thread ever reads, so it's perfectly safe)
        success, frame = camera.read()

        if not success:
            # If read times out, the driver might be stuck.
            print("CAM: Read failed (Timeout). Resetting...")
            if camera:
                camera.release()
                camera = None
            time.sleep(1) # Cooldown
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
            if ret:
                # Quickly swap the global frame so web clients can see it
                with frame_lock:
                    current_frame = buffer.tobytes()

        except Exception as e:
            print(f"CAM Logic Error: {e}")
            time.sleep(0.1)


def generate_frames():
    """
    Web clients call this. It just grabs the latest pre-processed frame instantly.
    """
    global is_running
    
    while is_running:
        frame_data = None
        
        # Grab the latest picture from the broadcaster safely
        with frame_lock:
            frame_data = current_frame

        if frame_data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
        
        # Stream at ~20 FPS. This prevents the web server from overloading the CPU
        time.sleep(0.05)


@app.route('/')
def video_feed():
    """
    Flask route to serve the video stream.
    """
    response = Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return response


def run_server():
    """
    Initializes hardware and starts the background threads.
    """
    print("CAM: Server Thread Starting...")
    
    # Run the hardware reset once on startup
    reset_camera_hardware()

    # Start the dedicated camera reading thread
    reader_thread = threading.Thread(target=camera_reader_worker)
    reader_thread.daemon = True
    reader_thread.start()

    # Start Flask Web Server
    # Host 0.0.0.0 is required to see it from another PC
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


def start_camera_thread():
    """
    Starts the entire camera server system in a daemon thread.
    """
    t = threading.Thread(target=run_server)
    t.daemon = True
    t.start()
