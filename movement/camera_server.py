import cv2
import threading
from flask import Flask, Response

# Configuration
app = Flask(__name__)
camera = None

def generate_frames():
    global camera
    while True:
        success, frame = camera.read()
        if not success: 
            break
        
        # Logic: Flip + Crosshair
        frame = cv2.flip(frame, -1)
        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 2, (0, 0, 255), -1)

        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_server():
    global camera
    # Initialize Camera
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    # --- INSTANT START FIXES ---
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # Keep only the newest frame
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    camera.set(cv2.CAP_PROP_FPS, 30)
    
    # Warm-up: clear the first frame immediately
    camera.read()

    # Run Flask (blocks this thread)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def start_camera_thread():
    """Call this function to start the camera in the background"""
    t = threading.Thread(target=run_server)
    t.daemon = True
    t.start()
    print("?? Camera Server Started on port 5000")
