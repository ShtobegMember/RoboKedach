import cv2
import threading
import time
from flask import Flask, Response

# --- CONFIGURATION ---
FLIP_CAMERA = True
SHOW_CROSSHAIR = True
STREAM_PORT = 5000

app = Flask(__name__)

# --- CAMERA SETUP ---
print("?? Attempting to open camera...")
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Verify Camera
if not camera.isOpened():
    print("? FATAL: Camera failed to open on index 0!")
    exit() # Stop script immediately if camera is missing

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
camera.set(cv2.CAP_PROP_FPS, 30)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            print("? ERROR: Camera read failed! (Is the cable loose?)")
            time.sleep(1) # Wait a bit before retrying so we don't spam logs
            continue # Try again instead of breaking
        
        # If we get here, the camera is working
        if FLIP_CAMERA:
            frame = cv2.flip(frame, -1)

        if SHOW_CROSSHAIR:
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_flask():
    app.run(host='0.0.0.0', port=STREAM_PORT, debug=False, use_reloader=False)

if __name__ == "__main__":
    t = threading.Thread(target=start_flask)
    t.daemon = True
    t.start()

    print(f"? Server Running at http://<YOUR_IP>:{STREAM_PORT}")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        camera.release()
