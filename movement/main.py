import time
from camera_server import start_camera_thread
from robot_movement import main as run_robot_logic

# 1. Start the Camera (Background)
start_camera_thread()

# Give it a second to initialize
time.sleep(2)

# 2. Start the Robot Control (Foreground/Main Thread)
print("?? Starting Robot Interface...")
run_robot_logic()
