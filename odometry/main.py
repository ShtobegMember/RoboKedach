"""
main.py
-------
Orchestrator script with live IMU integration and 2D Odometry (X, Y).
"""

import time
import sys
import math  # Needed for cos/sin calculations

from camera_server import start_camera_thread
from imu_thread import IMUThread
from robot_controller import RobotConfig, RobotInterface


def main():
    # ---------------------------------------------------------
    # 1. Start Camera (Thread 1)
    # ---------------------------------------------------------

    start_camera_thread()

    # Give the camera a moment to warm up
    print("Waiting for camera warmup...")
    time.sleep(3)

    # ---------------------------------------------------------
    # 2. Start IMU (Thread 2)
    # ---------------------------------------------------------

    imu_thread = IMUThread()

    # We initialize/calibrate BEFORE starting the thread loop,
    # so we know the robot is ready before we let the user move it.
    try:
        imu_thread.initialize_and_calibrate()

    except Exception as e:
        print(f"Failed to initialize IMU: {e}")
        sys.exit(1)

    # Start the continuous calculation loop
    imu_thread.start()

    # ---------------------------------------------------------
    # 3. Start Movement Interface (Main Thread)
    # ---------------------------------------------------------

    print("\nStarting Robot Control Interface...")

    try:
        config = RobotConfig()
        interface = RobotInterface(config)

        # =========================================================
        # ODOMETRY LOGIC (X, Y TRACKING)
        # =========================================================

        # State to track position (Starts at 0,0)
        robot_pos = {'x': 0.0, 'y': 0.0}

        # Save the original method so we can call it later
        original_handle_command = interface.handle_command

        def new_handle_command(key):
            """
            Wrapper to inject Odometry logic:
            1. Detect Forward command.
            2. Measure Yaw BEFORE and AFTER movement.
            3. Calculate Average Yaw.
            4. Update X, Y based on STEP_SIZE.
            """

            # Check for FORWARD command (Up Arrow)
            if key == '\x1b[A':
                # 1. Save initial Yaw (Index 2 is Yaw)
                _, _, yaw_initial = imu_thread.get_angles()

                # 2. Perform the actual movement (Blocking call)
                original_handle_command(key)

                # 3. Save the final Yaw
                _, _, yaw_final = imu_thread.get_angles()

                # 4. Calculate Average Yaw
                # Since IMU thread accumulates angles (doesn't wrap 360->0 immediately),
                # simple averaging is safe here.
                yaw_avg = (yaw_initial + yaw_final) / 2.0

                # 5. Update X, Y
                # Convert degrees to radians for math functions
                yaw_rad = math.radians(yaw_avg)
                dist = interface.STEP_SIZE

                # Standard 2D Kinematics:
                # New_X = Old_X + (dist * cos(theta))
                # New_Y = Old_Y + (dist * sin(theta))
                robot_pos['x'] += dist * math.cos(yaw_rad)
                robot_pos['y'] += dist * math.sin(yaw_rad)

            else:
                # For all other keys (Turn, Backward, etc.), just move without updating X/Y
                original_handle_command(key)

        # Apply the Monkey Patch
        interface.handle_command = new_handle_command

        # =========================================================
        # CUSTOM DISPLAY LOGIC
        # =========================================================

        # A. Override the 's' key Status Display
        original_display_status = interface.display_status

        def custom_status_display():
            original_display_status()  # Print encoders/speed
            r, p, y = imu_thread.get_angles()

            print(f"📐 IMU: Roll={r:.1f}° | Pitch={p:.3f}° | Yaw={y:.3f}°")
            print(f"📍 Pos: X={robot_pos['x']:.2f} | Y={robot_pos['y']:.2f}")
            print("-" * 20 + "\n")

        interface.display_status = custom_status_display

        # B. Override the Live Loop Line ("Ready..." text)
        def custom_live_line():
            # This runs inside the while loop, updating constantly
            r, p, y = imu_thread.get_angles()
            spd = interface.motor_ctrl.current_speed

            # Step counter from interface
            length = interface.step * interface.STEP_SIZE

            # Format: Speed | Length | X, Y | Yaw | Ready
            return (f"⚡ Spd:{spd} | Len:{length:.3f} | "
                    f"XY:({robot_pos['x']:.3f}, {robot_pos['y']:.3f}) | "
                    f"📐 Yaw:{y:5.1f}° | Ready...\n")

        interface._get_status_line = custom_live_line

        # =========================================================

        # Blocking call - this runs until the user hits 'q'
        interface.run()

    except Exception as e:
        print(f"Error in Main Control Loop: {e}")

    finally:
        print("\nShutting down threads...")
        imu_thread.stop()
        print("Done.")


if __name__ == "__main__":
    main()
