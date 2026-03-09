"""
main.py - System orchestrator with 2D odometry tracking.
Starts camera, IMU, web plotter, and robot control interface.
"""

import time
import sys
import math

from camera_server import start_camera_thread
from imu_thread import IMUThread
from robot_controller import RobotConfig, RobotInterface
from web_plotter import WebPlotter


def main():
    # ---------------------------------------------------------
    # 1. Start Camera (daemon thread)
    # ---------------------------------------------------------

    start_camera_thread()

    # Allow the camera hardware to warm up before proceeding
    print("Waiting for camera warmup...")
    time.sleep(1)

    # ---------------------------------------------------------
    # 2. Start IMU (daemon thread)
    # ---------------------------------------------------------

    imu_thread = IMUThread()

    # Initialize and calibrate while robot is still — must complete
    # before starting the integration thread
    try:
        imu_thread.initialize_and_calibrate()

    except Exception as e:
        print(f"Failed to initialize IMU: {e}")
        sys.exit(1)

    # Begin the continuous angle integration loop
    imu_thread.start()

    # ---------------------------------------------------------
    # 3. Start Web Plotter (daemon thread, port 5001)
    # ---------------------------------------------------------

    print("Initializing Web Plotter...")
    plotter = WebPlotter(port=5001)

    # ---------------------------------------------------------
    # 4. Start Movement Interface (runs on main thread)
    # ---------------------------------------------------------

    print("\nStarting Robot Control Interface...")

    try:
        config = RobotConfig()
        interface = RobotInterface(config)

        # =========================================================
        # ODOMETRY — X, Y Position Tracking
        # =========================================================
        # After each forward movement, the robot's heading (yaw) and
        # step distance are used to update its 2D position.

        robot_pos = {'x': 0.0, 'y': 0.0}

        # Store original method for use inside the wrapper
        original_handle_command = interface.handle_command

        def new_handle_command(key):
            """
            Wraps handle_command to inject odometry logic:
            1. Capture yaw before movement
            2. Execute the original movement command
            3. Capture yaw after movement
            4. Average the two yaw readings for accuracy
            5. Update X, Y using 2D kinematics
            """

            # Map of forward keys to their step fraction
            forward_keys = {
                '\x1b[A': 1.0,       # Up Arrow — full step
                '\x1b[1;2A': 0.25,   # Shift+Up — quarter step
            }

            if key in forward_keys:
                fraction = forward_keys[key]

                # 1. Record yaw before movement (index 2 = yaw)
                _, _, yaw_initial = imu_thread.get_angles()

                # 2. Execute the actual motor movement (blocking)
                original_handle_command(key)

                # 3. Record yaw after movement
                _, _, yaw_final = imu_thread.get_angles()

                # 4. Average yaw for better accuracy during the movement.
                #    Safe because IMU accumulates continuously without
                #    wrapping at 360°.
                yaw_avg = (yaw_initial + yaw_final) / 2.0

                # 5. 2D kinematics update:
                #    X += distance * cos(yaw)
                #    Y += distance * sin(yaw)
                yaw_rad = math.radians(yaw_avg)
                dist = interface.STEP_SIZE * fraction

                robot_pos['x'] += dist * math.cos(yaw_rad)
                robot_pos['y'] += dist * math.sin(yaw_rad)

                plotter.update(robot_pos['x'], robot_pos['y'])

            else:
                # Non-forward commands (turn, backward, etc.) — no odometry
                original_handle_command(key)

        # Monkey-patch the interface to use our odometry-aware version
        interface.handle_command = new_handle_command

        # =========================================================
        # CUSTOM DISPLAY OVERRIDES
        # =========================================================

        # A. Override 's' key status display — add IMU and position info
        original_display_status = interface.display_status

        def custom_status_display():
            original_display_status()  # Print encoder/speed info
            r, p, y = imu_thread.get_angles()

            print(f"📐 IMU: Roll={r:.1f}° | Pitch={p:.3f}° | Yaw={y:.3f}°")
            print(f"📍 Pos: X={robot_pos['x']:.2f} | Y={robot_pos['y']:.2f}")
            print("-" * 20 + "\n")

        interface.display_status = custom_status_display

        # B. Override the idle-loop status line with full telemetry
        def custom_live_line():
            r, p, y = imu_thread.get_angles()
            spd = interface.motor_ctrl.current_speed

            # Total distance from step count
            length = interface.step * interface.STEP_SIZE

            return (f"⚡ Spd:{spd} | Len:{length:.3f} | "
                    f"XY:({robot_pos['x']:.3f}, {robot_pos['y']:.3f}) | "
                    f"📐 Yaw:{y:5.1f}° | Ready...\n")

        interface.get_status_line = custom_live_line

        # ---------------------------------------------------------
        # System Ready Banner
        # ---------------------------------------------------------
        print("\n\n\n\n" + "=" * 40)
        print("🚀 SYSTEM READY")
        print("=" * 40)
        print(f"🎥 Camera Stream: http://127.0.0.1:5000")
        print(f"🗺️ Live Plotter:  http://127.0.0.1:5001")
        print("-" * 40)
        print("Controls active. Press 'q' to quit.")
        print("=" * 40)

        # Blocking call — runs until user presses 'q'
        interface.run()

    except Exception as e:
        print(f"Error in Main Control Loop: {e}")

    finally:
        print("\nShutting down threads...")
        imu_thread.stop()
        print("Done.")


if __name__ == "__main__":
    main()
