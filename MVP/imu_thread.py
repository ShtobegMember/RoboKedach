"""
imu_thread.py - Background thread for continuous IMU angle integration.
Polls the gyroscope, transforms rates to global frame, and accumulates Euler angles.
"""

import threading
import time
import numpy as np
from imu_driver import LSM6DSV16X, imu_to_global_coordinates


class IMUThread(threading.Thread):
    """Daemon thread that continuously integrates gyro rates into roll/pitch/yaw."""

    def __init__(self):
        super().__init__()
        self.daemon = True
        self.running = True

        # Shared state: [roll, pitch, yaw] in radians
        self.angles = np.zeros(3)
        self.lock = threading.Lock()

        self.imu = LSM6DSV16X()
        self.gyro_offsets = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}

    def initialize_and_calibrate(self, samples=100):
        """
        Initialize sensor and compute gyro bias offsets.
        Blocking call — run before starting the thread.
        """
        self.imu.initialize()

        print("Calibrating Gyro... Keep robot still!")
        offsets = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
        valid_samples = 0

        while valid_samples < samples:
            data = self.imu.get_data()
            if data:
                offsets['gx'] += data['gx']
                offsets['gy'] += data['gy']
                offsets['gz'] += data['gz']
                valid_samples += 1

            time.sleep(0.005)

        self.gyro_offsets['gx'] = offsets['gx'] / valid_samples
        self.gyro_offsets['gy'] = offsets['gy'] / valid_samples
        self.gyro_offsets['gz'] = offsets['gz'] / valid_samples

        print(f"Calibration Complete. Offsets: {self.gyro_offsets}")

    def get_angles(self):
        """Return current angles in degrees (thread-safe copy)."""
        with self.lock:
            return np.degrees(self.angles).copy()

    def run(self):
        """Integration loop: read gyro, transform to global frame, accumulate angles."""
        print("IMU Thread Started...")
        last_time = None

        while self.running:
            try:
                sensor_data = self.imu.get_data()

                if sensor_data:
                    curr_time = time.time()

                    # Skip first sample — just establish the time baseline
                    if last_time is None:
                        last_time = curr_time
                        time.sleep(0.005)
                        continue

                    dt = curr_time - last_time
                    last_time = curr_time

                    # Remove bias
                    sensor_data['gx'] -= self.gyro_offsets['gx']
                    sensor_data['gy'] -= self.gyro_offsets['gy']
                    sensor_data['gz'] -= self.gyro_offsets['gz']

                    gyro_body = np.array([sensor_data['gx'], sensor_data['gy'], sensor_data['gz']])

                    with self.lock:
                        # Rotate body-frame rates to global frame using current orientation
                        R = imu_to_global_coordinates(self.angles)
                        gyro_global = R @ gyro_body
                        self.angles += gyro_global * dt

                # ODR is 120Hz (~8.3ms), sleep 8ms to avoid busy-waiting
                time.sleep(0.008)

            except Exception as e:
                print(f"IMU Thread Error: {e}")

        self.imu.close()
        print("IMU Thread Stopped.")

    def stop(self):
        """Signal the thread to stop and wait for cleanup."""
        self.running = False
        self.join(timeout=2)
