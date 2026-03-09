"""
imu_thread.py
-------------
Background thread for continuous IMU angle integration.
"""

import threading
import time
import numpy as np
from imu_driver import LSM6DSV16X, imu_to_global_coordinates


class IMUThread(threading.Thread):
    """
    Thread class that continually polls the IMU, integrates the gyro rates,
    and updates a shared 'angles' variable.
    """

    def __init__(self):
        super().__init__()
        self.daemon = True  # Daemon threads close when the main program closes
        self.running = True

        # Shared State: [Roll, Pitch, Yaw] in radians
        self.angles = np.zeros(3)
        self.lock = threading.Lock()  # Optional lock for thread safety

        # Driver & Calibration storage
        self.imu = LSM6DSV16X()
        self.gyro_offsets = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}

    def initialize_and_calibrate(self, samples=100):
        """
        Initialize the sensor hardware and run the calibration routine.
        This is a BLOCKING call and should be run before starting the thread.
        """

        # 1. Initialize Sensor
        self.imu.initialize()

        # 2. Calibrate Gyro
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

            # Sleep to match sensor speed (approx)
            time.sleep(0.005)

        # Average using the ACTUAL count
        self.gyro_offsets['gx'] = offsets['gx'] / valid_samples
        self.gyro_offsets['gy'] = offsets['gy'] / valid_samples
        self.gyro_offsets['gz'] = offsets['gz'] / valid_samples

        print(f"Calibration Complete. Offsets: {self.gyro_offsets}")

    def get_angles(self):
        """
        Returns the current angles in DEGREES.
        This is safe to call from the main thread.
        """

        with self.lock:
            # Return a copy to ensure we don't return a reference to a changing array
            return np.degrees(self.angles).copy()

    def run(self):
        """
        The main thread loop.
        Continuously reads data, applies calibration, and integrates angles.
        """

        print("IMU Thread Started...")
        last_time = None

        while self.running:
            try:
                # Get the latest data
                sensor_data = self.imu.get_data()

                if sensor_data:
                    # Calculate dt only when we actually have a sample
                    curr_time = time.time()

                    # First valid sample: just establish baseline, skip integration
                    if last_time is None:
                        last_time = curr_time
                        time.sleep(0.005)
                        continue

                    dt = curr_time - last_time
                    last_time = curr_time

                    # Subtract the offset before processing
                    sensor_data['gx'] -= self.gyro_offsets['gx']
                    sensor_data['gy'] -= self.gyro_offsets['gy']
                    sensor_data['gz'] -= self.gyro_offsets['gz']

                    # --- Process Robotics Logic ---

                    # 1. Extract Local Gyro Rates (Body Frame)
                    gyro_body = np.array([sensor_data['gx'], sensor_data['gy'], sensor_data['gz']])

                    with self.lock:
                        # 2. Get Rotation Matrix (Body -> Global)
                        # Use the CURRENT angles to calculate how the robot is oriented right now.
                        R = imu_to_global_coordinates(self.angles)

                        # 3. Transform Rates to Global Frame,
                        # Rotate the velocity vector from the robot's to the world's perspective.
                        gyro_global = R @ gyro_body

                        # 4. Integrate Global Rates
                        # Add the change to our global Euler angles.
                        self.angles += gyro_global * dt

                # Short sleep to prevent CPU hogging
                # ODR is 120Hz (~8.3ms), so 5ms sleep is safe
                time.sleep(0.008)

            except Exception as e:
                print(f"IMU Thread Error: {e}")

        # Cleanup
        self.imu.close()
        print("IMU Thread Stopped.")

    def stop(self):
        """
        Signal the thread to stop and wait for it to finish.
        """

        self.running = False
        self.join(timeout=2)  # Wait up to 2 seconds for the thread to finish
