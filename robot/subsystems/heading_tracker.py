"""
heading_tracker.py - Pre-SLAM heading tracker using LSM6DSV16X IMU.

Uses a Mahony AHRS filter (gyro + accelerometer fusion) to track 3D orientation
during vertical pipe descent. Extracts heading (yaw around gravity) as the
cumulative rotation the robot underwent.
"""

import math
import time
import numpy as np
import multiprocessing

from core.config_loader import CONFIG
from robot.hardware.lsm6dsv16x import LSM6DSV16X

class MahonyAHRS:
    """Mahony filter for IMU attitude estimation."""
    def __init__(self, sample_freq: float, kp: float = 1.0, ki: float = 0.0):
        self.sample_freq = sample_freq
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]
        self.e_int = np.array([0.0, 0.0, 0.0])   # Integral error

    def update_imu(self, gx: float, gy: float, gz: float, ax: float, ay: float, az: float):
        """Update the filter with new gyro (rad/s) and accel (m/s^2) data."""
        # Normalize accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0: return
        ax, ay, az = ax / norm, ay / norm, az / norm

        # Estimated direction of gravity
        vx = 2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])
        vy = 2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3])
        vz = self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]

        # Error is cross product between estimated direction and measured direction of gravity
        ex, ey, ez = (ay * vz - az * vy), (az * vx - ax * vz), (ax * vy - ay * vx)

        if self.ki > 0.0:
            self.e_int += np.array([ex, ey, ez])
        else:
            self.e_int = np.array([0.0, 0.0, 0.0])

        gx += self.kp * ex + self.ki * self.e_int[0]
        gy += self.kp * ey + self.ki * self.e_int[1]
        gz += self.kp * ez + self.ki * self.e_int[2]

        # Integrate rate of change of quaternion
        pa, pb, pc = self.q[0], self.q[1], self.q[2]
        gx *= (0.5 * (1.0 / self.sample_freq))
        gy *= (0.5 * (1.0 / self.sample_freq))
        gz *= (0.5 * (1.0 / self.sample_freq))

        self.q[0] += (-pb * gx - pc * gy - self.q[3] * gz)
        self.q[1] += (pa * gx + pc * gz - self.q[3] * gy)
        self.q[2] += (pa * gy - pb * gz + self.q[3] * gx)
        self.q[3] += (pa * gz + pb * gy - pc * gx)

        # Normalize quaternion
        self.q /= math.sqrt(sum(self.q**2))

    def get_yaw(self) -> float:
        """Extract yaw from quaternion in degrees."""
        yaw = math.atan2(2.0 * (self.q[0] * self.q[3] + self.q[1] * self.q[2]),
                         1.0 - 2.0 * (self.q[2] * self.q[2] + self.q[3] * self.q[3]))
        return math.degrees(yaw)


class LSM6DSV16X_HeadingTracker:
    def __init__(self, bus_num: int = 1, address: int = 0x6B):
        self.hw = LSM6DSV16X(bus_num, address)
        self.ahrs = MahonyAHRS(sample_freq=120.0, kp=2.5, ki=0.0) # High Kp for quick init
        self.is_tracking = False

    def initialize_sensor(self):
        self.hw.initialize_sensor()
        
    def calibrate(self, num_samples: int = 500):
        """Phase 1: Stationary calibration (High Gains)."""
        print("TRACKER: Calibrating Mahony filter...")
        self.ahrs.kp = 2.5
        for _ in range(num_samples):
            gyro_deg, accel = self.hw.read_sensor_data()
            gyro_rad = np.radians(gyro_deg)
            self.ahrs.update_imu(*gyro_rad, *accel)
            time.sleep(1.0 / 120.0)

    def start_tracking(self):
        """Phase 2: Active tracking during descent (Low Gains)."""
        print("TRACKER: Descent tracking started.")
        self.ahrs.kp = 0.5  # Lower gain to trust gyro more during violent movement
        self.ahrs.ki = 0.05 # Enable integral to compensate for gyro drift
        self.is_tracking = True
        
        while self.is_tracking:
            gyro_deg, accel = self.hw.read_sensor_data()
            gyro_rad = np.radians(gyro_deg)
            self.ahrs.update_imu(*gyro_rad, *accel)
            time.sleep(1.0 / 120.0)

    def stop_tracking(self):
        self.is_tracking = False

    def finalize(self) -> float:
        """Phase 3: Settling after landing."""
        print("TRACKER: Settling after landing...")
        self.ahrs.kp = 1.0  # Moderate gains
        for _ in range(120): # 1 second of settling
            gyro_deg, accel = self.hw.read_sensor_data()
            gyro_rad = np.radians(gyro_deg)
            self.ahrs.update_imu(*gyro_rad, *accel)
            time.sleep(1.0 / 120.0)
        return self.ahrs.get_yaw()

    def close(self):
        self.hw.close()


def run_heading_tracker(command_queue: multiprocessing.Queue, result_queue: multiprocessing.Queue):
    """
    Subsystem process entry point. Manages the high-level phase logic of the tracker.
    """
    imu_cfg = CONFIG.get("hardware", {}).get("imu", {"bus": 1, "address": 0x6B})
    tracker = LSM6DSV16X_HeadingTracker(bus_num=imu_cfg["bus"], address=imu_cfg["address"])

    try:
        tracker.initialize_sensor()

        # Wait for Phase 1 Trigger
        while True:
            cmd = command_queue.get()
            if cmd == 'CALIBRATE': break
            if cmd == 'STOP': tracker.close(); return

        tracker.calibrate()
        result_queue.put(('STATUS', 'HEADING_CALIBRATED'))

        # Start tracking thread asynchronously, wait for Phase 3 Trigger
        import threading
        t = threading.Thread(target=tracker.start_tracking, daemon=True)
        t.start()

        while True:
            cmd = command_queue.get()
            if cmd == 'LANDED': break
            if cmd == 'STOP': tracker.stop_tracking(); tracker.close(); return

        tracker.stop_tracking()
        t.join(timeout=2.0)

        final_heading = tracker.finalize()
        result_queue.put(('HEADING', final_heading))

    except Exception as e:
        result_queue.put(('STATUS', f'HEADING_ERROR: {e}'))
    finally:
        tracker.close()