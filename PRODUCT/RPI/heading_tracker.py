"""
heading_tracker.py - Pre-SLAM heading tracker using LSM6DSV16X IMU.

Uses a Mahony AHRS filter (gyro + accelerometer fusion) to track 3D orientation
during vertical pipe descent. Extracts heading (yaw around gravity) as the
cumulative rotation the robot underwent.

The Mahony filter continuously estimates and compensates gyro bias via its
integral (Ki) term, handling orientation-dependent bias shifts (g-sensitivity)
that made a single-axis integrator drift.

Phases:
  1. Calibration — Mahony convergence while stationary (high gains)
  2. Tracking    — Mahony with gyro+accel fusion during descent (low gains)
  3. Settling    — Mahony refinement while stationary after landing (moderate gains)
"""

import math
import time
import threading
import struct

import smbus2
import numpy as np


# ========================== IMU Constants ==========================
DEVICE_ADDRESS = 0x6B
BUS_NUM = 1

REG_CTRL1_XL = 0x10
REG_CTRL2_G  = 0x11
REG_CTRL3_C  = 0x12
REG_CTRL6_C  = 0x15
REG_STATUS   = 0x1E
OUT_TEMP_L   = 0x20
REG_OUTX_L_G = 0x22

CFG_ACCEL_2G     = 0x06   # 120 Hz, +/-2g
CFG_GYRO_250DPS  = 0x06   # 120 Hz, default +/-250 dps

GYRO_SENSITIVITY  = 0.00875                    # deg/s per LSB at +/-250 dps
ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665   # mg/LSB -> m/s^2 at +/-2g

SAMPLE_RATE = 120.0
GRAVITY     = 9.80665

# Sign applied to the final heading output.
# +1 if the Mahony yaw sign matches your "clockwise from above = positive" convention,
# -1 if it's inverted. Verify on hardware after first test.
HEADING_SIGN = +1


# ========================== Mahony AHRS Filter ==========================
class MahonyAHRS:
    """
    Mahony Attitude and Heading Reference System.

    Fuses 3-axis gyroscope and 3-axis accelerometer to estimate 3D orientation
    as a unit quaternion. A PI controller drives the error between the measured
    gravity (accelerometer) and the estimated gravity (from the quaternion):

        error = a_measured x v_estimated           (cross product)
        bias += Ki * error * dt                    (integral — tracks gyro bias)
        gyro_corrected = gyro + Kp * error + bias  (proportional + integral)

    The accelerometer constrains pitch and roll (2 of 3 DOF). Heading (yaw
    around gravity) relies on gyro integration, but the continuous bias tracking
    via Ki reduces drift compared to open-loop integration.
    """

    def __init__(self, kp=1.0, ki=0.02):
        self.kp = kp
        self.ki = ki
        self.kp_base = kp
        self.ki_base = ki

        # Unit quaternion [w, x, y, z] — identity = no rotation
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Integral error — accumulates gyro bias estimate (rad/s, body frame)
        self.integral_error = np.array([0.0, 0.0, 0.0])

    def update(self, gyro, accel, dt, zupt=False):
        """
        Run one filter iteration with RK4 and Adaptive Gain.
        """

        q0, q1, q2, q3 = self.q
        gx, gy, gz = float(gyro[0]), float(gyro[1]), float(gyro[2])
        ax, ay, az = float(accel[0]), float(accel[1]), float(accel[2])

        # ---- Adaptive Gain Scaling (Continuous Weighting) ----
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)

        active_kp = 0.0
        active_ki = 0.0
        weight = 0.0

        if a_norm > 1e-6:
            error = abs(a_norm - GRAVITY)
            
            # The Trust Curve
            if error <= 0.05 * GRAVITY:
                weight = 1.0     # Inner Deadband (normal vibration)
            elif error >= 0.20 * GRAVITY:
                weight = 0.0     # Hard Cutoff (heavy jolt/spin)
            else:
                # Linearly scale from 1.0 down to 0.0
                error_ratio = (error - 0.05 * GRAVITY) / (0.15 * GRAVITY)
                weight = 1.0 - error_ratio

            active_kp = self.kp_base * weight
            active_ki = self.ki_base * weight

        if weight > 0.0:
            # Normalize accel
            ax /= a_norm
            ay /= a_norm
            az /= a_norm

            # Estimated gravity in body frame from current quaternion
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

            # Error = cross product
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx

            if zupt:
                ez = 0.0

            if active_ki > 0.0 and not zupt:
                self.integral_error[0] += active_ki * ex * dt
                self.integral_error[1] += active_ki * ey * dt
                self.integral_error[2] += active_ki * ez * dt

            if zupt:
                gx += active_kp * ex
                gy += active_kp * ey
                gz = 0.0
            else:
                gx += active_kp * ex + self.integral_error[0]
                gy += active_kp * ey + self.integral_error[1]
                gz += active_kp * ez + self.integral_error[2]
        else:
            if zupt:
                gz = 0.0
            else:
                gx += self.integral_error[0]
                gy += self.integral_error[1]
                gz += self.integral_error[2]

        # ---- RK4 Quaternion Integration ----
        def q_derivative(q, omega):
            qw, qx, qy, qz = q
            ox, oy, oz = omega
            return 0.5 * np.array([
                -qx * ox - qy * oy - qz * oz,
                 qw * ox + qy * oz - qz * oy,
                 qw * oy - qx * oz + qz * ox,
                 qw * oz + qx * oy - qy * ox
            ])

        omega = np.array([gx, gy, gz])
        
        k1 = q_derivative(self.q, omega)
        k2 = q_derivative(self.q + 0.5 * dt * k1, omega)
        k3 = q_derivative(self.q + 0.5 * dt * k2, omega)
        k4 = q_derivative(self.q + dt * k3, omega)
        
        self.q += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)

        # Normalize to maintain unit quaternion
        q_norm = math.sqrt(self.q[0]*self.q[0] + self.q[1]*self.q[1] + self.q[2]*self.q[2] + self.q[3]*self.q[3])
        self.q /= q_norm

    def get_yaw(self):
        """
        Gimbal-lock-free rotation about world-Z (gravity).

        Projects the body-Y axis into the world horizontal plane and takes
        atan2. Valid at any pitch, including the face-down pose (pitch = +/-90
        deg) where the standard Z-Y-X Euler yaw is singular. Body-Y is chosen
        as the "heading needle" because body-X is vertical during pipe descent,
        so body-Y is horizontal. Returns radians in [-pi, pi].

        For the upright case (no pitch/roll) this reduces to the same angle
        as the standard ZYX yaw, so it does not change the calibrated
        horizontal-rotation behavior — it only fixes the singularity at
        pitch = +/-90 deg.
        """

        q0, q1, q2, q3 = self.q

        # Body-Y axis expressed in the world frame (column 1 of R(q)).
        wy_x = 2.0 * (q1 * q2 - q0 * q3)
        wy_y = 1.0 - 2.0 * (q1 * q1 + q3 * q3)

        # IMU mounted right-side up, atan2(wy_x, wy_y) gives "clockwise-from-above = positive" heading convention
        return math.atan2(wy_x, wy_y)

    def get_euler_deg(self):
        """Return (roll, pitch, yaw) in degrees.  Z-Y-X intrinsic convention."""

        q0, q1, q2, q3 = self.q

        # Roll (X)
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3),
                          1.0 - 2.0 * (q1 * q1 + q2 * q2))
        
        # Pitch (Y) — clamped to avoid NaN at gimbal lock
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # Yaw (Z)
        yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2),
                         1.0 - 2.0 * (q2 * q2 + q3 * q3))

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def set_gains(self, kp, ki):
        """Update PI gains (e.g. switching between calibration and tracking)."""
        self.kp = kp
        self.ki = ki
        self.kp_base = kp
        self.ki_base = ki


# ========================== Heading Tracker ==========================
class LSM6DSV16X_HeadingTracker:
    """Manages the IMU and Mahony filter for heading tracking through pipe descent."""

    # Gain schedules  (Kp, Ki)
    GAINS_CALIBRATION = (10.0, 0.3)    # aggressive — fast initial convergence
    GAINS_TRACKING    = (0.5,  0.02)   # gentle     — trust gyro, slow bias track
    GAINS_SETTLING    = (5.0,  0.1)    # moderate   — post-landing refinement

    SETTLING_DURATION = 3.0            # seconds of extra Mahony after landing

    # ZUPT Thresholds (Zero Velocity Update)
    ZUPT_GYRO_THRESH  = math.radians(0.8)   # Force gyro to 0 if rotation < 0.8 deg/s
    ZUPT_ACCEL_THRESH = 0.05 * GRAVITY      # Must be within 5% of 1G to trigger ZUPT

    def __init__(self, bus_num=BUS_NUM, address=DEVICE_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None

        self.ahrs = MahonyAHRS(*self.GAINS_CALIBRATION)
        self.gyro_bias = np.array([0.0, 0.0, 0.0])

        # Yaw state
        self._yaw_initial = 0.0

        # Tracking thread
        self._tracking = False
        self._track_thread = None

    # -------------------- Sensor Init --------------------
    def initialize_sensor(self):
        """Reset the IMU and configure ODR, LPF, BDU, auto-increment."""

        self.bus = smbus2.SMBus(self.bus_num)

        # Software reset
        self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
        time.sleep(0.5)

        # Accelerometer: 120 Hz, +/-2g
        self.bus.write_byte_data(self.address, REG_CTRL1_XL, CFG_ACCEL_2G)

        # Gyroscope: 120 Hz, +/-250 dps
        self.bus.write_byte_data(self.address, REG_CTRL2_G, CFG_GYRO_250DPS)

        # BDU + IF_INC (auto-increment for burst reads)
        self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x44)

        # Hardware LPF on gyro: FTYPE=0b001 → ~30 Hz cutoff at 120 Hz ODR
        ctrl6 = self.bus.read_byte_data(self.address, REG_CTRL6_C)
        ctrl6 = (ctrl6 & 0xF8) | 0x01
        self.bus.write_byte_data(self.address, REG_CTRL6_C, ctrl6)

        time.sleep(0.2)
        print("HEADING: IMU initialized (120 Hz, +/-250 dps, HW LPF enabled)")

    # -------------------- Raw Reads --------------------
    def _data_ready(self):
        """Check if both accel and gyro have new data."""

        status = self.bus.read_byte_data(self.address, REG_STATUS)
        return bool(status & 0x03)

    def read_imu_burst(self):
        """
        Burst-read gyro + accel in one I2C transaction (12 bytes from 0x22-0x2D).
        Returns:
            gyro:  np.array [gx, gy, gz] in rad/s
            accel: np.array [ax, ay, az] in m/s^2
        """

        data = self.bus.read_i2c_block_data(self.address, REG_OUTX_L_G, 12)
        raw = struct.unpack('<6h', bytes(data))

        gyro = np.array([
            np.radians(raw[0] * GYRO_SENSITIVITY),
            np.radians(raw[1] * GYRO_SENSITIVITY),
            np.radians(raw[2] * GYRO_SENSITIVITY),
        ])

        accel = np.array([
            raw[3] * ACCEL_SENSITIVITY,
            raw[4] * ACCEL_SENSITIVITY,
            raw[5] * ACCEL_SENSITIVITY,
        ])

        return gyro, accel

    def read_temperature(self):
        """Read IMU die temperature in degrees Celsius."""

        data = self.bus.read_i2c_block_data(self.address, OUT_TEMP_L, 2)
        raw = struct.unpack('<h', bytes(data))[0]
        return raw / 256.0 + 25.0

    # -------------------- Phase 1: Calibration --------------------
    def calibrate(self, num_samples=500):
        """
        Phase 1: Stationary calibration (robot aligned to North).
        Runs the Mahony filter with high gains so the quaternion converges to the
        true orientation and the integral term builds a bias estimate.
        """

        print(f"HEADING: Calibrating ({num_samples} samples, "
              f"~{num_samples / SAMPLE_RATE:.0f}s)...")

        # Discard the first 1 second of readings to let internal hardware filters settle
        # preventing a DC bias transient from permanently skewing the calibration.
        settle_start = time.time()
        while time.time() - settle_start < 1.0:
            if self._data_ready():
                self.read_imu_burst()
            time.sleep(0.004)

        self.ahrs.set_gains(*self.GAINS_CALIBRATION)

        last_time = None
        collected = 0
        sum_gyro = np.zeros(3)

        while collected < num_samples:
            if self._data_ready():
                now = time.time()
                dt = (now - last_time) if last_time is not None else (1.0 / SAMPLE_RATE)
                dt = min(dt, 0.1)   # cap to prevent huge steps on timing glitches
                last_time = now

                gyro, accel = self.read_imu_burst()
                sum_gyro += gyro
                
                self.ahrs.update(gyro, accel, dt)
                collected += 1
            time.sleep(0.004)

        # Set manual static bias to eliminate yaw drift
        self.gyro_bias = sum_gyro / num_samples
        
        # Clear filter's integral error since we will now explicitly subtract the bias
        self.ahrs.integral_error = np.array([0.0, 0.0, 0.0])

        # Record reference yaw — heading starts at 0
        self._yaw_initial = self.ahrs.get_yaw()

        r, p, y = self.ahrs.get_euler_deg()
        temp = self.read_temperature()
        print(f"HEADING: Calibration done.  R={r:.1f}  P={p:.1f}  Y={y:.1f}  "
              f"Temp={temp:.1f}C")
        print(f"HEADING: Static Gyro Bias (rad/s): {self.gyro_bias}")

    # -------------------- Phase 2: Descent Tracking --------------------
    def start_tracking(self):
        """Start continuous Mahony updates in a background thread."""

        if self._tracking:
            return

        self.ahrs.set_gains(*self.GAINS_TRACKING)
        self._tracking = True
        self._track_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self._track_thread.start()
        print("HEADING: Tracking started (Mahony AHRS).")

    def _tracking_loop(self):
        """Mahony filter loop running at ~120 Hz."""

        last_time = None
        last_log_time = time.time()

        while self._tracking:
            if not self._data_ready():
                time.sleep(0.004)
                continue

            now = time.time()
            dt = (now - last_time) if last_time is not None else (1.0 / SAMPLE_RATE)
            dt = min(dt, 0.1)
            last_time = now

            gyro, accel = self.read_imu_burst()
            gyro -= self.gyro_bias

            # ZUPT (Zero Velocity Update)
            a_norm = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
            g_norm = math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)
            
            # Using the pre-defined accel gate and gyro noise floor to trigger
            zupt_active = abs(a_norm - GRAVITY) < self.ZUPT_ACCEL_THRESH and g_norm < self.ZUPT_GYRO_THRESH

            if zupt_active:
                gyro = np.array([0.0, 0.0, 0.0])

            self.ahrs.update(gyro, accel, dt, zupt=zupt_active)

            yaw = self.ahrs.get_yaw()
            delta_yaw = yaw - self._yaw_initial
            delta_yaw = (delta_yaw + math.pi) % (2.0 * math.pi) - math.pi
            heading = math.degrees(delta_yaw) * HEADING_SIGN

            # Periodic log (every ~5 s)
            if now - last_log_time > 5.0:
                r, p, _ = self.ahrs.get_euler_deg()
                temp = self.read_temperature()
                last_log_time = now
                print(f"HEADING: {heading:+.2f} deg | R={r:.1f} P={p:.1f} | "
                      f"Bias={self.ahrs.integral_error} | Temp={temp:.1f}C")

            time.sleep(0.004)

    def stop_tracking(self):
        """Stop the tracking loop."""

        self._tracking = False
        if self._track_thread:
            self._track_thread.join(timeout=2)

    # -------------------- Phase 3: Post-Landing Settling --------------------
    def finalize(self):
        """
        Phase 3: Stop tracking, then run the Mahony filter for a few more seconds
        with moderate gains while stationary so the bias estimate and orientation
        can settle before extracting the final heading.
        """

        self.stop_tracking()

        print(f"HEADING: Settling ({self.SETTLING_DURATION}s, moderate gains)...")
        self.ahrs.set_gains(*self.GAINS_SETTLING)

        last_time = None
        settle_start = time.time()

        while time.time() - settle_start < self.SETTLING_DURATION:
            if self._data_ready():
                now = time.time()
                dt = (now - last_time) if last_time is not None else (1.0 / SAMPLE_RATE)
                dt = min(dt, 0.1)
                last_time = now

                gyro, accel = self.read_imu_burst()
                gyro -= self.gyro_bias

                # ZUPT (Zero Velocity Update)
                a_norm = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
                g_norm = math.sqrt(gyro[0]**2 + gyro[1]**2 + gyro[2]**2)
                
                zupt_active = abs(a_norm - GRAVITY) < self.ZUPT_ACCEL_THRESH and g_norm < self.ZUPT_GYRO_THRESH

                if zupt_active:
                    gyro = np.array([0.0, 0.0, 0.0])

                self.ahrs.update(gyro, accel, dt, zupt=zupt_active)

            time.sleep(0.004)

        yaw = self.ahrs.get_yaw()
        delta_yaw = yaw - self._yaw_initial
        delta_yaw = (delta_yaw + math.pi) % (2.0 * math.pi) - math.pi
        heading_deg = math.degrees(delta_yaw) * HEADING_SIGN

        # Normalize to [-180, 180)
        heading_deg = (heading_deg + 180.0) % 360.0 - 180.0

        r, p, y = self.ahrs.get_euler_deg()
        temp = self.read_temperature()
        print(f"HEADING: Final orientation  R={r:.1f}  P={p:.1f}  Y={y:.1f}")
        print(f"HEADING: Final bias (rad/s): {self.ahrs.integral_error}")
        print(f"HEADING: Temp={temp:.1f}C")
        print(f"HEADING: *** Final heading = {heading_deg:+.2f} deg from North ***")

        return heading_deg

    # -------------------- Accessors --------------------
    def get_heading(self):
        """Return current heading estimate in degrees."""

        delta_yaw = self.ahrs.get_yaw() - self._yaw_initial
        delta_yaw = (delta_yaw + math.pi) % (2.0 * math.pi) - math.pi
        return math.degrees(delta_yaw) * HEADING_SIGN

    def close(self):
        """Release I2C bus and reset IMU."""

        self._tracking = False
        if self._track_thread and self._track_thread.is_alive():
            self._track_thread.join(timeout=2)
        if self.bus:
            try:
                # Soft reset the IMU to factory defaults so the ROS2 node 
                # (which runs immediately after us) inherits a clean hardware state.
                self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
                time.sleep(0.1)
            except Exception:
                pass
            self.bus.close()
            self.bus = None
            print("HEADING: IMU reset to factory defaults and I2C bus released.")


# ========================== Process Entry Point ==========================
def run_heading_tracker(command_queue, result_queue):
    """
    Multiprocessing entry point.  Same interface as before:
      Commands:  'CALIBRATE' | 'LANDED' | 'STOP'
      Results:   ('STATUS', 'HEADING_CALIBRATED')
                 ('HEADING', float)
                 ('STATUS', 'HEADING_ERROR:...')
    """

    tracker = LSM6DSV16X_HeadingTracker()

    try:
        tracker.initialize_sensor()

        # Wait for CALIBRATE
        while True:
            try:
                cmd = command_queue.get(timeout=0.5)
            except Exception:
                continue
            if cmd == 'CALIBRATE':
                break
            if cmd == 'STOP':
                tracker.close()
                return

        # Phase 1
        tracker.calibrate(num_samples=500)
        result_queue.put(('STATUS', 'HEADING_CALIBRATED'))

        # Phase 2
        tracker.start_tracking()

        # Wait for LANDED
        while True:
            try:
                cmd = command_queue.get(timeout=0.5)
            except Exception:
                continue
            if cmd == 'LANDED':
                break
            if cmd == 'STOP':
                tracker.stop_tracking()
                tracker.close()
                return

        # Phase 3
        final_heading = tracker.finalize()
        result_queue.put(('HEADING', final_heading))

    except Exception as e:
        print(f"HEADING: Fatal error: {e}")
        result_queue.put(('STATUS', f'HEADING_ERROR:{e}'))
    
    finally:
        tracker.close()
