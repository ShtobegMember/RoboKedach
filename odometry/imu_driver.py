"""
imu_driver.py
-------------
Low-level driver for the LSM6DSV16X IMU and coordinate transformation math.
"""

import time
import sys
import smbus2
import numpy as np


# ------------------------------------------------------------------
#   CONSTANTS & CONFIGURATION
# ------------------------------------------------------------------

# I2C Parameters
DEVICE_ADDRESS = 0x6B   # Default I2C address for LSM6DSV16X
BUS_NUM = 1             # Raspberry Pi I2C bus number

# Register Map Addresses
REG_CTRL1_XL = 0x10     # Accelerometer Control Register
REG_CTRL2_G = 0x11      # Gyroscope Control Register
REG_CTRL3_C = 0x12      # Control Register 3 (includes Reset, BDU)
REG_STATUS = 0x1E       # Status Register (checks if data is ready)
REG_OUTX_L_G = 0x22     # Gyroscope Output Data (Low Byte X)
REG_OUTX_L_A = 0x28     # Accelerometer Output Data (Low Byte X)

# --- SENSOR CONFIGURATION (Safe Mode) ---
# We use 0x04 because we know the sensor accepts this reliably.
# Bits [7:4] = 0 (Range Index 0)
# Bits [3:0] = 4 (ODR 120Hz)
CFG_ACCEL_2G = 0x04     # Actually ±2g
CFG_GYRO_125DPS = 0x04  # Actually ±125 dps

# --- SENSITIVITY FACTORS (Updated for Low Range) ---
# Accelerometer: ±2g Range -> Datasheet: 0.061 mg/LSB
ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665

# Gyroscope: ±125 dps Range -> Datasheet: 4.375 mdps/LSB -> 0.004375 deg/s per LSB
GYRO_SENSITIVITY = 0.004375


class LSM6DSV16X:
    """
    Driver class for the LSM6DSV16X IMU.
    Handles I2C connection, initialization, and raw data conversion.
    """

    def __init__(self, bus_num=BUS_NUM, address=DEVICE_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None

    def connect(self):
        """
        Establishes connection to the I2C bus.
        """

        try:
            self.bus = smbus2.SMBus(self.bus_num)
            print(f"--- I2C Connected on Bus {self.bus_num} ---")

        except Exception as e:
            print(f"Error connecting to I2C: {e}")
            sys.exit(1)

    def initialize(self):
        """
        Sets up the sensor registers:
        1. Reset the device.
        2. Set Accel/Gyro to Low Range (±2g / ±125dps).
        3. Enable Block Data Update (BDU) for data consistency.
        """

        if self.bus is None:
            self.connect()

        try:
            print("Initializing Sensor...", end="")

            # 1. Software Reset (Bit 0 of CTRL3_C)
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
            # Wait longer for the reset to finish.
            time.sleep(0.5)

            # 2. Configure Accelerometer (120Hz, ±2g)
            self.bus.write_byte_data(self.address, REG_CTRL1_XL, CFG_ACCEL_2G)

            # 3. Configure Gyroscope (120Hz, ±125dps)
            self.bus.write_byte_data(self.address, REG_CTRL2_G, CFG_GYRO_125DPS)

            # 4. Enable BDU (Block Data Update)
            # Ensures high/low bytes of data are not updated while reading.
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x44)

            time.sleep(0.2)  # Allow settings to stabilize
            print(" Done.")
            print("Configuration: 120Hz | Acc: ±2g | Gyr: ±125dps")

        except Exception as e:
            print(f"\nError initializing sensor registers: {e}")
            sys.exit(1)

    def _read_word_2c(self, reg):
        """
        Internal Helper: Reads 2 bytes from register 'reg'
        and converts them to a signed 16-bit integer (2's complement).
        """

        low = self.bus.read_byte_data(self.address, reg)
        high = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low

        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def get_data(self):
        """
        Polls the status register. If data is ready, reads raw bytes,
        applies sensitivity factors, and returns a clean dictionary.

        Returns:
            dict: {'ax', 'ay', 'az', 'gx', 'gy', 'gz'} in m/s^2 and rad/s.
            None: If data is not ready yet.
        """

        try:
            # Check Status Register (Bit 0 = Accel Data Ready, Bit 1 = Gyro Data Ready)
            status = self.bus.read_byte_data(self.address, REG_STATUS)

            # Check if BOTH Accel and Gyro have new data (Bitmask 0x03)
            if status & 0x03:
                # --- Read Raw Values ---
                raw_gx = self._read_word_2c(REG_OUTX_L_G)
                raw_gy = self._read_word_2c(REG_OUTX_L_G + 2)
                raw_gz = self._read_word_2c(REG_OUTX_L_G + 4)

                raw_ax = self._read_word_2c(REG_OUTX_L_A)
                raw_ay = self._read_word_2c(REG_OUTX_L_A + 2)
                raw_az = self._read_word_2c(REG_OUTX_L_A + 4)

                # --- Convert to Physical Units ---
                data = {
                    # Accel: Raw * Sensitivity -> m/s^2
                    'ax': raw_ax * ACCEL_SENSITIVITY,
                    'ay': raw_ay * ACCEL_SENSITIVITY,
                    'az': raw_az * ACCEL_SENSITIVITY,

                    # Gyro: Raw * Sensitivity -> radians/second
                    # Note: We convert to radians here for cleaner math later
                    'gx': np.radians(raw_gx * GYRO_SENSITIVITY),
                    'gy': -np.radians(raw_gy * GYRO_SENSITIVITY),
                    'gz': -np.radians(raw_gz * GYRO_SENSITIVITY)
                }
                return data

            return None  # Data not ready

        except Exception as e:
            print(f"Error reading data: {e}")
            return None

    def close(self):
        """
        Closes the I2C bus connection cleanly.
        """

        if self.bus:
            self.bus.close()
            print("\nI2C connection closed.")


def imu_to_global_coordinates(angles_rad):
    """
    Computes the rotation matrix to convert from the IMU (Body) frame
    to the Global (Inertial) frame using Euler angles in radians.

    The function applies a Z-Y-X rotation sequence (Yaw -> Pitch -> Roll).

    Args:
        angles_rad (array-like): [Roll, Pitch, Yaw] in RADIANS.

    Returns:
        np.ndarray: A 3x3 Rotation Matrix.
    """

    # Unpack angles for clarity
    roll, pitch, yaw = angles_rad[0], angles_rad[1], angles_rad[2]

    # Pre-calculate trigonometric values
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # Rotation matrix around the Z-axis (Yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])

    # Rotation matrix around the Y-axis (Pitch)
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])

    # Rotation matrix around the X-axis (Roll)
    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])

    # Combine rotations: R = Rz * Ry * Rx
    return Rz @ Ry @ Rx
