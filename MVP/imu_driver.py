"""
imu_driver.py
-------------
Low-level driver for the LSM6DSV16X IMU and coordinate transformation math.
Uses I2C burst reads for atomic, high-speed data acquisition.
"""

import time
import sys
import struct
import numpy as np
import smbus2
from smbus2 import i2c_msg


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

# --- SENSOR CONFIGURATION (Safe Mode) ---
CFG_ACCEL_2G = 0x04     # ±2g, 120Hz
CFG_GYRO_125DPS = 0x04  # ±125 dps, 120Hz

# --- SENSITIVITY FACTORS ---
# Accelerometer: ±2g Range -> 0.061 mg/LSB
ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665

# Gyroscope: ±125 dps Range -> 4.375 mdps/LSB -> 0.004375 deg/s per LSB
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
        """Establishes connection to the I2C bus."""
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            print(f"--- I2C Connected on Bus {self.bus_num} ---")
        except Exception as e:
            print(f"Error connecting to I2C: {e}")
            sys.exit(1)

    def initialize(self):
        """
        Sets up the sensor registers.
        """
        if self.bus is None:
            self.connect()

        try:
            print("Initializing Sensor...", end="")
            # 1. Software Reset
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
            time.sleep(0.5)

            # 2. Configure Accelerometer & Gyroscope
            self.bus.write_byte_data(self.address, REG_CTRL1_XL, CFG_ACCEL_2G)
            self.bus.write_byte_data(self.address, REG_CTRL2_G, CFG_GYRO_125DPS)

            # 3. Enable BDU (Block Data Update)
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x44)

            time.sleep(0.2)
            print(" Done.")
            print("Configuration: 120Hz | Acc: ±2g | Gyr: ±125dps")

        except Exception as e:
            print(f"\nError initializing sensor registers: {e}")
            sys.exit(1)

    def get_data(self):
        """
        Polls the status register. If data is ready, performs an atomic I2C
        burst read of all 6 axes to guarantee measurement synchronization.
        """
        try:
            status = self.bus.read_byte_data(self.address, REG_STATUS)

            # Check if BOTH Accel and Gyro have new data (Bitmask 0x03)
            if status & 0x03:
                # --- Atomic Burst Read ---
                # Read 12 contiguous bytes starting from Gyro X Low (0x22)
                write = i2c_msg.write(self.address, [REG_OUTX_L_G])
                read = i2c_msg.read(self.address, 12)
                self.bus.i2c_rdwr(write, read)

                # Unpack 6 signed 16-bit little-endian integers
                # Format '<6h': '<' = little-endian, '6h' = 6 standard shorts (16-bit)
                gx, gy, gz, ax, ay, az = struct.unpack('<6h', bytes(read))

                # --- Convert to Physical Units ---
                data = {
                    'ax': ax * ACCEL_SENSITIVITY,
                    'ay': ay * ACCEL_SENSITIVITY,
                    'az': az * ACCEL_SENSITIVITY,
                    'gx': np.radians(gx * GYRO_SENSITIVITY),
                    'gy': -np.radians(gy * GYRO_SENSITIVITY),
                    'gz': -np.radians(gz * GYRO_SENSITIVITY)
                }
                return data

            return None

        except Exception as e:
            print(f"Error reading data: {e}")
            return None

    def close(self):
        """Closes the I2C bus connection cleanly."""
        if self.bus:
            self.bus.close()
            print("\nI2C connection closed.")


def imu_to_global_coordinates(angles_rad):
    """
    Computes the rotation matrix to convert from the IMU (Body) frame
    to the Global (Inertial) frame using Euler angles in radians.
    """
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
