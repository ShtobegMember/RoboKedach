"""
imu_driver.py - Low-level I2C driver for the LSM6DSV16X IMU.
Handles sensor initialization, burst reads, and body-to-global coordinate transformation.
"""

import time
import sys
import struct
import numpy as np
import smbus2
from smbus2 import i2c_msg


# --- I2C Configuration ---

DEVICE_ADDRESS = 0x6B   # LSM6DSV16X default I2C address
BUS_NUM = 1             # RPi I2C bus

# --- Register Map ---

REG_CTRL1_XL = 0x10     # Accelerometer control
REG_CTRL2_G = 0x11      # Gyroscope control
REG_CTRL3_C = 0x12      # Control register 3 (reset, BDU)
REG_STATUS = 0x1E        # Data-ready status
REG_OUTX_L_G = 0x22     # Gyroscope output start (X low byte)

# --- Sensor Modes ---

CFG_ACCEL_2G = 0x04      # ±2g, 120Hz
CFG_GYRO_125DPS = 0x04   # ±125 dps, 120Hz

# --- Sensitivity Factors (from datasheet) ---

# ±2g  -> 0.061 mg/LSB -> convert to m/s²
ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665

# ±125 dps -> 4.375 mdps/LSB -> 0.004375 deg/s per LSB
GYRO_SENSITIVITY = 0.004375


class LSM6DSV16X:
    """Driver for the LSM6DSV16X IMU over I2C."""

    def __init__(self, bus_num=BUS_NUM, address=DEVICE_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None

    def connect(self):
        """Open the I2C bus."""
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            print(f"--- I2C Connected on Bus {self.bus_num} ---")
        except Exception as e:
            print(f"Error connecting to I2C: {e}")
            sys.exit(1)

    def initialize(self):
        """Reset the sensor and configure accelerometer/gyroscope registers."""
        if self.bus is None:
            self.connect()

        try:
            print("Initializing Sensor...", end="")
            # Software reset
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
            time.sleep(0.5)

            # Configure accelerometer and gyroscope
            self.bus.write_byte_data(self.address, REG_CTRL1_XL, CFG_ACCEL_2G)
            self.bus.write_byte_data(self.address, REG_CTRL2_G, CFG_GYRO_125DPS)

            # Enable BDU (Block Data Update) to prevent partial reads
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x44)

            time.sleep(0.2)
            print(" Done.")
            print("Configuration: 120Hz | Acc: ±2g | Gyr: ±125dps")

        except Exception as e:
            print(f"\nError initializing sensor registers: {e}")
            sys.exit(1)

    def get_data(self):
        """
        Read all 6 axes via atomic burst read when new data is available.
        Returns dict with ax/ay/az (m/s²) and gx/gy/gz (rad/s), or None.
        """
        try:
            status = self.bus.read_byte_data(self.address, REG_STATUS)

            # Both accel and gyro ready (bits 0 and 1)
            if status & 0x03:
                # Burst read: 12 contiguous bytes starting at gyro X low (0x22)
                write = i2c_msg.write(self.address, [REG_OUTX_L_G])
                read = i2c_msg.read(self.address, 12)
                self.bus.i2c_rdwr(write, read)

                # 6 signed 16-bit little-endian integers: gx, gy, gz, ax, ay, az
                gx, gy, gz, ax, ay, az = struct.unpack('<6h', bytes(read))

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
        """Close the I2C bus."""
        if self.bus:
            self.bus.close()
            print("\nI2C connection closed.")


def imu_to_global_coordinates(angles_rad):
    """
    Build the rotation matrix (body frame -> global frame) from Euler angles.
    Applies ZYX rotation order: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    roll, pitch, yaw = angles_rad[0], angles_rad[1], angles_rad[2]

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])

    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])

    return Rz @ Ry @ Rx
