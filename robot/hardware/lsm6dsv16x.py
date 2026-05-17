"""
lsm6dsv16x.py - I2C driver for the LSM6DSV16X IMU.
Handles bus-reading logic, register setup, and unit conversions (deg/s and m/s^2).
"""

import struct
import smbus2
import numpy as np

class LSM6DSV16X:
    # ========================== Register Map ==========================
    REG_CTRL1_XL = 0x10
    REG_CTRL2_G  = 0x11
    REG_CTRL3_C  = 0x12
    REG_CTRL6_C  = 0x15
    REG_STATUS   = 0x1E
    OUT_TEMP_L   = 0x20
    REG_OUTX_L_G = 0x22

    # ========================== Configuration =========================
    CFG_ACCEL_2G     = 0x06   # 120 Hz, +/-2g
    CFG_GYRO_250DPS  = 0x06   # 120 Hz, default +/-250 dps

    GYRO_SENSITIVITY  = 0.00875                    # deg/s per LSB at +/-250 dps
    ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665   # mg/LSB -> m/s^2 at +/-2g

    def __init__(self, bus_num: int = 1, address: int = 0x6B):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus2.SMBus(self.bus_num)

    def initialize_sensor(self):
        """Initialize the IMU with the required accelerometer and gyroscope settings."""
        # Soft reset the IMU (Write 0x01 to REG_CTRL3_C, then wait)
        self.bus.write_byte_data(self.address, self.REG_CTRL3_C, 0x01)
        
        # Configure Accelerometer
        self.bus.write_byte_data(self.address, self.REG_CTRL1_XL, self.CFG_ACCEL_2G)
        
        # Configure Gyroscope
        self.bus.write_byte_data(self.address, self.REG_CTRL2_G, self.CFG_GYRO_250DPS)
        
        # Additional configuration if needed (e.g., BDU, auto-increment)
        self.bus.write_byte_data(self.address, self.REG_CTRL3_C, 0x44)

    def read_sensor_data(self):
        """
        Reads gyro and accel data from the IMU in a single block read.
        Returns:
            gyro (np.array): [x, y, z] in deg/s
            accel (np.array): [x, y, z] in m/s^2
        """
        # Read 12 bytes starting from Gyro X-axis low byte (covers gyro + accel)
        data = self.bus.read_i2c_block_data(self.address, self.REG_OUTX_L_G, 12)
        
        # Unpack 6 signed 16-bit integers (little-endian)
        raw = struct.unpack("<6h", bytes(data))
        
        gyro = np.array([
            raw[0] * self.GYRO_SENSITIVITY,
            raw[1] * self.GYRO_SENSITIVITY,
            raw[2] * self.GYRO_SENSITIVITY
        ])
        
        accel = np.array([
            raw[3] * self.ACCEL_SENSITIVITY,
            raw[4] * self.ACCEL_SENSITIVITY,
            raw[5] * self.ACCEL_SENSITIVITY
        ])
        
        return gyro, accel

    def read_temperature(self) -> float:
        """Read IMU die temperature in degrees Celsius."""
        data = self.bus.read_i2c_block_data(self.address, self.OUT_TEMP_L, 2)
        raw = struct.unpack("<h", bytes(data))[0]
        # LSM6DSV16X typical temp sensitivity is 256 LSB/°C, 0 = 25°C
        return (raw / 256.0) + 25.0

    def close(self):
        """Close the I2C bus connection."""
        self.bus.close()