"""
ina226.py - I2C driver for the INA226 Voltage/Current monitor.
Handles register configuration, calibration math, and atomic reads.
"""

import struct
import smbus2

class INA226:
    # ========================== Register Map ==========================
    REG_CONFIG  = 0x00
    REG_BUS_V   = 0x02
    REG_CURRENT = 0x04
    REG_CAL     = 0x05
    
    # ========================== LSB Constants =========================
    BUS_V_LSB   = 1.25e-3    # 1.25 mV/bit
    CURRENT_LSB = 0.00025    # 0.25 mA/bit

    def __init__(self, bus_num: int, address: int, shunt_ohms: float = 0.01):
        self.bus_num = bus_num
        self.address = address
        self.shunt_ohms = shunt_ohms
        self.bus = smbus2.SMBus(self.bus_num)

    def _read_signed(self, reg: int) -> int:
        """Atomic 16-bit signed read from INA226."""
        data = self.bus.read_i2c_block_data(self.address, reg, 2)
        return struct.unpack(">h", bytes(data))[0]

    def _write(self, reg: int, value: int):
        """Write 16-bit big-endian value to INA226 register."""
        data = struct.pack(">H", value & 0xFFFF)
        self.bus.write_i2c_block_data(self.address, reg, list(data))

    def initialize(self):
        """Configure INA226: 16-sample averaging, 1.1ms conversion, continuous mode."""
        config = (0b010 << 12) | (0b100 << 9) | (0b100 << 6) | 0b111
        self._write(self.REG_CONFIG, config)
        
        # Calculate and set calibration register
        cal = int(0.00512 / (self.CURRENT_LSB * self.shunt_ohms))
        self._write(self.REG_CAL, cal)

    def get_voltage(self) -> float:
        """Read bus voltage in Volts."""
        raw_v = self._read_signed(self.REG_BUS_V)
        return raw_v * self.BUS_V_LSB

    def get_current(self) -> float:
        """Read current in Amperes."""
        raw_i = self._read_signed(self.REG_CURRENT)
        return raw_i * self.CURRENT_LSB

    def close(self):
        """Close the I2C bus connection."""
        self.bus.close()