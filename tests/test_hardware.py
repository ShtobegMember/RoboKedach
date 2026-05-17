import pytest
import struct
from unittest.mock import patch, MagicMock
import numpy as np

# Mock smbus2 before importing the hardware modules
with patch.dict('sys.modules', {'smbus2': MagicMock()}):
    from robot.hardware.ina226 import INA226
    from robot.hardware.lsm6dsv16x import LSM6DSV16X

def test_ina226_voltage_calculation():
    """Test that a raw 16-bit register value correctly converts to Volts."""
    ina = INA226(bus_num=1, address=0x40)
    
    # 1.25mV per bit. Let's simulate a raw reading of 10,000 (12.5V)
    # 10000 in hex is 0x2710
    mock_bytes = [0x27, 0x10]
    ina.bus.read_i2c_block_data.return_value = mock_bytes
    
    voltage = ina.get_voltage()
    
    assert voltage == pytest.approx(12.5)
    ina.bus.read_i2c_block_data.assert_called_with(0x40, INA226.REG_BUS_V, 2)


def test_lsm6dsv16x_sensor_unpacking():
    """Test that the 12-byte block read correctly unpacks into gyro and accel arrays."""
    imu = LSM6DSV16X(bus_num=1, address=0x6B)
    
    # Create 12 bytes of fake data (6 little-endian 16-bit ints)
    # Let's set Gyro X to 1000, Accel Z to 2000, everything else 0
    # 1000 = 0x03E8 (E8 03), 2000 = 0x07D0 (D0 07)
    fake_data = b'\xE8\x03\x00\x00\x00\x00\x00\x00\x00\x00\xD0\x07'
    imu.bus.read_i2c_block_data.return_value = list(fake_data)
    
    gyro, accel = imu.read_sensor_data()
    
    # Validate Gyro X (1000 * 0.00875 = 8.75 deg/s)
    assert gyro[0] == pytest.approx(8.75)
    assert gyro[1] == 0.0
    assert gyro[2] == 0.0
    
    # Validate Accel Z (2000 * (0.061 / 1000 * 9.80665) = ~1.196 m/s^2)
    expected_accel_z = 2000 * (0.061 / 1000.0 * 9.80665)
    assert accel[0] == 0.0
    assert accel[1] == 0.0
    assert accel[2] == pytest.approx(expected_accel_z)