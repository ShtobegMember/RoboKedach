"""
INA226 Voltage/Current Monitor Reader (5V 5A)
Reads bus voltage (V) and current (A) from an INA226 over I2C.

Wiring:
  - VCC -> 3.3V (Pi pin 1)
  - GND -> GND  (Pi pin 6)
  - SDA -> SDA  (Pi pin 7 / GPIO 4)
  - SCL -> SCL  (Pi pin 29 / GPIO 5)
  - ALERT -> Pi pin 11 / GPIO 11 (optional)

  - VS+ -> Battery positive (high side)
  - VS- -> Load side (after shunt)
  - VIN- and VIN+ across the shunt resistor

Hardware: INA226 on I2C bus 3, default address 0x40.
"""

import time
import struct
import smbus2


# --------------- Configuration ---------------
I2C_BUS     = 3
INA226_ADDR = 0x40

# Shunt resistor value in ohms — check your INA226 breakout board.
# Common values: 0.1 (100mR), 0.01 (10mR), 0.002 (2mR).
SHUNT_RESISTOR_OHMS = 0.01

# INA226 register addresses
REG_CONFIG       = 0x00
REG_SHUNT_V      = 0x01
REG_BUS_V        = 0x02
REG_POWER        = 0x03
REG_CURRENT      = 0x04
REG_CALIBRATION  = 0x05
REG_MANUFACTURER = 0xFE
REG_DIE_ID       = 0xFF

# INA226 LSBs (fixed by hardware)
BUS_VOLTAGE_LSB   = 1.25e-3   # 1.25 mV per bit
SHUNT_VOLTAGE_LSB = 2.5e-6    # 2.5 uV per bit


# --------------- Helper Functions ---------------
def read_register(bus, addr, reg):
    """Read a 16-bit big-endian register from the INA226."""

    data = bus.read_i2c_block_data(addr, reg, 2)
    return struct.unpack(">H", bytes(data))[0]

def read_register_signed(bus, addr, reg):
    """Read a signed 16-bit big-endian register from the INA226."""

    data = bus.read_i2c_block_data(addr, reg, 2)
    return struct.unpack(">h", bytes(data))[0]

def write_register(bus, addr, reg, value):
    """Write a 16-bit big-endian value to a register."""

    data = struct.pack(">H", value & 0xFFFF)
    bus.write_i2c_block_data(addr, reg, list(data))

def configure_ina226(bus):
    """
    Configure the INA226 and write the calibration register.

    Config register (0x00) bit layout:
      [15]    Reset
      [14:12] Averaging mode  — 0b010 = 16 samples
      [11:9]  Bus voltage conversion time  — 0b100 = 1.1 ms
      [8:6]   Shunt voltage conversion time — 0b100 = 1.1 ms
      [5:3]   Operating mode — 0b111 = continuous shunt & bus
    """

    # Bits [14:12] AVG=16, [11:9] VBUSCT=1.1ms, [8:6] VSHCT=1.1ms, [2:0] MODE=continuous both
    config = (0b010 << 12) | (0b100 << 9) | (0b100 << 6) | 0b111
    write_register(bus, INA226_ADDR, REG_CONFIG, config)

    # Calibration: pick Current_LSB = 0.25 mA for 0.01 ohm shunt (max ~8.19 A)
    # CAL = 0.00512 / (Current_LSB * R_shunt)
    current_lsb = 0.00025  # 0.25 mA
    cal_value = int(0.00512 / (current_lsb * SHUNT_RESISTOR_OHMS))
    write_register(bus, INA226_ADDR, REG_CALIBRATION, cal_value)

    return current_lsb

def read_bus_voltage(bus):
    """Read the bus voltage in volts."""

    raw = read_register_signed(bus, INA226_ADDR, REG_BUS_V)
    return raw * BUS_VOLTAGE_LSB

def read_shunt_voltage(bus):
    """Read the shunt voltage in volts."""

    raw = read_register_signed(bus, INA226_ADDR, REG_SHUNT_V)
    return raw * SHUNT_VOLTAGE_LSB

def read_current(bus, current_lsb):
    """Read the current in amps using the calibration register."""

    raw = read_register_signed(bus, INA226_ADDR, REG_CURRENT)
    return raw * current_lsb

def read_power(bus, current_lsb):
    """Read the power in watts (power LSB = 25 * current_lsb)."""

    raw = read_register(bus, INA226_ADDR, REG_POWER)
    return raw * 25 * current_lsb


# --------------- Main ---------------
def main():
    bus = smbus2.SMBus(I2C_BUS)

    # Verify device identity
    mfr_id = read_register(bus, INA226_ADDR, REG_MANUFACTURER)
    die_id = read_register(bus, INA226_ADDR, REG_DIE_ID)
    print(f"Manufacturer ID: 0x{mfr_id:04X} (expect 0x5449 = 'TI')")
    print(f"Die ID:          0x{die_id:04X} (expect 0x2260)")
    print(f"Shunt resistor:  {SHUNT_RESISTOR_OHMS * 1000:.1f} mOhm")
    print("-" * 45)

    current_lsb = configure_ina226(bus)

    try:
        while True:
            voltage = read_bus_voltage(bus)
            current = read_current(bus, current_lsb)
            power   = read_power(bus, current_lsb)
            shunt_v = read_shunt_voltage(bus)

            print(
                f"Bus: {voltage:6.3f} V | "
                f"Current: {current:7.4f} A | "
                f"Power: {power:7.4f} W | "
                f"Shunt: {shunt_v * 1000:8.4f} mV"
            )
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
