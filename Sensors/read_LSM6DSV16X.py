import smbus2
import time
import sys


# --- Configuration ---
DEVICE_ADDRESS = 0x6B
BUS_NUM = 1

# Register Map
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
STATUS_REG = 0x1E
OUTX_L_G = 0x22
OUTX_L_A = 0x28

# --- SENSITIVITY FACTORS (Based on 0x04 Config) ---
# Accel: Range is ±2g (Default for 0x04)
ACCEL_FACTOR = 0.061 / 1000.0

# Gyro: Range is ±125 dps (Default for 0x04)
# Sensitivity for ±125 dps is 4.375 mdps/LSB
GYRO_FACTOR = 4.375 / 1000.0

GRAVITY_MS2 = 9.80665
THRESHOLD = 0.01


def read_word_2c(bus, reg):
    """Reads 2 bytes and converts to signed 16-bit integer."""
    try:
        low = bus.read_byte_data(DEVICE_ADDRESS, reg)
        high = bus.read_byte_data(DEVICE_ADDRESS, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value
    except OSError:
        return 0


def init_sensor():
    try:
        bus = smbus2.SMBus(BUS_NUM)

        # 1. Reset
        bus.write_byte_data(DEVICE_ADDRESS, CTRL3_C, 0x01)
        time.sleep(0.1)

        # 2. Configure (Stable 0x04 Setting)
        # [7:4] Scale=0 (±2g/±125dps), [3:0] ODR=4 (120Hz)
        bus.write_byte_data(DEVICE_ADDRESS, CTRL1_XL, 0x04)
        bus.write_byte_data(DEVICE_ADDRESS, CTRL2_G, 0x04)

        # 3. Enable BDU
        bus.write_byte_data(DEVICE_ADDRESS, CTRL3_C, 0x44)

        time.sleep(0.2)
        return bus
    except Exception as e:
        print(f"Error initializing: {e}")
        sys.exit(1)


def calibrate_sensor(bus):
    print("--- CALIBRATION (100 Samples) ---", end="", flush=True)

    # Assumption: Robot is already stationary when script starts

    sum_ax, sum_ay, sum_az = 0, 0, 0
    sum_gx, sum_gy, sum_gz = 0, 0, 0
    samples = 0
    target_samples = 100  # Updated to 100 as requested
    timeout = 0

    # Flush buffer once
    try:
        bus.read_byte_data(DEVICE_ADDRESS, STATUS_REG)
    except:
        pass

    while samples < target_samples:
        status = bus.read_byte_data(DEVICE_ADDRESS, STATUS_REG)
        if status & 0x03:
            # Read Raw
            ax = read_word_2c(bus, OUTX_L_A) * ACCEL_FACTOR * GRAVITY_MS2
            ay = read_word_2c(bus, OUTX_L_A + 2) * ACCEL_FACTOR * GRAVITY_MS2
            az = read_word_2c(bus, OUTX_L_A + 4) * ACCEL_FACTOR * GRAVITY_MS2

            gx = read_word_2c(bus, OUTX_L_G) * GYRO_FACTOR
            gy = read_word_2c(bus, OUTX_L_G + 2) * GYRO_FACTOR
            gz = read_word_2c(bus, OUTX_L_G + 4) * GYRO_FACTOR

            sum_ax += ax
            sum_ay += ay
            sum_az += az
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz

            samples += 1
            timeout = 0
        else:
            timeout += 1
            if timeout > 1000:
                print(" Error: Timeout.")
                return [0, 0, 0, 0, 0, 0]
            # No sleep needed here for max speed

    print(" Done.")

    off_ax = sum_ax / samples
    off_ay = sum_ay / samples
    off_az = (sum_az / samples) - GRAVITY_MS2

    off_gx = sum_gx / samples
    off_gy = sum_gy / samples
    off_gz = sum_gz / samples

    return [off_ax, off_ay, off_az, off_gx, off_gy, off_gz]


def main():
    bus = init_sensor()
    offsets = calibrate_sensor(bus)
    off_ax, off_ay, off_az, off_gx, off_gy, off_gz = offsets

    print("\nReading CORRECTED data... (Ctrl+C to stop)")

    try:
        while True:
            status = bus.read_byte_data(DEVICE_ADDRESS, STATUS_REG)
            if status & 0x03:
                # 1. Read Raw
                raw_ax = read_word_2c(bus, OUTX_L_A) * ACCEL_FACTOR * GRAVITY_MS2
                raw_ay = read_word_2c(bus, OUTX_L_A + 2) * ACCEL_FACTOR * GRAVITY_MS2
                raw_az = read_word_2c(bus, OUTX_L_A + 4) * ACCEL_FACTOR * GRAVITY_MS2

                raw_gx = read_word_2c(bus, OUTX_L_G) * GYRO_FACTOR
                raw_gy = read_word_2c(bus, OUTX_L_G + 2) * GYRO_FACTOR
                raw_gz = read_word_2c(bus, OUTX_L_G + 4) * GYRO_FACTOR

                # 2. Apply Calibration
                ax = raw_ax - off_ax
                ay = raw_ay - off_ay
                az = raw_az - off_az
                gx = raw_gx - off_gx
                gy = raw_gy - off_gy
                gz = raw_gz - off_gz

                # 3. Apply Deadzone Filter
                if abs(ax) < THRESHOLD:
                    ax = 0.0
                if abs(ay) < THRESHOLD:
                    ay = 0.0
                if abs(az) < THRESHOLD:
                    az = 0.0

                if abs(gx) < THRESHOLD:
                    gx = 0.0
                if abs(gy) < THRESHOLD:
                    gy = 0.0
                if abs(gz) < THRESHOLD:
                    gz = 0.0

                print(f"Acc: {ax:>6.2f} {ay:>6.2f} {az:>6.2f} | "
                      f"Gyr: {gx:>6.2f} {gy:>6.2f} {gz:>6.2f}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")
        bus.close()


if __name__ == "__main__":
    main()
