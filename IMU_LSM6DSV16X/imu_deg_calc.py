import smbus2
import time
import sys

import numpy as np

# ------------------------------------------------------------------
#   CONFIGURATION & CONSTANTS
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
# Accelerometer: ±2g Range
# Datasheet: 0.061 mg/LSB
ACCEL_SENSITIVITY = 0.061 / 1000.0 * 9.80665

# Gyroscope: ±125 dps Range
# Datasheet: 4.375 mdps/LSB -> 0.004375 deg/s per LSB
GYRO_SENSITIVITY = 0.004375


# ------------------------------------------------------------------
#   DRIVER CLASS
# ------------------------------------------------------------------

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
        2. Set Accel/Gyro to High Range (±16g / ±2000dps).
        3. Enable Block Data Update (BDU) for data consistency.
        """

        if self.bus is None:
            self.connect()

        try:
            print("Initializing Sensor...", end="")

            # 1. Software Reset (Bit 0 of CTRL3_C)
            self.bus.write_byte_data(self.address, REG_CTRL3_C, 0x01)
            # --- CRITICAL CHANGE HERE ---
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
        and converts them to a signed 16-bit integer.
        """

        low = self.bus.read_byte_data(self.address, reg)
        high = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low

        # Convert unsigned 16-bit to signed 16-bit (2's complement)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def get_data(self):
        """
        Polls the status register. If data is ready, reads raw bytes,
        applies sensitivity factors, and returns a clean dictionary.

        Returns:
            dict: {'ax', 'ay', 'az', 'gx', 'gy', 'gz'} in m/s^2 and rps.
            None: If data is not ready yet.
        """

        try:
            # Check Status Register (Bit 0 = Accel Data Ready, Bit 1 = Gyro Data Ready)
            status = self.bus.read_byte_data(self.address, REG_STATUS)

            # Check if BOTH Accel and Gyro have new data (Bitmask 0x03)
            if status & 0x03:
                # --- Read Raw Values ---
                # Gyroscope Data (starts at REG_OUTX_L_G)
                raw_gx = self._read_word_2c(REG_OUTX_L_G)
                raw_gy = self._read_word_2c(REG_OUTX_L_G + 2)
                raw_gz = self._read_word_2c(REG_OUTX_L_G + 4)

                # Accelerometer Data (starts at REG_OUTX_L_A)
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
                    'gx': -np.radians(raw_gx * GYRO_SENSITIVITY),
                    'gy': -np.radians(raw_gy * GYRO_SENSITIVITY),
                    'gz': np.radians(raw_gz * GYRO_SENSITIVITY)
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


# ------------------------------------------------------------------
#   USER LOGIC SECTION
# ------------------------------------------------------------------

def calibrate_gyro(imu, samples=100):
    print("Calibrating Gyro... Keep robot still!")
    offsets = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
    valid_samples = 0  # Counter for actual data received

    while valid_samples < samples:
        data = imu.get_data()
        if data:
            offsets['gx'] += data['gx']
            offsets['gy'] += data['gy']
            offsets['gz'] += data['gz']
            valid_samples += 1

            # Optional: Print progress
            if valid_samples % 10 == 0:
                print(f".", end="", flush=True)

        # Sleep to match sensor speed (approx)
        time.sleep(0.005)

    print("")  # New line

    # Average using the ACTUAL count
    offsets['gx'] /= valid_samples
    offsets['gy'] /= valid_samples
    offsets['gz'] /= valid_samples

    print(f"Calibration Complete. Offsets: {offsets}")
    return offsets


def imu_to_global_coordinates(angles_rad):
    """
    Computes the rotation matrix to convert from the IMU (Body) frame
    to the Global (Inertial) frame using Euler angles in radians.

    The function applies a Z-Y-X rotation sequence (Yaw -> Pitch -> Roll).

    Args:
        angles_rad (array-like): A list or array of 3 angles in RADIANS.
                                 Expected order: [Roll, Pitch, Yaw].

    Returns:
        np.ndarray: A 3x3 Rotation Matrix.
    """

    # Unpack angles for clarity
    # roll (phi), pitch (theta), yaw (psi)
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


def process_robotics_logic(imu_angles, data, dt):
    """
    This function is called every time new sensor data is available.
    Implement your robotics algorithms, filters, or logic here.

    Args:
        imu_angles: Contains the angles of the IMU in the global coordinates
        data: Contains keys 'ax', 'ay', 'az', 'gx', 'gy', 'gz'
              Units: m/s^2 for Accel, radians/sec for Gyro
        dt: the time window of the current measurement
    """

    # Example: Simple print to verify high-range data
    # We use :>7.2f to format the numbers nicely (align right, 2 decimal places)
    print(f"Acc: {data['ax']:>7.2f} {data['ay']:>7.2f} {data['az']:>7.2f} | "
          f"Gyr: {data['gx']:>7.2f} {data['gy']:>7.2f} {data['gz']:>7.2f}")

    # 1. Extract Local Gyro Rates (Body Frame):
    #    Units: radians/second
    gyro_body = np.array([data['gx'], data['gy'], data['gz']])

    # 2. Get Rotation Matrix (Body -> Global):
    #    We use the CURRENT angles to calculate how the robot is oriented right now.
    R = imu_to_global_coordinates(imu_angles)

    # 3. Transform Rates to Global Frame:
    #    We rotate the velocity vector from the robot's perspective to the world's perspective.
    gyro_global = R @ gyro_body

    # 4. Integrate Global Rates:
    #    Now we can safely add the change to our global Euler angles.
    imu_angles += gyro_global * dt

    # Convert to degrees for display only
    deg = np.degrees(imu_angles)
    # print(f"Roll: {deg[0]:>6.1f}° | Pitch: {deg[1]:>6.1f}° | Yaw: {deg[2]:>6.1f}°")


# ------------------------------------------------------------------
#   MAIN LOOP
# ------------------------------------------------------------------

def main():
    # 1. Instantiate the Driver
    imu = LSM6DSV16X()

    # 2. Initialize the Sensor (Reset & Configure)
    imu.initialize()

    # 3. Calibration bias (calculate gyro offsets)
    gyro_offsets = calibrate_gyro(imu)

    print("\nStarting Data Loop... (Press Ctrl+C to stop)")

    try:
        imu_angles = np.zeros(3)
        last_time = time.time()     # Initialize last_time once before the loop

        while True:
            # Get the latest data
            sensor_data = imu.get_data()

            if sensor_data:
                # Calculate dt only when we actually have a sample
                curr_time = time.time()
                dt = curr_time - last_time
                last_time = curr_time

                # Subtract the offset before processing
                sensor_data['gx'] -= gyro_offsets['gx']
                sensor_data['gy'] -= gyro_offsets['gy']
                sensor_data['gz'] -= gyro_offsets['gz']

                # Pass valid data to your custom logic
                process_robotics_logic(imu_angles, sensor_data, dt)

            # Short sleep to prevent CPU hogging
            # ODR is 120Hz (~8.3ms), so 10ms (0.01s) sleep is safe
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        imu.close()


if __name__ == "__main__":
    main()
