import time
import math
from bmi270.BMI270 import BMI270

print("\n--- BMI270: Accel (m/s^2) + Gyro (deg/s) ---")

# 1. Initialize
imu = BMI270(i2c_addr=0x68)

# 2. Load Config
try:
    imu.load_config_file()
except Exception:
    pass

# 3. Enable Sensors
imu.enable_acc()
imu.enable_gyr()

print("Sensor Ready!")
print("-" * 95)
print(
    f"{'Accel X':<10} {'Accel Y':<10} {'Accel Z':<10} | {'Gyro X':<10} {'Gyro Y':<10} {'Gyro Z':<10}")
print(
    f"{'(m/s^2)':<10} {'(m/s^2)':<10} {'(m/s^2)':<10} | {'(deg/s)':<10} {'(deg/s)':<10} {'(deg/s)':<10}")
print("-" * 95)

# Conversion Factor: 1 Radian = ~57.296 Degrees
RAD_TO_DEG = 180 / math.pi

try:
    while True:
        acc_data = imu.get_acc_data()
        gyr_data = imu.get_gyr_data()

        if acc_data is not None and gyr_data is not None:
            # --- ACCELEROMETER (m/s^2) ---
            ax = acc_data[0] * 4.0
            ay = acc_data[1] * 4.0
            az = acc_data[2] * 4.0

            # --- GYROSCOPE (Convert Rad/s to Deg/s) ---
            gx = gyr_data[0] * RAD_TO_DEG
            gy = gyr_data[1] * RAD_TO_DEG
            gz = gyr_data[2] * RAD_TO_DEG

            # Print
            print(f"{ax:<10.3f} {ay:<10.3f} {az:<10.3f} | {gx:<10.3f} {gy:<10.3f} {gz:<10.3f}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
