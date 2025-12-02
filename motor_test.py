from gpiozero import PhaseEnableMotor
from time import sleep

# Configuration
# phase = The pin that controls direction (PH)
# enable = The pin that controls speed (EN)
motor = PhaseEnableMotor(phase=17, enable=27)

print("Starting Motor Test...")

try:
    # 1. Move Forward at 50% Speed
    print("Moving Forward - 50% Speed")
    motor.forward(speed=0.5)
    sleep(2)

    # 2. Stop
    print("Stopping")
    motor.stop()
    sleep(1)

    # 3. Move Backward at 100% Speed
    print("Moving Backward - 100% Speed")
    motor.backward(speed=1.0)
    sleep(2)

    motor.stop()
    print("Test Complete.")

except KeyboardInterrupt:
    # Safety: Stop motor if you press Ctrl+C
    motor.stop()
    print("\nForce Stopped")
