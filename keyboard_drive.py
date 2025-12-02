import sys
import tty
import termios
from gpiozero import Robot, PhaseEnableMotor
from time import sleep

# --- CONFIGURATION (Match your Table) ---
# Left Motor Pins: Phase=24, Enable=23
left_m = PhaseEnableMotor(phase=24, enable=23)

# Right Motor Pins: Phase=17, Enable=27
right_m = PhaseEnableMotor(phase=17, enable=27)

# Create the Robot object (Controls BOTH motors)
robot = Robot(left=left_m, right=right_m)


# --- KEYBOARD READING FUNCTION ---
def getch():
    """
    Reads a single character from the keyboard without hitting Enter.
    Works in standard Terminal (not Thonny).
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# --- MAIN CONTROL LOOP ---
print("=" * 30)
print("   ROBOT CONTROL CENTER")
print("=" * 30)
print("W = Forward  (Both Motors)")
print("S = Backward (Both Motors)")
print("A = Left     (Pivot)")
print("D = Right    (Pivot)")
print("SPACE = STOP")
print("Q = QUIT")
print("=" * 30)

try:
    while True:
        # Read the key pressed
        char = getch()

        # LOGIC FOR BOTH MOTORS
        if char.lower() == "w":
            print(">> FORWARD")
            robot.forward(speed=1)

        elif char.lower() == "s":
            print(">> BACKWARD")
            robot.backward(speed=1)

        elif char.lower() == "a":
            print(">> TURN LEFT")
            # Left motor back, Right motor fwd
            robot.left(speed=1)

        elif char.lower() == "d":
            print(">> TURN RIGHT")
            # Left motor fwd, Right motor back
            robot.right(speed=1)

        elif char == " ":  # Spacebar
            print(">> STOP")
            robot.stop()

        elif char.lower() == "q":
            print("Quitting...")
            robot.stop()
            break

        # Small delay to prevent CPU overload
        sleep(0.1)

except KeyboardInterrupt:
    robot.stop()
    print("\nSafety Stop Triggered.")
