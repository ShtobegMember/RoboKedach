"""
robot_controller.py
------------------
Main robotics control logic. Handles:
1. RoboClaw motor control communication
2. Encoder reading and wrapping
3. Movement logic (Distance driving, Turning)
4. Keyboard input handling
"""

import sys
import tty
import termios
import select
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple
from roboclaw import Roboclaw


# ==============================================================================
# HELP & PROGRESS PRINTS
# ==============================================================================

def display_help():
    print("\n" + "=" * 60)
    print("🤖 ROBOT CONTROL INTERFACE")
    print("=" * 60)
    print("Navigation (360° turns):")
    print("  ↑  - Forward (both motors)")
    print("  ←  - Turn Left (M2 only)")
    print("  →  - Turn Right (M1 only)")
    print("  ↓  - Backward (both motors)")
    print("\nNavigation (90° turns - HOLD SHIFT):")
    print("  Shift+↑  - Forward 90°")
    print("  Shift+←  - Turn Left 90°")
    print("  Shift+→  - Turn Right 90°")
    print("  Shift+↓  - Backward 90°")
    print("\nSpeed Control:")
    print("  +  - Increase speed")
    print("  -  - Decrease speed")
    print("\nOther:")
    print("  SPACE - Abort current movement")
    print("  h     - Show this help")
    print("  s     - Show status")
    print("  r     - Reset encoders")
    print("  q     - Quit")
    print("=" * 60 + "\n")


def display_progress(cur1: int, tgt1: int, cur2: int, tgt2: int) -> None:
    """
    Display movement progress percentage on single line.
    """

    pct1 = 100 if tgt1 == 0 else min(100, abs(cur1) * 100 // abs(tgt1))
    pct2 = 100 if tgt2 == 0 else min(100, abs(cur2) * 100 // abs(tgt2))
    print(f"\r📊 M1: {pct1}% | M2: {pct2}%     ", end="")


# ==============================================================================
# CONFIGURATION
# ==============================================================================

@dataclass
class RobotConfig:
    """
    Centralized robot configuration.
    """

    port: str = "/dev/ttyAMA0"
    baud_rate: int = 38400
    address: int = 0x80

    # Motor direction corrections
    m1_multiplier: int = -1
    m2_multiplier: int = -1

    # Encoder settings
    ticks_per_cycle: int = 5880

    # Speed settings
    default_speed: int = 64
    min_speed: int = 10
    max_speed: int = 127
    speed_increment: int = 10

    # Control settings
    poll_interval: float = 0.01  # seconds between position checks


class Direction(Enum):
    """
    Motor direction enumeration.
    """

    STOP = 0
    FORWARD = 1
    BACKWARD = -1


# ==============================================================================
# MOTOR CONTROLLER CLASS
# ==============================================================================

class MotorController:
    """
    Handles all RoboClaw motor control operations.
    """

    def __init__(self, config: RobotConfig):
        self.config = config
        self.rc = Roboclaw(config.port, config.baud_rate)
        self.current_speed = config.default_speed

        if not self.rc.Open():
            raise ConnectionError(f"Could not open serial port: {config.port}")

        self.reset_encoders()

    def reset_encoders(self) -> bool:
        """
        Reset encoder counts to zero.
        """

        return self.rc.ResetEncoders(self.config.address)

    def read_encoders(self) -> Tuple[bool, int, int]:
        """
        Read raw encoder values from both motors
        Returns: (success, m1_value, m2_value)
        Note: Encoder readings are swapped in hardware - ReadEncM1 returns M2, ReadEncM2 returns M1
        """

        status1, enc2, _ = self.rc.ReadEncM1(self.config.address)
        status2, enc1, _ = self.rc.ReadEncM2(self.config.address)

        if not (status1 and status2):
            return False, 0, 0

        return True, enc1, enc2

    def get_cycle_positions(self, full_rotation) -> Tuple[int, int]:
        """
        Get the current position within the rotation cycle (0 to ticks_per_cycle).
        Automatically handles direction correction and wrapping.
        """

        success, enc1_raw, enc2_raw = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        # Normalize direction (with extra negation for M1)
        enc1_norm = -(enc1_raw * self.config.m1_multiplier)
        enc2_norm = enc2_raw * self.config.m2_multiplier

        # Wrap to cycle range
        pos1 = enc1_norm % full_rotation
        pos2 = enc2_norm % full_rotation

        return pos1, pos2

    def get_absolute_positions(self) -> Tuple[int, int]:
        """
        Get raw absolute encoder positions (M1 negated due to mounting)
        """

        success, enc1, enc2 = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        return -enc1, enc2

    def set_motor(self, motor: int, direction: Direction, speed: int = None):
        """
        Set a single motor's speed and direction.
        Handles the hardware-specific motor swapping logic internally.
        """

        if speed is None:
            speed = self.current_speed

        speed = max(0, min(127, speed))  # Clamp to valid range

        # Logic for Motor 1 (Hardware swap handled here)
        if motor == 1:
            if direction == Direction.FORWARD:
                self.rc.BackwardM1(self.config.address, speed)  # SWAPPED
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM1(self.config.address, speed)  # SWAPPED
            else:
                self.rc.ForwardM1(self.config.address, 0)

        # Logic for Motor 2 (Hardware swap handled here)
        elif motor == 2:
            if direction == Direction.FORWARD:
                self.rc.BackwardM2(self.config.address, speed)  # SWAPPED
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM2(self.config.address, speed)  # SWAPPED
            else:
                self.rc.ForwardM2(self.config.address, 0)

    def stop_all(self):
        """
        Emergency stop both motors.
        """

        self.rc.ForwardM1(self.config.address, 0)
        self.rc.ForwardM2(self.config.address, 0)

    def adjust_speed(self, delta: int):
        """
        Adjust current speed by delta amount.
        """

        self.current_speed = max(
            self.config.min_speed,
            min(self.config.max_speed, self.current_speed + delta)
        )


# ==============================================================================
# MOVEMENT CONTROLLER CLASS
# ==============================================================================

class MovementController:
    """
    Handles high-level movement commands (Drive Distance, etc.).
    """

    def __init__(self, motor_ctrl: MotorController):
        self.motor_ctrl = motor_ctrl
        self.config = motor_ctrl.config

    def drive_distance(self, m1_dir: Direction, m2_dir: Direction, fraction: float = 1.0) -> bool:
        """
        Drive motors for a specified fraction of a rotation cycle.
        Args:
            m1_dir: direction of first motor
            m2_dir: direction of second motor
            fraction: Fraction of full cycle (1.0 = full, 0.25 = quarter)
        Returns: True if completed, False if aborted
        """

        if self.motor_ctrl.current_speed <= 0:
            return False

        # Get starting positions
        abs_pos1, abs_pos2 = self.motor_ctrl.get_absolute_positions()

        # Calculate movement parameters
        m1_forward = m1_dir.value * self.config.m1_multiplier
        m2_forward = m2_dir.value * self.config.m2_multiplier

        # Calculate distance to travel (fraction of full cycle)
        cycle_pos1, cycle_pos2 = self.motor_ctrl.get_cycle_positions(
            self.config.ticks_per_cycle * fraction
        )
        distance_to_travel = int(self.config.ticks_per_cycle * fraction)

        # Calculate target positions (absolute)
        target1 = abs_pos1 + ((distance_to_travel - cycle_pos1) * m1_forward)
        target2 = abs_pos2 + ((distance_to_travel - cycle_pos2) * m2_forward)

        # Start motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor progress
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def _monitor_movement(self, target1: int, target2: int,
                          dir1: int, dir2: int) -> bool:
        """
        Monitor movement loop. Checks encoder values against targets.
        """

        while True:
            # Check for abort key (space-bar)
            key = get_key(timeout=self.config.poll_interval)
            if key == ' ':
                print("\n⚠️  ABORTED")
                return False

            # Read current positions
            current1, current2 = self.motor_ctrl.get_absolute_positions()

            # Check completion for each motor
            m1_done = (dir1 == 0) or \
                      (dir1 > 0 and current1 >= target1) or \
                      (dir1 < 0 and current1 <= target1)

            m2_done = (dir2 == 0) or \
                      (dir2 > 0 and current2 >= target2) or \
                      (dir2 < 0 and current2 <= target2)

            # Stop individual motors as they complete
            if m1_done:
                self.motor_ctrl.set_motor(1, Direction.STOP)
            if m2_done:
                self.motor_ctrl.set_motor(2, Direction.STOP)

            # Display progress
            display_progress(current1, target1, current2, target2)

            # Check if both done
            if m1_done and m2_done:
                print("\n✓ Target Reached")
                return True


# ==============================================================================
# KEYBOARD INPUT HELPER
# ==============================================================================

def get_key(timeout: Optional[float] = None) -> Optional[str]:
    """
    Get single keypress (including arrow keys and modifiers) from stdin.
    Uses raw mode to avoid waiting for Enter.
    """

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        if timeout is not None:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if not rlist:
                return None

        ch = sys.stdin.read(1)

        # Handle escape sequences (arrow keys, etc.)
        if ch == '\x1b':
            extra = sys.stdin.read(1)
            if extra == '[':
                seq = sys.stdin.read(1)
                if seq.isdigit():
                    modifier = seq
                    while True:
                        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                        if not ready:
                            break  # No more data coming, use what we have
                        next_char = sys.stdin.read(1)
                        modifier += next_char
                        if next_char.isalpha():
                            break
                    ch = '\x1b[' + modifier
                else:
                    ch = '\x1b[' + seq
            else:
                ch += extra
        return ch

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ==============================================================================
# ROBOT INTERFACE CLASS
# ==============================================================================

class RobotInterface:
    """
    Main user interface for robot control.
    """

    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_ctrl = MotorController(config)
        self.movement_ctrl = MovementController(self.motor_ctrl)
        self.running = True

        # --- LENGTH TRACKING ---
        self.step = 0
        self.STEP_SIZE = 0.256

    def display_status(self):
        """
        Display current robot status (Encoders). Can be overridden/monkey-patched.
        """

        try:
            abs1, abs2 = self.motor_ctrl.get_absolute_positions()
            print("\n--- ROBOT STATUS ---")
            print(f"Speed: {self.motor_ctrl.current_speed}/127")
            print(f"Encoders: M1={abs1}, M2={abs2}")
            print(f"Length Ran: {self.step * self.STEP_SIZE:.1f}")
            print("--------------------\n")

        except IOError as e:
            print(f"Error reading status: {e}")

    def get_status_line(self):
        """
        Generates the text for the idle loop. Can be overridden.
        """

        return (f"⚡ Speed: {self.motor_ctrl.current_speed} | "
                f"Length: {self.step * self.STEP_SIZE:.1f} | "
                f"Ready... ")

    def handle_command(self, key: str):
        """
        Process keyboard command.
        """

        # Mapping helpers
        m = self.movement_ctrl
        d = Direction

        # Navigation commands - Full cycle (360°)
        if key == '\x1b[A':
            m.drive_distance(d.FORWARD, d.FORWARD)   # Up
            self.step += 1  # Increment Step Counter
        # elif key == '\x1b[B':
        #     m.drive_distance(d.BACKWARD, d.BACKWARD) # Down
        #     self.step = max(0, self.step - 1)  # Decrement Step Counter (no negative)
        elif key == '\x1b[C':
            m.drive_distance(d.STOP, d.FORWARD)      # Right
        elif key == '\x1b[D':
            m.drive_distance(d.FORWARD, d.STOP)      # Left

        # Navigation commands - Quarter cycle (90°) with Shift
        elif key == '\x1b[1;2A':
            m.drive_distance(d.FORWARD, d.FORWARD, 0.25)    # Shift + Up
        elif key == '\x1b[1;2C':
            m.drive_distance(d.STOP, d.FORWARD, 0.25)       # Shift + Right
        elif key == '\x1b[1;2D':
            m.drive_distance(d.FORWARD, d.STOP, 0.25)       # Shift + Left

        # Speed control
        elif key in ['+', '=']:
            self.motor_ctrl.adjust_speed(self.config.speed_increment)
        elif key in ['-', '_']:
            self.motor_ctrl.adjust_speed(-self.config.speed_increment)

        # Utility commands
        elif key.lower() == 'h':
            display_help()
        elif key.lower() == 's':
            self.display_status()
        elif key.lower() == 'r':
            self.motor_ctrl.reset_encoders()
            print("✓ Encoders reset")
        elif key.lower() == 'q':
            self.running = False

    def run(self):
        """
        Main control loop.
        """

        display_help()

        try:
            while self.running:
                # Use the new helper method instead of hardcoded text
                print(f"\r{self.get_status_line()}", end="")

                key = get_key()
                if key:
                    self.handle_command(key)

        except KeyboardInterrupt:
            print("\n⚠️  Interrupted!")

        finally:
            self.motor_ctrl.stop_all()
            print("✓ Motors stopped\n")
