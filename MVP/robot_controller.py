"""
robot_controller.py - Motor control, movement logic, and keyboard interface.
Communicates with RoboClaw motor controller over serial for encoder-based distance driving.
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
# HELP & PROGRESS DISPLAY
# ==============================================================================

def display_help():
    """Print the full control reference to the terminal."""
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
    """Display movement progress as percentage for both motors on a single line."""
    pct1 = 100 if tgt1 == 0 else min(100, abs(cur1) * 100 // abs(tgt1))
    pct2 = 100 if tgt2 == 0 else min(100, abs(cur2) * 100 // abs(tgt2))
    print(f"\r📊 M1: {pct1}% | M2: {pct2}%     ", end="")


# ==============================================================================
# CONFIGURATION
# ==============================================================================

@dataclass
class RobotConfig:
    """Centralized hardware and control parameters for the robot."""

    # Serial connection to the RoboClaw controller
    port: str = "/dev/ttyAMA0"
    baud_rate: int = 38400
    address: int = 0x80

    # Motor direction corrections — both motors are mounted inverted
    m1_multiplier: int = -1
    m2_multiplier: int = -1

    # Encoder ticks per full wheel rotation
    ticks_per_cycle: int = 5880

    # Speed settings (RoboClaw PWM range: 0–127)
    default_speed: int = 64
    min_speed: int = 10
    max_speed: int = 127
    speed_increment: int = 10

    # Seconds between encoder polls during movement
    poll_interval: float = 0.01


class Direction(Enum):
    """Motor direction values for the RoboClaw."""
    STOP = 0
    FORWARD = 1
    BACKWARD = -1


# ==============================================================================
# MOTOR CONTROLLER
# ==============================================================================

class MotorController:
    """Low-level motor operations: speed control, direction, and encoder reading."""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.rc = Roboclaw(config.port, config.baud_rate)
        self.current_speed = config.default_speed

        if not self.rc.Open():
            raise ConnectionError(f"Could not open serial port: {config.port}")

        self.reset_encoders()

    def reset_encoders(self) -> bool:
        """Reset both encoder counts to zero."""
        return self.rc.ResetEncoders(self.config.address)

    def read_encoders(self) -> Tuple[bool, int, int]:
        """
        Read raw encoder values from both motors.
        Note: Hardware has M1/M2 swapped — ReadEncM1 returns M2 and vice versa.
        Returns: (success, m1_value, m2_value)
        """
        status1, enc2, _ = self.rc.ReadEncM1(self.config.address)
        status2, enc1, _ = self.rc.ReadEncM2(self.config.address)

        if not (status1 and status2):
            return False, 0, 0

        return True, enc1, enc2

    def get_cycle_positions(self, full_rotation) -> Tuple[int, int]:
        """
        Get current position within a rotation cycle (wrapped to 0..full_rotation).
        Applies direction correction and modular wrapping.
        """
        success, enc1_raw, enc2_raw = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        # Apply direction correction (M1 needs extra negation due to mounting)
        enc1_norm = -(enc1_raw * self.config.m1_multiplier)
        enc2_norm = enc2_raw * self.config.m2_multiplier

        # Wrap to cycle range
        pos1 = enc1_norm % full_rotation
        pos2 = enc2_norm % full_rotation

        return pos1, pos2

    def get_absolute_positions(self) -> Tuple[int, int]:
        """
        Get raw absolute encoder positions.
        M1 is negated to account for inverted mounting.
        """
        success, enc1, enc2 = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        return -enc1, enc2

    def set_motor(self, motor: int, direction: Direction, speed: int = None):
        """
        Set a single motor's speed and direction.
        Forward/Backward commands are swapped in hardware for both motors,
        so the swap is handled transparently here.
        """
        if speed is None:
            speed = self.current_speed

        speed = max(0, min(127, speed))  # Clamp to valid RoboClaw range

        # Motor 1 — Forward/Backward are hardware-swapped
        if motor == 1:
            if direction == Direction.FORWARD:
                self.rc.BackwardM1(self.config.address, speed)
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM1(self.config.address, speed)
            else:
                self.rc.ForwardM1(self.config.address, 0)

        # Motor 2 — Forward/Backward are hardware-swapped
        elif motor == 2:
            if direction == Direction.FORWARD:
                self.rc.BackwardM2(self.config.address, speed)
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM2(self.config.address, speed)
            else:
                self.rc.ForwardM2(self.config.address, 0)

    def stop_all(self):
        """Emergency stop — set both motors to zero speed."""
        self.rc.ForwardM1(self.config.address, 0)
        self.rc.ForwardM2(self.config.address, 0)

    def adjust_speed(self, delta: int):
        """Adjust current speed by delta, clamped to [min_speed, max_speed]."""
        self.current_speed = max(
            self.config.min_speed,
            min(self.config.max_speed, self.current_speed + delta)
        )


# ==============================================================================
# MOVEMENT CONTROLLER
# ==============================================================================

class MovementController:
    """High-level movement commands using encoder-based distance tracking."""

    def __init__(self, motor_ctrl: MotorController):
        self.motor_ctrl = motor_ctrl
        self.config = motor_ctrl.config

    def drive_distance(self, m1_dir: Direction, m2_dir: Direction, fraction: float = 1.0) -> bool:
        """
        Drive motors for a specified fraction of a full wheel rotation cycle.
        Args:
            m1_dir:   Direction for motor 1
            m2_dir:   Direction for motor 2
            fraction: Fraction of full cycle (1.0 = 360°, 0.25 = 90°)
        Returns: True if completed, False if aborted by user (spacebar)
        """
        if self.motor_ctrl.current_speed <= 0:
            return False

        # Record starting positions
        abs_pos1, abs_pos2 = self.motor_ctrl.get_absolute_positions()

        # Calculate effective movement direction per motor
        m1_forward = m1_dir.value * self.config.m1_multiplier
        m2_forward = m2_dir.value * self.config.m2_multiplier

        # Account for current position within the cycle to complete the
        # remaining fraction exactly
        cycle_pos1, cycle_pos2 = self.motor_ctrl.get_cycle_positions(
            self.config.ticks_per_cycle * fraction
        )
        distance_to_travel = int(self.config.ticks_per_cycle * fraction)

        # Calculate absolute target positions
        target1 = abs_pos1 + ((distance_to_travel - cycle_pos1) * m1_forward)
        target2 = abs_pos2 + ((distance_to_travel - cycle_pos2) * m2_forward)

        # Start both motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor until targets are reached (or aborted)
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def _monitor_movement(self, target1: int, target2: int,
                          dir1: int, dir2: int) -> bool:
        """
        Poll encoders in a loop until both motors reach their target positions.
        Pressing spacebar aborts the movement.
        """
        while True:
            # Check for abort (spacebar)
            key = get_key(timeout=self.config.poll_interval)
            if key == ' ':
                print("\n⚠️  ABORTED")
                return False

            # Read current encoder positions
            current1, current2 = self.motor_ctrl.get_absolute_positions()

            # Check if each motor has reached its target
            m1_done = (dir1 == 0) or \
                      (dir1 > 0 and current1 >= target1) or \
                      (dir1 < 0 and current1 <= target1)

            m2_done = (dir2 == 0) or \
                      (dir2 > 0 and current2 >= target2) or \
                      (dir2 < 0 and current2 <= target2)

            # Stop individual motors as they reach their targets
            if m1_done:
                self.motor_ctrl.set_motor(1, Direction.STOP)
            if m2_done:
                self.motor_ctrl.set_motor(2, Direction.STOP)

            display_progress(current1, target1, current2, target2)

            if m1_done and m2_done:
                print("\n✓ Target Reached")
                return True


# ==============================================================================
# KEYBOARD INPUT
# ==============================================================================

def get_key(timeout: Optional[float] = None) -> Optional[str]:
    """
    Read a single keypress from stdin using raw terminal mode.
    Handles multi-byte ANSI escape sequences (arrow keys, shift+arrow, etc.).
    Returns None on timeout.
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

        # Parse ANSI escape sequences: ESC [ <params> <letter>
        if ch == '\x1b':
            extra = sys.stdin.read(1)
            if extra == '[':
                seq = sys.stdin.read(1)
                if seq.isdigit():
                    # Read extended modifier sequence (e.g., "1;2A" for Shift+Up)
                    modifier = seq
                    while True:
                        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                        if not ready:
                            break  # No more data, use what we have
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
# ROBOT INTERFACE
# ==============================================================================

class RobotInterface:
    """Top-level keyboard-driven robot control interface."""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_ctrl = MotorController(config)
        self.movement_ctrl = MovementController(self.motor_ctrl)
        self.running = True

        # Odometry step tracking
        self.step = 0
        self.STEP_SIZE = 0.256  # Meters per full wheel rotation

    def display_status(self):
        """
        Print current robot status (encoders, speed, distance).
        Can be monkey-patched by main.py to add IMU/position data.
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
        Generate the idle-loop status line text.
        Can be monkey-patched by main.py for custom telemetry.
        """
        return (f"⚡ Speed: {self.motor_ctrl.current_speed} | "
                f"Length: {self.step * self.STEP_SIZE:.1f} | "
                f"Ready... ")

    def handle_command(self, key: str):
        """Dispatch a keyboard command to the appropriate action."""
        m = self.movement_ctrl
        d = Direction

        # --- Full rotation (360°) ---
        if key == '\x1b[A':                                     # Up Arrow
            m.drive_distance(d.FORWARD, d.FORWARD)
            self.step += 1
        # elif key == '\x1b[B':                                 # Down Arrow (disabled)
        #     m.drive_distance(d.BACKWARD, d.BACKWARD)
        #     self.step = max(0, self.step - 1)
        elif key == '\x1b[C':                                   # Right Arrow
            m.drive_distance(d.STOP, d.FORWARD)
        elif key == '\x1b[D':                                   # Left Arrow
            m.drive_distance(d.FORWARD, d.STOP)

        # --- Quarter rotation (90°) with Shift ---
        elif key == '\x1b[1;2A':                                # Shift+Up
            m.drive_distance(d.FORWARD, d.FORWARD, 0.25)
        elif key == '\x1b[1;2C':                                # Shift+Right
            m.drive_distance(d.STOP, d.FORWARD, 0.25)
        elif key == '\x1b[1;2D':                                # Shift+Left
            m.drive_distance(d.FORWARD, d.STOP, 0.25)

        # --- Speed control ---
        elif key in ['+', '=']:
            self.motor_ctrl.adjust_speed(self.config.speed_increment)
        elif key in ['-', '_']:
            self.motor_ctrl.adjust_speed(-self.config.speed_increment)

        # --- Utility commands ---
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
        """Main control loop — blocks until user presses 'q'."""
        display_help()

        try:
            while self.running:
                print(f"\r{self.get_status_line()}", end="")

                key = get_key()
                if key:
                    self.handle_command(key)

        except KeyboardInterrupt:
            print("\n⚠️  Interrupted!")

        finally:
            self.motor_ctrl.stop_all()
            print("✓ Motors stopped\n")
