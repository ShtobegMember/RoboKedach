import sys
import tty
import termios
import time
import select
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple
from roboclaw_3 import Roboclaw


# ==============================================================================
# CONFIGURATION
# ==============================================================================

@dataclass
class RobotConfig:
    """Centralized robot configuration"""

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
    """Motor direction enumeration"""

    STOP = 0
    FORWARD = 1
    BACKWARD = -1


class TurnMode(Enum):
    """Turn angle modes"""
    """Turn angle modes"""

    FULL = 1.0  # 360 degrees (full cycle)
    QUARTER = 0.25  # 90 degrees (quarter cycle)


class MotorController:
    """Handles all RoboClaw motor control operations"""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.rc = Roboclaw(config.port, config.baud_rate)
        self.current_speed = config.default_speed

        if not self.rc.Open():
            raise ConnectionError(f"Could not open serial port: {config.port}")

        self.reset_encoders()

    def reset_encoders(self) -> bool:
        """Reset encoder counts to zero"""

        return self.rc.ResetEncoders(self.config.address)

    def read_encoders(self) -> Tuple[bool, int, int]:
        """
        Read raw encoder values from both motors
        Returns: (success, m1_value, m2_value)
        Note: Encoder readings are swapped - ReadEncM1 returns M2, ReadEncM2 returns M1
        """

        status1, enc2, _ = self.rc.ReadEncM1(self.config.address)
        status2, enc1, _ = self.rc.ReadEncM2(self.config.address)

        if not (status1 and status2):
            return False, 0, 0

        return True, enc1, enc2

    def get_cycle_positions(self, full_rotation) -> Tuple[int, int]:
        """
        Get current position within the rotation cycle (0 to ticks_per_cycle)
        Automatically handles direction correction and wrapping
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
        """Get raw absolute encoder positions (M1 negated)"""

        success, enc1, enc2 = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        return -enc1, enc2


    def set_motor(self, motor: int, direction: Direction, speed: int = None):
        """
        Set a single motor's speed and direction
        Args:
            motor: 1 or 2
            direction: Direction enum value
            speed: 0-127, uses current_speed if None
        """

        if speed is None:
            speed = self.current_speed

        speed = max(0, min(127, speed))  # Clamp to valid range

        if motor == 1:
            if direction == Direction.FORWARD:
                self.rc.BackwardM1(self.config.address, speed)  # SWAPPED
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM1(self.config.address, speed)  # SWAPPED
            else:
                self.rc.ForwardM1(self.config.address, 0)
        elif motor == 2:
            if direction == Direction.FORWARD:
                self.rc.BackwardM2(self.config.address, speed)  # SWAPPED
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM2(self.config.address, speed)  # SWAPPED
            else:
                self.rc.ForwardM2(self.config.address, 0)

    def stop_all(self):
        """Emergency stop both motors"""

        self.rc.ForwardM1(self.config.address, 0)
        self.rc.ForwardM2(self.config.address, 0)

    def adjust_speed(self, delta: int):
        """Adjust current speed by delta amount"""

        self.current_speed = max(
            self.config.min_speed,
            min(self.config.max_speed, self.current_speed + delta)
        )


class MovementController:
    """Handles high-level movement commands"""

    def __init__(self, motor_ctrl: MotorController):
        self.motor_ctrl = motor_ctrl
        self.config = motor_ctrl.config

    def drive_distance(self, m1_dir: Direction, m2_dir: Direction,
                       fraction: float = 1.0) -> bool:
        """
        Drive motors for a specified fraction of a rotation cycle
        Args:
            m1_dir: Direction for motor 1
            m2_dir: Direction for motor 2
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

        cycle_pos1, cycle_pos2 = self.motor_ctrl.get_cycle_positions(self.config.ticks_per_cycle * fraction)
        distance_to_travel = int(self.config.ticks_per_cycle * fraction)

        # Calculate target positions (absolute)
        target1 = abs_pos1 + ((distance_to_travel - cycle_pos1) * m1_forward)
        target2 = abs_pos2 + ((distance_to_travel - cycle_pos2) * m2_forward)

        angle_degrees = int(360 * fraction)
        print(f"\n🎯 Movement Plan ({angle_degrees}°):")
        print(f"   M1: {abs_pos1} → {target1} (Δ{distance_to_travel * m1_forward}) | Cycle start: {cycle_pos1}")
        print(f"   M2: {abs_pos2} → {target2} (Δ{distance_to_travel * m2_forward})")

        # Start motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor progress
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def drive_one_cycle(self, m1_dir: Direction, m2_dir: Direction) -> bool:
        """
        Drive motors for one complete rotation cycle (360°)
        Returns: True if completed, False if aborted
        """

        return self.drive_distance(m1_dir, m2_dir, fraction=1.0)

    def drive_quarter_cycle(self, m1_dir: Direction, m2_dir: Direction) -> bool:
        """
        Drive motors for one quarter rotation cycle (90°)
        Returns: True if completed, False if aborted
        """

        return self.drive_distance(m1_dir, m2_dir, fraction=0.25)

    def _monitor_movement(self, target1: int, target2: int,
                          dir1: int, dir2: int) -> bool:
        """
        Monitor movement until targets reached or aborted
        Returns: True if completed, False if aborted
        """

        while True:
            # Check for abort
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
            self._display_progress(current1, target1, current2, target2)

            # Check if both done
            if m1_done and m2_done:
                print("\n✓ Target Reached")
                return True

    def _display_progress(self, cur1: int, tgt1: int, cur2: int, tgt2: int):
        """Display movement progress on single line"""

        pct1 = 100 if tgt1 == 0 else min(100, abs(cur1) * 100 // abs(tgt1))
        pct2 = 100 if tgt2 == 0 else min(100, abs(cur2) * 100 // abs(tgt2))

        print(f"\r📊 M1: {cur1/self.config.ticks_per_cycle:.2f}/{tgt1/self.config.ticks_per_cycle} ({pct1}%) | "
              f"M2: {cur2/self.config.ticks_per_cycle:.2f}/{tgt2/self.config.ticks_per_cycle} ({pct2}%)     ", end="")


class KeyboardInput:
    """Handle keyboard input in raw mode"""

    @staticmethod
    def get_key(timeout: Optional[float] = None) -> Optional[str]:
        """
        Get single keypress (including arrow keys and modifiers)
        Returns: key string or None if timeout
        Examples: '\x1b[A' (up), '\x1b[1;2A' (shift+up)
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
                # Read the rest of the escape sequence
                extra = sys.stdin.read(1)
                if extra == '[':
                    # Could be simple arrow or modified arrow
                    seq = sys.stdin.read(1)
                    # Check if there's more (for shift/ctrl/alt modifiers)
                    if seq.isdigit():
                        # Extended sequence like '\x1b[1;2A' for Shift+Up
                        modifier = seq
                        while True:
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


# Global accessor for keyboard (for use in monitor loop)
get_key = KeyboardInput.get_key


class RobotInterface:
    """Main user interface for robot control"""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_ctrl = MotorController(config)
        self.movement_ctrl = MovementController(self.motor_ctrl)
        self.running = True
        self.step = 0
        self.STEP_SIZE = 0.256

    def display_help(self):
        """Display control instructions"""
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

    def display_status(self):
        """Display current robot status"""

        try:
            cycle_pos1, cycle_pos2 = self.motor_ctrl.get_cycle_positions(self.config.ticks_per_cycle)
            abs_pos1, abs_pos2 = self.motor_ctrl.get_absolute_positions()

            print("\n" + "-" * 60)
            print("📍 ROBOT STATUS")
            print("-" * 60)
            print(f"Speed: {self.motor_ctrl.current_speed}/127")
            print(f"Cycle Positions: M1={cycle_pos1}, M2={cycle_pos2}")
            print(f"Absolute Positions: M1={abs_pos1}, M2={abs_pos2}")
            print("-" * 60 + "\n")
        except IOError as e:
            print(f"❌ Error reading status: {e}")

    def handle_command(self, key: str):
        """Process keyboard command"""

        # DEBUG: Print the key code
        print(f"\nDEBUG: Key pressed = {repr(key)}")

        # Check for Shift modifier (indicated by ;2 in escape sequence)
        is_shift = ';2' in key

        # Navigation commands - Full cycle (360°)
        if key == '\x1b[A':  # Up arrow
            self.movement_ctrl.drive_one_cycle(Direction.FORWARD, Direction.FORWARD)
            self.step += 1
        # elif key == '\x1b[B':  # Down arrow
        #     self.movement_ctrl.drive_one_cycle(Direction.BACKWARD, Direction.BACKWARD)
        elif key == '\x1b[D':  # Left arrow
            self.movement_ctrl.drive_one_cycle(Direction.STOP, Direction.FORWARD)
        elif key == '\x1b[C':  # Right arrow
            self.movement_ctrl.drive_one_cycle(Direction.FORWARD, Direction.STOP)

        # Navigation commands - Quarter cycle (90°) with Shift
        elif key == '\x1b[1;2A':  # Shift+Up arrow
            self.movement_ctrl.drive_quarter_cycle(Direction.FORWARD, Direction.FORWARD)
        # elif key == '\x1b[1;2B':  # Shift+Down arrow
        #     self.movement_ctrl.drive_quarter_cycle(Direction.BACKWARD, Direction.BACKWARD)
        elif key == '\x1b[1;2D':  # Shift+Left arrow
            self.movement_ctrl.drive_quarter_cycle(Direction.STOP, Direction.FORWARD)
        elif key == '\x1b[1;2C':  # Shift+Right arrow
            self.movement_ctrl.drive_quarter_cycle(Direction.FORWARD, Direction.STOP)

        # Speed control
        elif key in ['+', '=']:
            self.motor_ctrl.adjust_speed(self.config.speed_increment)
        elif key in ['-', '_']:
            self.motor_ctrl.adjust_speed(-self.config.speed_increment)

        # Utility commands
        elif key.lower() == 'h':
            self.display_help()
        elif key.lower() == 's':
            self.display_status()
        elif key.lower() == 'r':
            self.motor_ctrl.reset_encoders()
            print("✓ Encoders reset")
        elif key.lower() == 'q':
            self.running = False
            print("\n👋 Exiting...")

    def run(self):
        """Main control loop"""

        self.display_help()

        try:
            while self.running:
                speed = self.motor_ctrl.current_speed
                print(f"\r⚡ Speed: {speed:3d}/127 | Ready... (h for help) | Length Ran: {self.step*self.STEP_SIZE:.2f}    ", end="")

                key = get_key()
                if key:
                    self.handle_command(key)

        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted!")

        finally:
            self.motor_ctrl.stop_all()
            print("✓ Motors stopped\n")


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================

def main():
    """Initialize and run robot control system"""

    try:
        config = RobotConfig()
        interface = RobotInterface(config)
        interface.run()

    except ConnectionError as e:
        print(f"❌ Connection Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
