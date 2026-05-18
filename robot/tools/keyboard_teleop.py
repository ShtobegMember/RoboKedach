"""
keyboard_teleop.py - Standalone terminal user interface for the robot.
Provides TTY keyboard hooks, progress bars, and debugging utilities. 
Uses movement_controller.py to actually drive the hardware.
"""

import sys
import tty
import termios
import select
import logging
from typing import Optional

from robot.control.movement_controller import RobotConfig, MotorController, MovementController


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

def display_progress(cur1: int, tgt1: int, m1_done: bool, cur2: int, tgt2: int, m2_done: bool) -> None:
    """Display movement progress and the current command state (MOVING/STILL)."""
    pct1 = 100 if tgt1 == 0 else min(100, abs(cur1) * 100 // abs(tgt1))
    pct2 = 100 if tgt2 == 0 else min(100, abs(cur2) * 100 // abs(tgt2))
    
    state1 = "STILL" if m1_done else "MOVING"
    state2 = "STILL" if m2_done else "MOVING"
    
    print(f"\r📊 M1: {state1} ({pct1}%) | M2: {state2} ({pct2}%)     ", end="")

# ==============================================================================
# MAIN TERMINAL APP
# ==============================================================================

def _default_abort(self):
    """Terminal-based abort check (standalone mode)."""

    key = get_key(timeout=self.config.poll_interval)
    return key == ' '

class RobotInterface:
    """Top-level keyboard-driven robot control interface."""
    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_ctrl = MotorController(config)
        self.movement_ctrl = MovementController(self.motor_ctrl)
        self.movement_ctrl._should_abort = _default_abort
        self.running = True

        # Odometry step tracking
        self.step = 0
        self.STEP_SIZE = 0.256  # Meters per full wheel rotation


    def _check_spacebar_abort(self) -> bool:
        """Return true if spacebar is pressed during movement."""
        return get_key(timeout=0.0) == ' '

    def display_status(self):
        """
        Print current robot status (encoders, speed, distance).
        Can be monkey-patched by main.py to add IMU/position data.
        """

        try:
            abs1, abs2 = self.motor_ctrl.get_absolute_positions()
            print("\n--- ROBOT STATUS ---")
            print(f"Speed: L={self.motor_ctrl.left_speed} R={self.motor_ctrl.right_speed}/127")
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

        return (f"⚡ Speed: L={self.motor_ctrl.left_speed} R={self.motor_ctrl.right_speed} | "
                f"Length: {self.step * self.STEP_SIZE:.1f} | "
                f"Ready... ")

    def key_to_command(self, key: str) -> str:
        command_dict = \
        {'\x1b[A': 'forward step',
         '\x1b[B': 'backward step',
         '\x1b[D': 'left',
         '\x1b[C': 'right',
         '\x1b[1;2D': 'left legs',
         '\x1b[1;2C': 'right legs',
         '-': 'slow down',
         '=': 'speed up',
         ' ': 'abort'
        }

        if key in command_dict:
            return command_dict[key]
        else:
            print(f"Recieved bad key: {key}")
            return ''

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
            self.motor_ctrl.adjust_speed_uniform(self.config.speed_increment)
        elif key in ['-', '_']:
            self.motor_ctrl.adjust_speed_uniform(-self.config.speed_increment)

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

    def handle_command_(self, key: str):
        print(f"Recieved key: {key}")
        command = self.key_to_command(key)
        print(f'Sending command: {command}')

        import pdb; pdb.set_trace()

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

if __name__ == "__main__":
    try:
        # Initialize logging so MovementController's logs appear in the console
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            datefmt='%H:%M:%S'
        )
        cfg = RobotConfig()
        print(f"\n🔍 DIAGNOSTIC: Initializing RoboClaw on {cfg.port}...")
        robot = RobotInterface(cfg)
        
        # Verify Serial Connection
        success, version = robot.motor_ctrl.rc.ReadVersion(cfg.address)
        if success:
            print(f"✅ CONNECTION OK: Firmware Version: {version}")
        
        robot.run()
    except Exception as e:
        print(f"❌ FATAL ERROR: {e}")
