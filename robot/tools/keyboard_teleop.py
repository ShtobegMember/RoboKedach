"""
keyboard_teleop.py - Standalone terminal user interface for the robot.
Provides TTY keyboard hooks, progress bars, and debugging utilities. 
Uses movement_controller.py to actually drive the hardware.
"""

import sys
import tty
import termios
import select

from robot.control.movement_controller import RobotConfig, MotorController, MovementController

# ==============================================================================
# TTY INPUT LOGIC
# ==============================================================================
def get_key(timeout=0.1):
    """Non-blocking read of a single key press from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            key = sys.stdin.read(1)
            # Handle escape sequences (like arrows)
            if key == '\x1b':
                r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r2:
                    key += sys.stdin.read(1)
                    r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r3:
                        key += sys.stdin.read(1)
            return key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

# ==============================================================================
# UI HELPERS
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
class RobotInterface:
    """Top-level keyboard-driven robot control interface."""
    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_ctrl = MotorController(config)
        self.movement_ctrl = MovementController(self.motor_ctrl)
        
        # Hook up the UI callbacks to the movement controller
        self.movement_ctrl.abort_callback = self._check_spacebar_abort
        self.movement_ctrl.progress_callback = display_progress
        
        self.running = True
        self.step = 0
        self.STEP_SIZE = 0.256 # Meters per full wheel rotation

    def _check_spacebar_abort(self) -> bool:
        """Return true if spacebar is pressed during movement."""
        return get_key(timeout=0.0) == ' '

    def display_status(self):
        abs1, abs2 = self.motor_ctrl.get_absolute_positions()
        print("\n--- ROBOT STATUS ---")
        print(f"Speed: L={self.motor_ctrl.left_speed}, R={self.motor_ctrl.right_speed}")
        print(f"Encoders: L={abs2}, R={abs1}")
        print("-" * 20 + "\n")

    def get_status_line(self) -> str:
        return f"⚙️ Speed: {self.motor_ctrl.avg_speed:03d} | 📍 Step: {self.step:04d} ({self.step * self.STEP_SIZE:.2f}m) | Cmd: "

    def handle_command(self, key: str):
        print(f"{key!r}   ", end="\n")
        
        # 360 Degree Commands
        if key == '\x1b[A':   self.movement_ctrl.execute_move(1.0, 1.0); self.step += 1
        elif key == '\x1b[B': self.movement_ctrl.execute_move(-1.0, -1.0); self.step -= 1
        elif key == '\x1b[D': self.movement_ctrl.execute_move(0.0, 1.0)
        elif key == '\x1b[C': self.movement_ctrl.execute_move(1.0, 0.0)
        
        # 90 Degree Commands (Shift Arrow Keys - Escape sequences may vary by terminal)
        elif key == '\x1b[1;2A': self.movement_ctrl.execute_move(0.25, 0.25)
        elif key == '\x1b[1;2B': self.movement_ctrl.execute_move(-0.25, -0.25)
        elif key == '\x1b[1;2D': self.movement_ctrl.execute_move(0.0, 0.25)
        elif key == '\x1b[1;2C': self.movement_ctrl.execute_move(0.25, 0.0)

        # Utilities
        elif key in ['+', '=']:
            self.motor_ctrl.adjust_speed_uniform(self.config.speed_increment)
        elif key in ['-', '_']:
            self.motor_ctrl.adjust_speed_uniform(-self.config.speed_increment)
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
        cfg = RobotConfig()
        print(f"\n🔍 DIAGNOSTIC: Initializing RoboClaw on {cfg.port}...")
        robot = RobotInterface(cfg)
        robot.run()
    except Exception as e:
        print(f"\n🚨 DIAGNOSTIC FAILED: {e}")