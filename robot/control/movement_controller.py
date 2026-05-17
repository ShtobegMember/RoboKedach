"""
movement_controller.py - Core movement logic and hardware configuration.
Handles encoder synchronization, mathematical target calculation, and direct 
serial commands to the RoboClaw motor controller.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Callable, Optional
import time

from robot.hardware.roboclaw import Roboclaw

# ==============================================================================
# CONFIGURATION
# ==============================================================================

@dataclass
class RobotConfig:
    """Centralized hardware and control parameters for the robot."""
    port: str = "/dev/ttyAMA0"
    baud_rate: int = 38400
    address: int = 0x80

    # Motor direction corrections
    m1_multiplier: int = -1
    m2_multiplier: int = -1

    # Encoder ticks per full wheel rotation
    ticks_per_cycle: int = 8400

    # Speed settings (RoboClaw PWM range: 0–127)
    default_speed: int = 64
    min_speed: int = 10
    max_speed: int = 127
    speed_increment: int = 10
    diff_speed_increment: int = 5

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
        self.left_speed = config.default_speed
        self.right_speed = config.default_speed

        if not self.rc.Open():
            raise ConnectionError(f"Could not open serial port: {config.port}")

        success, version = self.rc.ReadVersion(config.address)
        if not success:
            print("🚨 ERROR: RoboClaw not found! Check wiring and Baud Rate.")
        else:
            print(f"✅ Connected to RoboClaw: {version}")
        
        self.reset_encoders()

    @property
    def avg_speed(self) -> int:
        return (self.left_speed + self.right_speed) // 2

    def reset_encoders(self) -> bool:
        """Reset both encoder counts to zero."""
        return self.rc.ResetEncoders(self.config.address)

    def read_encoders(self) -> Tuple[bool, int, int]:
        """Read raw encoder values from both motors. Returns: (success, m1, m2)"""
        res1 = self.rc.ReadEncM2(self.config.address)
        res2 = self.rc.ReadEncM1(self.config.address)

        if len(res1) == 3 and len(res2) == 3 and res1[0] == 1 and res2[0] == 1:
            return True, res1[1], res2[1]

        return False, 0, 0

    def get_absolute_positions(self) -> Tuple[int, int]:
        """Get absolute continuous positions taking hardware inversion into account."""
        success, enc1_raw, enc2_raw = self.read_encoders()
        if not success:
            return 0, 0
        return (enc1_raw * self.config.m1_multiplier), (enc2_raw * self.config.m2_multiplier)

    def set_motor(self, physical_motor: int, direction: Direction):
        """Set motor direction and speed handling hardware inversion."""
        speed = self.left_speed if physical_motor == 2 else self.right_speed

        # Account for hardware inversion multiplier (1 or -1)
        dir_val = direction.value * (self.config.m1_multiplier if physical_motor == 1 else self.config.m2_multiplier)
        
        # Determine actual command (1 = Forward, -1 = Backward, 0 = Stop)
        # Note: The original code logic is preserved here to match the RoboClaw wiring setup.
        if physical_motor == 1:
            if dir_val == 1:
                self.rc.BackwardM1(self.config.address, speed)
            elif dir_val == -1:
                self.rc.ForwardM1(self.config.address, speed)
            else:
                self.rc.ForwardM1(self.config.address, 0)
        elif physical_motor == 2:
            if dir_val == 1:
                self.rc.BackwardM2(self.config.address, speed)
            elif dir_val == -1:
                self.rc.ForwardM2(self.config.address, speed)
            else:
                self.rc.ForwardM2(self.config.address, 0)

    def stop_all(self):
        """Emergency stop — set both motors to zero speed."""
        self.rc.ForwardM1(self.config.address, 0)
        self.rc.ForwardM2(self.config.address, 0)

    def adjust_speed_uniform(self, delta: int) -> bool:
        """Apply delta to both sides. No-op if either side leaves [min_speed, max_speed]."""
        new_left = self.left_speed + delta
        new_right = self.right_speed + delta
        lo, hi = self.config.min_speed, self.config.max_speed
        if not (lo <= new_left <= hi and lo <= new_right <= hi):
            return False
        self.left_speed = new_left
        self.right_speed = new_right
        return True


# ==============================================================================
# MOVEMENT CONTROLLER
# ==============================================================================

class MovementController:
    """High-level algorithms to synchronize motors using encoder fractions."""

    def __init__(self, motor_ctrl: MotorController):
        self.motor_ctrl = motor_ctrl
        self.abort_callback: Optional[Callable[[], bool]] = None
        self.progress_callback: Optional[Callable[[int, int, bool, int, int, bool], None]] = None

    def execute_move(self, fraction1: float, fraction2: float) -> bool:
        """
        Executes a synced move. fractions define how many rotations each motor completes.
        Positive fractions move forward, negative move backward.
        """
        cycle_length = self.motor_ctrl.config.ticks_per_cycle
        
        # Calculate raw deltas
        raw_target1 = fraction1 * cycle_length
        raw_target2 = fraction2 * cycle_length
        
        m1_dir = Direction.FORWARD if raw_target1 > 0 else (Direction.BACKWARD if raw_target1 < 0 else Direction.STOP)
        m2_dir = Direction.FORWARD if raw_target2 > 0 else (Direction.BACKWARD if raw_target2 < 0 else Direction.STOP)

        m1_forward = 1 if m1_dir == Direction.FORWARD else (-1 if m1_dir == Direction.BACKWARD else 0)
        m2_forward = 1 if m2_dir == Direction.FORWARD else (-1 if m2_dir == Direction.BACKWARD else 0)

        # Get initial absolute positions
        current1, current2 = self.motor_ctrl.get_absolute_positions()
        raw_target1 += current1
        raw_target2 += current2

        # Snap to nearest cycle boundary to ensure constant-phase synchronization
        target1 = int(round(raw_target1 / cycle_length) * cycle_length)
        target2 = int(round(raw_target2 / cycle_length) * cycle_length)

        # Start both motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor until targets are reached
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def set_speeds(self, left_fraction: float, right_fraction: float):
        """
        Non-blocking speed control for teleoperation.
        Takes fractions from -1.0 to 1.0, scales to min/max speed,
        applies direction multipliers, and instantly sends PWM to motors.
        """
        # Physical mapping: Motor 1 is Right, Motor 2 is Left
        # (Swap these if your hardware is wired oppositely)
        right_frac = right_fraction
        left_frac = left_fraction
        
        def calculate_pwm(fraction):
            if fraction == 0.0:
                return 0
                
            # Scale the absolute fraction between min_speed and max_speed
            cfg = self.motor_ctrl.config
            base_speed = cfg.min_speed + abs(fraction) * (cfg.max_speed - cfg.min_speed)
            base_speed = min(cfg.max_speed, max(cfg.min_speed, base_speed))
            
            # Re-apply the forward/backward sign
            return int(base_speed if fraction > 0 else -base_speed)

        # Apply the hardware polarity multipliers defined in RobotConfig
        m1_pwm = calculate_pwm(right_frac) * self.motor_ctrl.config.m1_multiplier
        m2_pwm = calculate_pwm(left_frac) * self.motor_ctrl.config.m2_multiplier

        # Dispatch directly to the motor driver
        self.motor_ctrl.set_motor(1, m1_pwm)
        self.motor_ctrl.set_motor(2, m2_pwm)

    def _monitor_movement(self, target1: int, target2: int, dir1: int, dir2: int) -> bool:
        """Poll encoders until motors reach targets."""
        while True:
            # Check for abort callback
            if self.abort_callback and self.abort_callback():
                print("\n⚠️ ABORTED")
                return False

            current1, current2 = self.motor_ctrl.get_absolute_positions()

            # Check if each motor has reached its target
            m1_done = (dir1 == 0) or (dir1 > 0 and current1 >= target1) or (dir1 < 0 and current1 <= target1)
            m2_done = (dir2 == 0) or (dir2 > 0 and current2 >= target2) or (dir2 < 0 and current2 <= target2)

            if self.progress_callback:
                self.progress_callback(current1, target1, m1_done, current2, target2, m2_done)

            if m1_done: self.motor_ctrl.set_motor(1, Direction.STOP)
            if m2_done: self.motor_ctrl.set_motor(2, Direction.STOP)

            if m1_done and m2_done:
                return True

            time.sleep(self.motor_ctrl.config.poll_interval)