"""
movement_controller.py - Core movement logic and hardware configuration.
Handles encoder synchronization, mathematical target calculation, and direct 
serial commands to the RoboClaw motor controller.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Callable, Optional
import logging
import time

from robot.hardware.roboclaw import Roboclaw

logger = logging.getLogger("MOVE_CTRL")

# ==============================================================================
# CONFIGURATION
# ==============================================================================

@dataclass
class RobotConfig:
    """Centralized hardware and control parameters for the robot."""
    port: str = "/dev/ttyAMA0"
    baud_rate: int = 38400
    address: int = 0x80

    # Motor direction corrections (Fixes inverted controls)
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
    BACKWARD = 2


class MotorController:
    """Low-level motor operations: speed control, direction, and encoder reading."""

    def __init__(self, config: RobotConfig):
        self.config = config
        self.rc = Roboclaw(config.port, config.baud_rate)
        self.left_speed = config.default_speed
        self.right_speed = config.default_speed

        if not self.rc.Open():
            raise ConnectionError(f"Could not open serial port: {config.port}")

        # Inside MotorController.__init__
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
        """
        Read raw encoder values from both motors.
        Returns: (success, m1_value, m2_value)
        """

        # The local roboclaw.py returns (success, value, status) on success, or (0, 0) on failure.
        res1 = self.rc.ReadEncM2(self.config.address)
        res2 = self.rc.ReadEncM1(self.config.address)

        if len(res1) == 3 and len(res2) == 3 and res1[0] == 1 and res2[0] == 1:
            return True, res1[1], res2[1]

        return False, 0, 0

    def get_cycle_positions(self, full_rotation) -> Tuple[int, int]:
        """
        Get current position within a rotation cycle (wrapped to 0..full_rotation).
        Applies direction correction and modular wrapping.
        """

        success, enc1_raw, enc2_raw = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        # Apply direction correction for inverted mounting
        enc1_norm = enc1_raw * self.config.m1_multiplier
        enc2_norm = enc2_raw * self.config.m2_multiplier

        # Wrap to cycle range
        pos1 = enc1_norm % full_rotation
        pos2 = enc2_norm % full_rotation

        return pos1, pos2

    def get_absolute_positions(self) -> Tuple[int, int]:
        """
        Get raw absolute encoder positions.
        """

        success, enc1, enc2 = self.read_encoders()

        if not success:
            raise IOError("Failed to read encoders")

        return enc1, enc2

    def set_motor(self, motor: int, direction: Direction, speed: int = None):
        """
        Set a single motor's speed and direction.
        Motor 1 = left side, Motor 2 = right side.
        """

        if speed is None:
            speed = self.left_speed if motor == 1 else self.right_speed

        speed = max(0, min(127, speed))  # Clamp to valid RoboClaw range

        # Swap the motor targeting so logical M1 commands physical M2, and vice versa
        physical_motor = 2 if motor == 1 else 1

        if physical_motor == 1:
            if direction == Direction.FORWARD:
                self.rc.BackwardM1(self.config.address, speed)
            elif direction == Direction.BACKWARD:
                self.rc.ForwardM1(self.config.address, speed)
            else:
                self.rc.ForwardM1(self.config.address, 0)

        elif physical_motor == 2:
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

    def adjust_speed_uniform(self, delta: int) -> bool:
        """
        Vector (1,1): apply delta to both sides. No-op if either side would
        leave [min_speed, max_speed]. Returns True if applied.
        """

        new_left = self.left_speed + delta
        new_right = self.right_speed + delta
        lo, hi = self.config.min_speed, self.config.max_speed

        if not (lo <= new_left <= hi and lo <= new_right <= hi):
            return False

        self.left_speed = new_left
        self.right_speed = new_right
        return True

    def adjust_speed_diff(self, delta: int) -> bool:
        """
        Vector (1,-1): right gets +delta, left gets -delta. No-op if either
        side would leave [min_speed, max_speed]. Returns True if applied.
        """

        new_left = self.left_speed - delta
        new_right = self.right_speed + delta
        lo, hi = self.config.min_speed, self.config.max_speed

        if not (lo <= new_left <= hi and lo <= new_right <= hi):
            return False

        self.left_speed = new_left
        self.right_speed = new_right
        return True
    

class MovementController:
    """High-level movement commands using encoder-based distance tracking."""

    def __init__(self, motor_ctrl: MotorController, should_abort=None):
        self.motor_ctrl = motor_ctrl
        self.config = motor_ctrl.config
        self._should_abort = should_abort

    def drive_distance(self, m1_dir: Direction, m2_dir: Direction, fraction: float = 1.0) -> bool:
        """
        Drive motors for a specified fraction of a full wheel rotation cycle.
        Args:
            m1_dir:   Direction for motor 1
            m2_dir:   Direction for motor 2
            fraction: Fraction of full cycle (1.0 = 360°, 0.25 = 90°)
        Returns: True if completed, False if aborted by user (spacebar)
        """

        if self.motor_ctrl.left_speed <= 0 and self.motor_ctrl.right_speed <= 0:
            return False

        # Record starting positions
        abs_pos1, abs_pos2 = self.motor_ctrl.get_absolute_positions()

        # Calculate effective movement direction per motor
        m1_forward = m1_dir.value * self.config.m1_multiplier
        m2_forward = m2_dir.value * self.config.m2_multiplier

        # We want to move by a full exact cycle fraction in the requested direction
        cycle_length = self.config.ticks_per_cycle * fraction
        raw_target1 = abs_pos1 + (cycle_length * m1_forward)
        raw_target2 = abs_pos2 + (cycle_length * m2_forward)

        # Snap to the nearest mathematically perfect phase boundary to ensure
        # constant-phase synchronization even if motors drift slightly.
        target1 = int(round(raw_target1 / cycle_length) * cycle_length)
        target2 = int(round(raw_target2 / cycle_length) * cycle_length)

        # Start both motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor until targets are reached (or aborted)
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def _monitor_movement(self, target1: int, target2: int, dir1: int, dir2: int) -> bool:
        """
        Poll encoders in a loop until both motors reach their target positions.
        Pressing spacebar aborts the movement.
        """

        while True:
            # Check for abort (spacebar in standalone, or network abort/heartbeat timeout)
            if self._should_abort():
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

            display_progress(current1, target1, m1_done, current2, target2, m2_done)


            if m1_done and m2_done:
                print("\n✓ Target Reached")
                return True
