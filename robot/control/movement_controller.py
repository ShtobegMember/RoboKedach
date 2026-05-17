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

# ==============================================================================
# MOTOR CONTROLLER
# ==============================================================================

class MotorController:
    """Low-level wrapper for the RoboClaw serial commands."""
    def __init__(self, config: RobotConfig):
        self.config = config
        self.rc = Roboclaw(config.port, config.baud_rate)
        if not self.rc.Open():
            logger.warning(f"Could not open serial port {config.port}")
            
    def set_motor(self, motor: int, direction: Direction):
        """Legacy compatibility method for blocking encoder moves."""
        speed = 0 if direction == Direction.STOP else self.config.default_speed
        if direction == Direction.BACKWARD:
            speed = -speed
            
        self.set_motor_speed(motor, speed)

    def set_motor_speed(self, motor: int, speed: int):
        """
        Sets the motor to a specific PWM value (-127 to 127).
        Automatically translates negative values into Backward commands.
        """
        # Clamp speed to the absolute 0-127 range the RoboClaw accepts
        abs_speed = min(self.config.max_speed, max(0, abs(speed)))
        
        if motor == 1:
            if speed >= 0:
                self.rc.ForwardM1(self.config.address, abs_speed)
            else:
                self.rc.BackwardM1(self.config.address, abs_speed)
        elif motor == 2:
            if speed >= 0:
                self.rc.ForwardM2(self.config.address, abs_speed)
            else:
                self.rc.BackwardM2(self.config.address, abs_speed)

    def stop_all(self):
        self.rc.ForwardM1(self.config.address, 0)
        self.rc.ForwardM2(self.config.address, 0)

    def get_absolute_positions(self) -> Tuple[int, int]:
        val1 = self.rc.ReadEncM1(self.config.address)
        val2 = self.rc.ReadEncM2(self.config.address)
        return (val1[1] if val1[0] else 0, val2[1] if val2[0] else 0)

    # --- Restored Methods for Telemetry & GUI functionality ---
    def read_encoders(self) -> Tuple[int, int]:
        return self.get_absolute_positions()

    def read_speeds(self) -> Tuple[int, int]:
        val1 = self.rc.ReadSpeedM1(self.config.address)
        val2 = self.rc.ReadSpeedM2(self.config.address)
        return (val1[1] if val1[0] else 0, val2[1] if val2[0] else 0)

    def reset_encoders(self):
        self.rc.ResetEncoders(self.config.address)

# ==============================================================================
# MOVEMENT CONTROLLER
# ==============================================================================

class MovementController:
    """High-level movement coordinator (blocking and non-blocking)."""
    def __init__(self, motor_ctrl: MotorController):
        self.motor_ctrl = motor_ctrl
        self.progress_callback = None
        self.abort_callback = None

    def set_speeds(self, left_fraction: float, right_fraction: float):
        """
        Non-blocking speed control for teleoperation.
        Takes fractions from -1.0 to 1.0, scales to min/max speed,
        applies direction multipliers, and instantly sends PWM to motors.
        """
        logger.info(f"Command Received (set_speeds): L_frac={left_fraction:.2f}, R_frac={right_fraction:.2f}")
        
        cfg = self.motor_ctrl.config
        
        def calculate_pwm(fraction: float) -> int:
            if fraction == 0.0:
                return 0
            
            # Scale the fraction strictly between min_speed and max_speed
            base_speed = cfg.min_speed + abs(fraction) * (cfg.max_speed - cfg.min_speed)
            base_speed = min(cfg.max_speed, max(cfg.min_speed, base_speed))
            
            # Re-apply the sign based on the joystick/key input direction
            return int(base_speed if fraction > 0 else -base_speed)

        # Calculate base PWMs
        right_pwm = calculate_pwm(right_fraction)
        left_pwm = calculate_pwm(left_fraction)

        # Apply hardware polarity multipliers to fix inverted wiring
        m1_pwm = right_pwm * cfg.m1_multiplier
        m2_pwm = left_pwm * cfg.m2_multiplier

        # Dispatch using the new direct integer method
        self.motor_ctrl.set_motor_speed(1, m1_pwm)
        self.motor_ctrl.set_motor_speed(2, m2_pwm)

    def execute_move(self, raw_target1: float, raw_target2: float):
        """Legacy blocking move logic based on encoder targets."""
        logger.info(f"Command Received (execute_move): Target1={raw_target1}, Target2={raw_target2}")
        
        cycle_length = self.motor_ctrl.config.ticks_per_cycle
        target1 = int(round(raw_target1 / cycle_length) * cycle_length)
        target2 = int(round(raw_target2 / cycle_length) * cycle_length)
        
        m1_dir = Direction.FORWARD if target1 > 0 else Direction.BACKWARD
        m2_dir = Direction.FORWARD if target2 > 0 else Direction.BACKWARD
        if target1 == 0: m1_dir = Direction.STOP
        if target2 == 0: m2_dir = Direction.STOP
        
        m1_forward = 1 if target1 > 0 else -1
        m2_forward = 1 if target2 > 0 else -1
        if target1 == 0: m1_forward = 0
        if target2 == 0: m2_forward = 0

        # Start both motors
        self.motor_ctrl.set_motor(1, m1_dir)
        self.motor_ctrl.set_motor(2, m2_dir)

        # Monitor until targets are reached
        try:
            return self._monitor_movement(target1, target2, m1_forward, m2_forward)
        finally:
            self.motor_ctrl.stop_all()

    def _monitor_movement(self, target1: int, target2: int, dir1: int, dir2: int) -> bool:
        """Poll encoders until motors reach targets."""
        while True:
            # Check for abort callback
            if self.abort_callback and self.abort_callback():
                logger.warning("Movement sequence ABORTED by callback.")
                return False

            current1, current2 = self.motor_ctrl.get_absolute_positions()

            # Check if each motor has reached its target
            m1_done = (dir1 == 0) or (dir1 > 0 and current1 >= target1) or (dir1 < 0 and current1 <= target1)
            m2_done = (dir2 == 0) or (dir2 > 0 and current2 >= target2) or (dir2 < 0 and current2 <= target2)

            if self.progress_callback:
                self.progress_callback(current1, target1, m1_done, current2, target2, m2_done)

            if m1_done and m2_done:
                return True
            
            time.sleep(self.motor_ctrl.config.poll_interval)