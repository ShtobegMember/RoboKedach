import pytest
from unittest.mock import patch, MagicMock

from robot.control.movement_controller import RobotConfig, MotorController, MovementController, Direction

# @patch injects a mock object into the function, keeping it active while the test runs
@patch('robot.control.movement_controller.Roboclaw')
def test_motor_speed_limits(mock_roboclaw_class):
    # 1. Setup the mock to pretend the serial port opened successfully
    mock_rc_instance = MagicMock()
    mock_rc_instance.Open.return_value = True
    mock_rc_instance.ReadVersion.return_value = (True, "RoboClaw Mock")
    mock_roboclaw_class.return_value = mock_rc_instance

    # 2. Run the actual logic tests
    cfg = RobotConfig(default_speed=64, min_speed=10, max_speed=100)
    ctrl = MotorController(cfg)
    
    assert ctrl.left_speed == 64
    
    success = ctrl.adjust_speed_uniform(20)
    assert success is True
    assert ctrl.left_speed == 84
    
    success = ctrl.adjust_speed_uniform(20)
    assert success is False
    assert ctrl.left_speed == 84 
    
    success = ctrl.adjust_speed_uniform(-80)
    assert success is False
    assert ctrl.left_speed == 84


@patch('robot.control.movement_controller.Roboclaw')
def test_absolute_position_multipliers(mock_roboclaw_class):
    mock_rc_instance = MagicMock()
    mock_rc_instance.Open.return_value = True
    mock_rc_instance.ReadVersion.return_value = (True, "RoboClaw Mock")
    mock_roboclaw_class.return_value = mock_rc_instance

    cfg = RobotConfig(m1_multiplier=-1, m2_multiplier=1)
    ctrl = MotorController(cfg)
    ctrl.read_encoders = MagicMock(return_value=(True, 500, 1000))
    
    abs1, abs2 = ctrl.get_absolute_positions()
    
    assert abs1 == -500
    assert abs2 == 1000


@patch('robot.control.movement_controller.Roboclaw')
def test_movement_controller_logic(mock_roboclaw_class):
    mock_rc_instance = MagicMock()
    mock_rc_instance.Open.return_value = True
    mock_rc_instance.ReadVersion.return_value = (True, "RoboClaw Mock")
    mock_roboclaw_class.return_value = mock_rc_instance

    cfg = RobotConfig(ticks_per_cycle=1000)
    ctrl = MotorController(cfg)
    ctrl.set_motor = MagicMock()
    ctrl.stop_all = MagicMock()
    
    move_ctrl = MovementController(ctrl)
    move_ctrl._monitor_movement = MagicMock(return_value=True)
    
    move_ctrl.execute_move(1.0, -0.5)
    
    calls = ctrl.set_motor.call_args_list
    assert calls[0][0] == (1, Direction.FORWARD)
    assert calls[1][0] == (2, Direction.BACKWARD)
    ctrl.stop_all.assert_called_once()