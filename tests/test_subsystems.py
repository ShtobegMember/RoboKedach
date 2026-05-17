import pytest
import struct
import queue
from unittest.mock import patch, MagicMock

# Define a mock configuration
MOCK_CONFIG = {
    "network": {"motor_port": 5555, "vm_port": 5556, "camera_port": 5000},
    "hardware": {
        "ina226": {"bus": 1, "address": 64, "shunt_ohms": 0.01},
        "roboclaw": {"port": "/dev/ttyAMA0", "baud_rate": 38400, "address": 128}
    }
}

with patch.dict('sys.modules', {'core.config_loader': MagicMock(CONFIG=MOCK_CONFIG)}):
    from robot.subsystems.motor_server import run_motor_engine
    from robot.subsystems.lidar_node import run_lidar_process

@patch('robot.control.movement_controller.Roboclaw')
@patch('socket.socket')
def test_motor_server_slam_trigger(mock_socket_class, mock_roboclaw_class):
    """Test that the motor server routes 'START_SLAM' strings to the queue."""
    
    # Mock the Roboclaw hardware driver so MotorController initialization succeeds
    mock_rc_instance = MagicMock()
    mock_rc_instance.Open.return_value = True
    mock_rc_instance.ReadVersion.return_value = (True, "RoboClaw Mock")
    mock_roboclaw_class.return_value = mock_rc_instance
    
    # Mock the TCP socket server
    mock_server = MagicMock()
    mock_socket_class.return_value = mock_server
    
    mock_client = MagicMock()
    # Accept returns a client and an address
    mock_server.accept.return_value = (mock_client, ('127.0.0.1', 12345))
    
    # Send "START_SLAM" with a newline to satisfy potential line-buffered readers
    # then return empty byte to signal connection close.
    mock_client.recv.side_effect = [b'START_SLAM\n', b'']
    
    # We must trigger a KeyboardInterrupt to break out of the server's outer while loop
    mock_server.accept.side_effect = [(mock_client, ('127.0.0.1', 12345)), KeyboardInterrupt()]
    
    # Use a standard queue for more reliable state checking in synchronous tests
    cmd_queue = queue.Queue()
    
    run_motor_engine(cmd_queue)
    
    # Check if 'START_SLAM' made it into the queue
    # Using get with timeout is more robust than checking empty()
    # We handle both bytes and strings to be robust against different engine implementations.
    received = cmd_queue.get(timeout=2)
    if isinstance(received, bytes):
        received = received.decode()
        
    assert received.strip().upper() == 'START_SLAM'

@patch('robot.subsystems.lidar_node.subprocess.Popen')
def test_lidar_subprocess_launch(mock_popen):
    """Test that the Lidar subprocess is launched with the correct ROS2 arguments."""
    
    # Mock the process to just return immediately
    mock_proc = MagicMock()
    mock_proc.communicate.return_value = (None, None)
    mock_proc.returncode = 0
    mock_popen.return_value = mock_proc
    
    run_lidar_process()
    
    mock_popen.assert_called_once()
    launch_args = mock_popen.call_args[0][0]
    
    assert "ros2" in launch_args
    assert "sllidar_ros2" in launch_args
    assert "serial_port:=/dev/ttyUSB0" in launch_args