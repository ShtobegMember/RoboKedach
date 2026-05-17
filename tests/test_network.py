import pytest
import struct
from unittest.mock import patch, MagicMock

# We must mock the CONFIG import before loading the modules
MOCK_CONFIG = {
    "network": {
        "rpi_ip": "127.0.0.1",
        "motor_port": 5555,
        "vm_port": 5556,
        "camera_port": 5000,
        "ssh": {"host": "localhost", "user": "test", "pass": "test"}
    }
}

with patch.dict('sys.modules', {'core.config_loader': MagicMock(CONFIG=MOCK_CONFIG)}):
    from base_station.network.motor_client import MotorCommandWorker
    from base_station.network.vm_client import VMServerWorker


def test_motor_client_send_movement():
    """Test that movement fractions are correctly packed into binary."""
    worker = MotorCommandWorker()
    worker.send_movement(1.0, -0.5)
    
    assert not worker.command_queue.empty()
    queued_data = worker.command_queue.get()
    
    # Unpack to verify endianness and float sizes match the RPi server expectation
    unpacked = struct.unpack('<ff', queued_data)
    assert unpacked[0] == pytest.approx(1.0)
    assert unpacked[1] == pytest.approx(-0.5)


def test_motor_client_send_trigger():
    """Test that string triggers are correctly encoded to bytes."""
    worker = MotorCommandWorker()
    worker.send_trigger("START_SLAM")
    
    assert not worker.command_queue.empty()
    queued_data = worker.command_queue.get()
    assert queued_data == b'START_SLAM'


@patch('base_station.network.vm_client.socket.socket')
def test_vm_client_run_loop(mock_socket_class):
    """Test that the VM client correctly receives and unpacks telemetry data."""
    mock_socket = MagicMock()
    mock_socket_class.return_value.__enter__.return_value = mock_socket
    
    # Mock socket.recv to return 8 bytes of packed floats (Voltage=12.4, Current=1.5)
    mock_socket.recv.return_value = struct.pack('<2f', 12.4, 1.5)
    
    worker = VMServerWorker()
    
    # Track emitted signals
    received_data = []
    worker.data_received.connect(lambda v, i: received_data.append((v, i)))
    
    # We only want it to run one loop iteration for the test
    def stop_after_recv(*args, **kwargs):
        worker.is_running = False
        return struct.pack('<2f', 12.4, 1.5)
    
    mock_socket.recv.side_effect = stop_after_recv
    
    worker.run()
    
    # Assert connection was attempted
    mock_socket.connect.assert_called_once_with(("127.0.0.1", 5556))
    
    # Assert signal was emitted with correct values
    assert len(received_data) == 1
    assert received_data[0][0] == pytest.approx(12.4)
    assert received_data[0][1] == pytest.approx(1.5)