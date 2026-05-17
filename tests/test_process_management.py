import pytest
from unittest.mock import patch, MagicMock
from base_station.process_management.wsl_manager import WSLManager
from base_station.process_management.rpi_deployer import RPiDeployer

MOCK_CONFIG = {
    "network": {"ssh": {"host": "192.168.1.2", "user": "pi", "pass": "secret"}},
    "wsl": {
        "distro": "Ubuntu-24.04",
        "ros_domain_id": 42,
        "core_commands": {
            "RViz2": "rviz2 -d test.rviz",
            "Cartographer": "ros2 launch slam online.launch.py"
        }
    }
}

@patch('base_station.process_management.wsl_manager.CONFIG', MOCK_CONFIG)
@patch('base_station.process_management.wsl_manager.subprocess.Popen')
def test_wsl_start_core_nodes(mock_popen):
    """Test that WSL commands are formatted with correct env vars."""
    manager = WSLManager()
    manager.start_core_slam_nodes()
    
    assert mock_popen.call_count == 2
    
    # Check RViz command
    rviz_call_args = mock_popen.call_args_list[0][0][0]
    assert "wsl" in rviz_call_args
    assert "-d" in rviz_call_args
    assert "Ubuntu-24.04" in rviz_call_args
    
    bash_cmd = rviz_call_args[-1]
    assert "ROS_DOMAIN_ID=42" in bash_cmd
    assert "rviz2 -d test.rviz" in bash_cmd


@patch('base_station.process_management.wsl_manager.CONFIG', MOCK_CONFIG)
@patch('base_station.process_management.wsl_manager.subprocess.Popen')
@patch('base_station.process_management.wsl_manager.subprocess.run')
def test_wsl_toggle_rosbag(mock_run, mock_popen):
    """Test the state machine for starting and stopping rosbags."""
    manager = WSLManager()
    
    # Start recording
    manager.toggle_rosbag(True)
    assert manager.rosbag_proc is not None
    mock_popen.assert_called_once()
    assert "ros2 bag record -a" in mock_popen.call_args[0][0][-1]
    
    # Stop recording
    manager.toggle_rosbag(False)
    mock_run.assert_called_once()
    assert "pkill -SIGINT" in mock_run.call_args[0][0][-1]
    assert manager.rosbag_proc is None


@patch('base_station.process_management.rpi_deployer.CONFIG', MOCK_CONFIG)
@patch('base_station.process_management.rpi_deployer.paramiko.SSHClient')
def test_rpi_deployer_start_main(mock_ssh_client):
    """Test that the deployer sends the correct remote launch command over SSH."""
    mock_ssh = MagicMock()
    mock_ssh_client.return_value = mock_ssh
    
    deployer = RPiDeployer()
    deployer.start_rpi_main()
    
    mock_ssh.connect.assert_called_once_with(
        hostname="192.168.1.2", username="pi", password="secret", timeout=5
    )
    
    # Extract the exact commands sent to the Pi
    calls = mock_ssh.exec_command.call_args_list
    assert "pkill -f rpi_main.py" in calls[0][0][0]
    
    launch_cmd = calls[1][0][0]
    assert "export PYTHONPATH=" in launch_cmd
    assert "nohup python3 robot/rpi_main.py" in launch_cmd