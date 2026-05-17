"""
test_hardware_ssh.py
Runs a remote diagnostic on the physical Raspberry Pi via SSH.
Requires the robot to be powered on and connected to the network.
"""

import pytest
import paramiko
from base_station.process_management.rpi_deployer import RPiDeployer
import time
from core.config_loader import CONFIG

def test_roboclaw_connection_via_ssh():
    """Logs into the RPi, stops the main server to free the serial port, and tests the hardware."""
    ssh_cfg = CONFIG["network"]["ssh"]
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        # 1. Connect to the Pi
        ssh.connect(
            hostname=ssh_cfg["host"], 
            username=ssh_cfg["user"], 
            password=ssh_cfg.get("pass", ""), 
            timeout=5
        )

        # 2. Kill the main process so the serial port (/dev/ttyAMA0) is unlocked
        ssh.exec_command("pkill -f rpi_main.py")
        time.sleep(1)

        # Get the remote base directory from the deployer
        deployer = RPiDeployer()
        remote_base_dir = deployer.remote_base_dir

        # 3. Create a python snippet to run LOCALLY on the Pi
        # This snippet attempts to open the RoboClaw and checks its version
        remote_python_script = f"""
import sys
sys.path.insert(0, '{remote_base_dir}') # Ensure project root is in Python path
from robot.control.movement_controller import RobotConfig, MotorController

try:
    cfg = RobotConfig()
    ctrl = MotorController(cfg)
    success, version = ctrl.rc.ReadVersion(cfg.address)
    if success:
        print("HARDWARE_SUCCESS")
    else:
        print("HARDWARE_FAIL_NO_VERSION")
except Exception as e:
    print(f"HARDWARE_EXCEPTION: {{e}}")
"""
        
        # 4. Execute the script over SSH
        stdin, stdout, stderr = ssh.exec_command(f"export PYTHONPATH={remote_base_dir} && python3")
        stdin.write(remote_python_script.encode('utf-8'))
        stdin.flush()
        stdin.channel.shutdown_write()
        output = stdout.read().decode('utf-8').strip()
        err_output = stderr.read().decode('utf-8').strip()

        # 5. Assert the result
        assert "HARDWARE_SUCCESS" in output, f"Hardware check failed. Output: {output} | Errors: {err_output}"

    except paramiko.AuthenticationException:
        pytest.fail("SSH Authentication failed. Check your config.json.")
    except Exception as e:
        pytest.fail(f"SSH Test Execution Failed: {e}")
    finally:
        ssh.close()