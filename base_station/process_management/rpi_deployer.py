"""
rpi_deployer.py - Automated deployment and execution manager for the Raspberry Pi.
Syncs the latest codebase to the Pi via SFTP (only uploading modified files)
and launches rpi_main.py over SSH.
"""

import os
import sys
import time
import logging
import threading
import paramiko
from core.config_loader import CONFIG

class RPiDeployer:
    def __init__(self):
        self.ssh_cfg = CONFIG["network"]["ssh"]
        self.host = self.ssh_cfg["host"]
        self.user = self.ssh_cfg["user"]
        self.password = self.ssh_cfg.get("pass", "")
        
        self.logger = logging.getLogger("RPI_DEPLOY")

        # Deployment path on the Raspberry Pi
        self.remote_base_dir = f"/home/{self.user}/robokedach_workspace"
        
        # The root of our local project (two levels up from this file)
        self.local_base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    def _connect(self):
        """Establish and return an SSH client connection."""
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(hostname=self.host, username=self.user, password=self.password, timeout=5)
        except Exception as e:
            self.logger.error(f"Failed to resolve or connect to SSH host '{self.host}'.")
            self.logger.error("Please verify the hostname/IP in config.json and ensure the robot is powered on and reachable.")
            self.logger.debug(f"Detailed connection error: {e}")
            raise
        return ssh

    def sync_code(self, ssh=None):
        """Uploads robot, core, and config directories to the Pi, only updating changed files."""
        self.logger.info(f"Checking for code changes on {self.host}...")
        own_connection = False
        try:
            if ssh is None:
                ssh = self._connect()
                own_connection = True
                
            sftp = ssh.open_sftp()
            
            # Ensure the base directory exists
            self._mkdir_p(sftp, self.remote_base_dir)

            # Core folders required for the RPi to run
            folders_to_sync = ["robot", "core", "config"]
            
            updates_made = False
            for folder in folders_to_sync:
                local_folder = os.path.join(self.local_base_dir, folder)
                if os.path.exists(local_folder):
                    changed = self._sync_folder(sftp, local_folder, f"{self.remote_base_dir}/{folder}")
                    if changed:
                        updates_made = True

            sftp.close()
            if own_connection:
                ssh.close()
            
            if updates_made:
                self.logger.info("Code sync complete (changes pushed).")
            else:
                self.logger.info("Code is up to date (no changes).")
                
        except Exception as e:
            self.logger.error(f"Failed to sync code: {e}")

    def _mkdir_p(self, sftp, remote_directory):
        """Emulates 'mkdir -p' in SFTP."""
        dirs_ = []
        dir_ = str(remote_directory)
        while len(dir_) > 1:
            dirs_.append(dir_)
            dir_, _ = os.path.split(dir_)
        if len(dir_) == 1 and not dir_.startswith("/"):
            dirs_.append(dir_)
        while len(dirs_):
            dir_ = dirs_.pop()
            try:
                sftp.stat(dir_)
            except IOError:
                sftp.mkdir(dir_)

    def _sync_folder(self, sftp, local_dir, remote_dir) -> bool:
        """Recursively syncs a folder. Returns True if any files were updated."""
        self._mkdir_p(sftp, remote_dir)
        changed = False
        
        for item in os.listdir(local_dir):
            # Ignore cache and hidden files
            if item == "__pycache__" or item.startswith("."):
                continue
                
            local_path = os.path.join(local_dir, item)
            remote_path = f"{remote_dir}/{item}"
            
            if os.path.isfile(local_path):
                try:
                    remote_stat = sftp.stat(remote_path)
                    local_stat = os.stat(local_path)
                    
                    # Upload if sizes differ or local file is newer
                    if local_stat.st_size != remote_stat.st_size or local_stat.st_mtime > remote_stat.st_mtime:
                        sftp.put(local_path, remote_path)
                        changed = True
                except IOError:
                    # Remote file doesn't exist, upload it
                    sftp.put(local_path, remote_path)
                    changed = True
                    
            elif os.path.isdir(local_path):
                sub_changed = self._sync_folder(sftp, local_path, remote_path)
                if sub_changed:
                    changed = True
                    
        return changed

    def start_rpi_main(self, ssh=None):
        """Kills any existing instance and starts rpi_main.py on the Pi."""
        self.logger.info("Launching rpi_main.py on the robot...")
        own_connection = False
        try:
            if ssh is None:
                ssh = self._connect()
                own_connection = True
            
            # 1. Gracefully kill existing processes
            ssh.exec_command("pkill -f rpi_main.py")
            time.sleep(0.5)
            
            # 2. Start new instance in the background using nohup
            # Use PYTHONUNBUFFERED=1 to satisfy unbuffered logging without 
            # breaking tests that check for the specific command string.
            cmd = (
                f"cd {self.remote_base_dir} && "
                f"export PYTHONPATH={self.remote_base_dir} PYTHONUNBUFFERED=1 && "
                f"nohup python3 robot/rpi_main.py > rpi_main.log 2>&1 &"
            )
            ssh.exec_command(cmd)
            self.logger.info("rpi_main.py is now running remotely. Starting log stream...")

            # 3. Start a background thread to stream the remote logs to the local console
            # Pass the existing connection to avoid redundant 'connect' calls
            log_thread = threading.Thread(target=self._stream_remote_logs, args=(ssh,), daemon=True)
            log_thread.start()

        except Exception as e:
            self.logger.error(f"Failed to start rpi_main.py: {e}")

    def _stream_remote_logs(self, ssh=None):
        """Opens a persistent SSH connection to tail the remote log file."""
        own_connection = False
        try:
            # Reuse provided connection or establish a new one if none provided
            if ssh is None:
                ssh = self._connect()
                own_connection = True
                
            # 'tail -F' follows by name, surviving if the file is recreated on RPi restart
            stdin, stdout, stderr = ssh.exec_command(f"tail -F {self.remote_base_dir}/rpi_main.log")
            
            for line in iter(stdout.readline, ""):
                if line:
                    # Prefix with Magenta color for remote logs
                    sys.stdout.write(f"\033[95m[RPI-REMOTE]\033[0m {line}")
                    sys.stdout.flush()
            
            if own_connection:
                ssh.close()
        except Exception as e:
            self.logger.debug(f"Log stream disconnected: {e}")