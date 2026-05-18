"""
network_utils.py - Shared networking utilities and protocol constants.
Contains helper functions for struct-packing binary data over TCP and 
generating dynamically pinned CycloneDDS XML configurations.
"""

import struct
import subprocess
import platform

# ========================== VM Data Packing Protocol ==========================
# The VM Streamer sends two floats (Voltage, Current) over TCP.
# <2f means little-endian, 2 standard floats (4 bytes each). Total = 8 bytes.
VM_DATA_FORMAT = '<2f'
VM_DATA_SIZE = struct.calcsize(VM_DATA_FORMAT)

def pack_vm_data(voltage: float, current: float) -> bytes:
    """
    Pack voltage and current into a standardized binary struct for TCP transmission.
    """
    return struct.pack(VM_DATA_FORMAT, voltage, current)

def unpack_vm_data(data: bytes) -> tuple[float, float]:
    """
    Unpack voltage and current from a binary struct received over TCP.
    
    :param data: The 8-byte byte-string received from the socket.
    :return: A tuple of (voltage, current).
    """
    if len(data) != VM_DATA_SIZE:
        raise ValueError(f"Expected {VM_DATA_SIZE} bytes for VM data, got {len(data)}")
    return struct.unpack(VM_DATA_FORMAT, data)


# ========================== ROS2 / CycloneDDS Configuration ==========================
def get_cyclonedds_config(ip_address: str) -> str:
    """
    Generate a CycloneDDS XML configuration pinned to a specific network interface IP.
    This prevents ROS2 traffic from flooding unintended networks (like Wi-Fi) and 
    restricts it to the direct fiber/ethernet link between the PC and RPi.
    
    :param ip_address: The IP address of the local network interface to bind to.
    :return: An XML string formatted for the CYCLONEDDS_URI environment variable.
    """
    return (
        '<CycloneDDS><Domain><General>'
        f'<NetworkInterfaceAddress>{ip_address}</NetworkInterfaceAddress>'
        '<AllowMulticast>spdp</AllowMulticast>'
        '</General></Domain></CycloneDDS>'
    )

def run_command(command, description, as_admin=False):
    """Executes a shell command and checks for success."""
    print(f"➔ {description}...")
    
    # If admin rights are requested on Windows, wrap the command in a PowerShell RunAs verb.
    # Note: This will trigger a UAC prompt and prevents direct output capture.
    if as_admin and platform.system() == "Windows":
        # command = f'powershell -Command "Start-Process cmd -ArgumentList \'/c {command}\' -Verb RunAs -Wait"'
        command = f'powershell -Command "Start-Process powershell -Verb RunAs -ArgumentList \'{command}\''

    try:
        # Capture output to check success
        print(f"    Running command: \n {command}")
        result = subprocess.run(
            command, 
            shell=True, 
            check=True,
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True
        )
        return True
    except subprocess.CalledProcessError as e:
        print(f"   ❌ Failed (Exit Code: {e.returncode})")
        if e.stderr.strip():
            print(f"   Error Details: {e.stderr.strip()}")
        elif e.stdout.strip():
            print(f"   Output: {e.stdout.strip()}")
        return False

def setup_windows_network(interface_name, network_name, static_ip, subnet):
    """Configures the Windows (WDS) host network connection."""
    print("\n=== Configuring Windows Network (WDS) ===")   
    ip_cmd = f'netsh interface ip set address "{interface_name}" static {static_ip} {subnet}'
    ip_cmd_escaped = ip_cmd.replace("\"", "\\\\\\\"") # Escape all sub scopes that appear later when running as admni in powershell
    run_command(ip_cmd_escaped, f"Setting static IP {static_ip} on '{interface_name}'", as_admin=True)
    # 2. Set network category to Private
    ps_cmd = f'Set-NetConnectionProfile -Name "{network_name}" -NetworkCategory Private'
    ps_cmd_escaped = ps_cmd.replace("\"", "\\\\\\\"") # Escape all sub scopes that appear later when running as admni in powershell
    run_command(ps_cmd_escaped, f"Setting '{network_name}' profile to Private", as_admin=True)

def is_windows_network_ready(interface_name, expected_ip):
    """
    Checks if the Windows network interface is correctly configured with the 
    expected static IP and Private network profile.
    """
    if platform.system() != "Windows":
        return True

    try:
        # 1. Check if the expected static IP is assigned to the interface
        ip_cmd = f'powershell -Command "(Get-NetIPAddress -InterfaceAlias \'{interface_name}\' -AddressFamily IPv4 -ErrorAction SilentlyContinue).IPAddress"'
        ip_res = subprocess.run(ip_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if expected_ip != ip_res.stdout[:-1]:
            return False

        # 2. Check if the network profile is set to Private (required for discovery)
        cat_cmd = f'powershell -Command "(Get-NetConnectionProfile -InterfaceAlias \'{interface_name}\' -ErrorAction SilentlyContinue).NetworkCategory"'
        cat_res = subprocess.run(cat_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if "Private" != cat_res.stdout[:-1]:
            return False

        return True
    except Exception:
        return False