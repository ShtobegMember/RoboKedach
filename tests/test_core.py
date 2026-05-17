import pytest
import struct
import json
from unittest.mock import patch, mock_open

# Import target functions
from core.network_utils import pack_vm_data, unpack_vm_data, get_cyclonedds_config, VM_DATA_SIZE
from core.config_loader import get_config

# ==============================================================================
# TEST: network_utils.py
# ==============================================================================

def test_pack_vm_data():
    """Verify that voltage and current are packed into exactly 8 bytes (little-endian floats)."""
    voltage = 12.6
    current = 2.5
    packed = pack_vm_data(voltage, current)
    
    assert len(packed) == 8
    assert packed == struct.pack('<2f', voltage, current)


def test_unpack_vm_data():
    """Verify that a valid 8-byte payload unpacks back into the correct floats."""
    valid_data = struct.pack('<2f', 11.1, 0.5)
    voltage, current = unpack_vm_data(valid_data)
    
    assert voltage == pytest.approx(11.1)
    assert current == pytest.approx(0.5)


def test_unpack_vm_data_invalid_length():
    """Verify that attempting to unpack the wrong amount of data raises a ValueError."""
    invalid_data = b'\x00' * 7  # Only 7 bytes instead of 8
    
    with pytest.raises(ValueError, match=f"Expected {VM_DATA_SIZE} bytes"):
        unpack_vm_data(invalid_data)


def test_get_cyclonedds_config():
    """Verify the CycloneDDS XML string is formatted correctly with the pinned IP."""
    target_ip = "192.168.1.100"
    xml_output = get_cyclonedds_config(target_ip)
    
    assert f"<NetworkInterfaceAddress>{target_ip}</NetworkInterfaceAddress>" in xml_output
    assert "<AllowMulticast>spdp</AllowMulticast>" in xml_output


# ==============================================================================
# TEST: config_loader.py
# ==============================================================================

def test_get_config_success():
    """Verify that a valid JSON file is successfully loaded and returned as a dict."""
    mock_json_content = '{"network": {"rpi_ip": "10.0.0.1"}, "test_key": 42}'
    
    # Mock os.path.exists to always return True so it "finds" the file immediately
    with patch('os.path.exists', return_value=True):
        # Mock the built-in open() function to return our fake JSON string
        with patch('builtins.open', mock_open(read_data=mock_json_content)):
            config = get_config()
            
            assert isinstance(config, dict)
            assert config["network"]["rpi_ip"] == "10.0.0.1"
            assert config["test_key"] == 42


def test_get_config_file_not_found():
    """Verify that a FileNotFoundError is raised if config.json is not found anywhere."""
    # Mock os.path.exists to always return False for all search paths
    with patch('os.path.exists', return_value=False):
        with pytest.raises(FileNotFoundError, match="Could not find config.json"):
            get_config()


def test_get_config_invalid_json():
    """Verify that a ValueError is raised if the file is found but contains corrupted JSON."""
    bad_json_content = '{ network: "missing_quotes", broken_data }'
    
    with patch('os.path.exists', return_value=True):
        with patch('builtins.open', mock_open(read_data=bad_json_content)):
            with pytest.raises(ValueError, match="Failed to parse JSON"):
                get_config()