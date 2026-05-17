"""
config_loader.py - Centralized configuration manager.
Handles secure path resolution for config.json regardless of where the entry script is executed.
"""

import os
import json

def get_config() -> dict:
    """
    Search for and load the system configuration from config.json.
    Resolves paths automatically to support both the new modular architecture
    and backwards compatibility with the original layout.
    """
    # Base directory is one level up from the 'core' folder (the project root)
    base_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
    
    locations = [
        os.path.join(base_dir, "config", "config.json"),   # Target new architecture
        os.path.join(base_dir, "config.json"),             # Target root directory
        os.path.join(base_dir, "PRODUCT", "config.json"),  # Legacy location
        "config.json"                                      # Current working directory fallback
    ]
    
    for loc in locations:
        if os.path.exists(loc):
            try:
                with open(loc, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError as e:
                raise ValueError(f"Failed to parse JSON in {loc}: {e}")
                
    raise FileNotFoundError(f"Could not find config.json in any of the expected locations: {locations}")

# Load once when the module is imported to act as a singleton
CONFIG = get_config()