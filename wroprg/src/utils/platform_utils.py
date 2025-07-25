"""Platform detection utilities for cross-platform development."""
import platform
import os
import logging

logger = logging.getLogger(__name__)

def is_raspberry_pi():
    """Check if running on Raspberry Pi."""
    try:
        # Check for Raspberry Pi specific files
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model', 'r', encoding='utf-8') as f:
                model = f.read()
                return 'raspberry pi' in model.lower()

        # Fallback: check CPU info
        if os.path.exists('/proc/cpuinfo'):
            with open('/proc/cpuinfo', 'r', encoding='utf-8') as f:
                cpuinfo = f.read()
                return 'bcm' in cpuinfo.lower() and 'arm' in cpuinfo.lower()

    except (OSError, IOError):
        pass

    return False

def get_platform_info():
    """Get detailed platform information."""
    info = {
        'system': platform.system(),
        'machine': platform.machine(),
        'platform': platform.platform(),
        'is_raspberry_pi': is_raspberry_pi(),
        'python_version': platform.python_version()
    }

    logger.info("Platform info: %s", info)
    return info

def should_use_mock_hardware():
    """Determine if mock hardware implementations should be used."""
    return not is_raspberry_pi()
