"""
Pamir AI Signal Aggregation Module (SAM) Version Module
======================================================

This module defines version information for the Pamir AI SAM firmware
and provides helper functions for version compatibility checks.

The versioning system follows semantic versioning principles:
- MAJOR version for incompatible API changes
- MINOR version for new functionality in a backward-compatible manner
- PATCH version for backward-compatible bug fixes

Version History:
--------------
0.1.0  - Initial development version
"""

# Firmware version constants
VERSION_MAJOR = 0
VERSION_MINOR = 1
VERSION_PATCH = 0
VERSION_STRING = f"{VERSION_MAJOR}.{VERSION_MINOR}.{VERSION_PATCH}"

# Compatibility flags - features available in specific versions
FEATURE_BASIC = (0, 1, 0)  # Basic functionality (buttons, LEDs, power, version)


def version_to_tuple(version_string):
    """Convert a version string to a tuple of integers

    Args:
        version_string: Version string in format "major.minor.patch"

    Returns:
        Tuple of (major, minor, patch) as integers
    """
    try:
        parts = version_string.split(".")
        return (int(parts[0]), int(parts[1]), int(parts[2]))
    except (ValueError, IndexError):
        return (0, 0, 0)  # Default to 0.0.0 for invalid version strings


def check_compatibility(host_version, feature=None):
    """Check if the host version is compatible with a specific feature

    Args:
        host_version: Host version as tuple (major, minor, patch) or string
        feature: Optional feature version tuple to check against

    Returns:
        True if compatible, False otherwise
    """
    # Convert string to tuple if needed
    if isinstance(host_version, str):
        host_version = version_to_tuple(host_version)

    # Default to checking against current firmware version if no feature specified
    if feature is None:
        feature = (VERSION_MAJOR, VERSION_MINOR, 0)  # Ignore patch version

    # Compare major and minor versions
    return (host_version[0] > feature[0]) or (
        host_version[0] == feature[0] and host_version[1] >= feature[1]
    )


def get_version_info():
    """Get current version information as a dictionary

    Returns:
        Dictionary with version components
    """
    return {
        "major": VERSION_MAJOR,
        "minor": VERSION_MINOR,
        "patch": VERSION_PATCH,
        "string": VERSION_STRING,
    }


def log_version_info(debug_print=None):
    """Log version information using the provided debug function

    Args:
        debug_print: Function to use for printing debug info
    """
    if debug_print:
        debug_print(f"Pamir AI SAM Firmware v{VERSION_STRING}")
        debug_print(f"  - Major: {VERSION_MAJOR}")
        debug_print(f"  - Minor: {VERSION_MINOR}")
        debug_print(f"  - Patch: {VERSION_PATCH}")
