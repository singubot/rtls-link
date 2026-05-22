#!/usr/bin/env python3
"""
pre_build_features.py - PlatformIO pre-build script for feature configuration

This script runs before compilation and:
1. Reads user_defines.txt for feature flags
2. Validates the configuration (early error detection)
3. Injects flags into the build environment

Usage:
    Add to platformio.ini:
    extra_scripts = pre:scripts/pre_build_features.py

Configuration file format (user_defines.txt):
    ; Comment line (ignored)
    # Also a comment (ignored)
    -D USE_WIFI
    -D USE_MAVLINK_POSITION
    -DUSE_CONSOLE  ; Inline comment
"""

import os
import sys

# PlatformIO SCons environment import
Import("env")


def parse_user_defines(filepath):
    """
    Parse user_defines.txt and extract -D flags.

    Args:
        filepath: Path to user_defines.txt

    Returns:
        List of -D flag strings (e.g., ["-D USE_WIFI", "-D USE_MAVLINK"])
    """
    flags = []

    if not os.path.exists(filepath):
        print(f"[FEATURES] No user_defines.txt found at {filepath}")
        print("[FEATURES] Using default configuration (all features enabled)")
        return flags

    with open(filepath, 'r') as f:
        for line_num, line in enumerate(f, 1):
            # Strip whitespace
            line = line.strip()

            # Skip empty lines
            if not line:
                continue

            # Skip comment lines
            if line.startswith(';') or line.startswith('#'):
                continue

            # Handle inline comments
            if ';' in line:
                line = line.split(';')[0].strip()
            if '#' in line:
                line = line.split('#')[0].strip()

            # Skip if nothing left after removing comments
            if not line:
                continue

            # Validate -D flag format
            if line.startswith('-D'):
                flags.append(line)
            else:
                print(f"[FEATURES] Warning: Line {line_num} ignored (not a -D flag): {line}")

    return flags


def extract_flag_name(flag_str):
    """
    Extract the flag name from a -D flag string.

    Args:
        flag_str: String like "-D USE_WIFI" or "-DUSE_WIFI" or "-D USE_WIFI=1"

    Returns:
        Flag name without -D prefix and without value (e.g., "USE_WIFI")
    """
    # Remove -D prefix (with or without space)
    if flag_str.startswith('-D '):
        name = flag_str[3:]
    elif flag_str.startswith('-D'):
        name = flag_str[2:]
    else:
        name = flag_str

    # Remove any value assignment
    if '=' in name:
        name = name.split('=')[0]

    return name.strip()


def get_board_define(env):
    """
    Detect the board define from the environment's build flags.

    Returns:
        Board define string (e.g., 'MAKERFABS_ESP32_BOARD', 'ESP32S3_UWB_BOARD')
        or None if not detected.
    """
    build_flags = env.get('BUILD_FLAGS', [])
    for flag in build_flags:
        flag_str = str(flag)
        if 'MAKERFABS_ESP32_BOARD' in flag_str:
            return 'MAKERFABS_ESP32_BOARD'
        if 'ESP32S3_UWB_BOARD' in flag_str:
            return 'ESP32S3_UWB_BOARD'
    return None


def validate_features(flags, board_define=None):
    """
    Validate feature dependencies, conflicts, and board compatibility.

    This provides early error detection before compilation starts.
    The C++ headers also validate, but this gives cleaner error messages.

    Args:
        flags: List of -D flag strings
        board_define: Board define string (e.g., 'ESP32S3_UWB_BOARD') or None

    Returns:
        Tuple of (errors list, warnings list)
    """
    # Extract just the flag names for easier checking
    flag_set = set(extract_flag_name(f) for f in flags)

    errors = []
    warnings = []

    # === WIFI DEPENDENCIES ===
    wifi_features = [
        'USE_WIFI_WEBSERVER',
        'USE_WIFI_TCP_LOGGING',
        'USE_WIFI_UART_BRIDGE',
        'USE_WIFI_DISCOVERY',
        'USE_WIFI_MDNS'
    ]
    for feat in wifi_features:
        if feat in flag_set and 'USE_WIFI' not in flag_set:
            errors.append(f"{feat} requires USE_WIFI")

    # === MAVLINK DEPENDENCIES ===
    mavlink_features = [
        'USE_MAVLINK_POSITION',
        'USE_MAVLINK_HEARTBEAT',
        'USE_MAVLINK_ORIGIN',
        'USE_MAVLINK_COVARIANCE',
        'USE_MAVLINK_RANGEFINDER'
    ]
    for feat in mavlink_features:
        if feat in flag_set and 'USE_MAVLINK' not in flag_set:
            errors.append(f"{feat} requires USE_MAVLINK")

    # === RTLSLINK BEACON DEPENDENCIES ===
    if 'USE_RTLSLINK_BEACON_BACKEND' in flag_set and 'USE_UWB_MODE_TDOA_TAG' not in flag_set:
        errors.append("USE_RTLSLINK_BEACON_BACKEND requires USE_UWB_MODE_TDOA_TAG")

    # === CONSOLE DEPENDENCIES ===
    console_features = [
        'USE_CONSOLE_PARAM_RW',
        'USE_CONSOLE_CONFIG_MGMT',
        'USE_CONSOLE_LED_CONTROL'
    ]
    for feat in console_features:
        if feat in flag_set and 'USE_CONSOLE' not in flag_set:
            errors.append(f"{feat} requires USE_CONSOLE")

    # === OTA DEPENDENCIES ===
    if 'USE_OTA_WEB' in flag_set and 'USE_OTA' not in flag_set:
        errors.append("USE_OTA_WEB requires USE_OTA")
    if 'USE_OTA_WEB' in flag_set and 'USE_WIFI_WEBSERVER' not in flag_set:
        errors.append("USE_OTA_WEB requires USE_WIFI_WEBSERVER")

    # === TAG MODE REQUIRES OUTPUT ===
    tag_modes = ['USE_UWB_MODE_TDOA_TAG']
    has_tag_mode = any(m in flag_set for m in tag_modes)
    has_output = 'USE_MAVLINK' in flag_set or 'USE_RTLSLINK_BEACON_BACKEND' in flag_set

    if has_tag_mode and not has_output:
        errors.append("Tag modes require USE_MAVLINK or USE_RTLSLINK_BEACON_BACKEND for position output")

    # === AT LEAST ONE UWB MODE ===
    uwb_modes = [
        'USE_UWB_MODE_TDOA_ANCHOR',
        'USE_UWB_MODE_TDOA_TAG'
    ]
    if flag_set and not any(m in flag_set for m in uwb_modes):
        # Only error if user explicitly defined features but forgot UWB modes
        warnings.append("No TDoA UWB mode enabled - at least one USE_UWB_MODE_TDOA_* is recommended")

    # === BOARD-SPECIFIC COMPATIBILITY ===
    if board_define is not None:
        if 'USE_MAVLINK_RANGEFINDER' in flag_set and board_define != 'ESP32S3_UWB_BOARD':
            errors.append(
                f"USE_MAVLINK_RANGEFINDER is only supported on ESP32S3_UWB_BOARD "
                f"(current board: {board_define})"
            )

        if 'USE_DYNAMIC_ANCHOR_POSITIONS' in flag_set and board_define == 'MAKERFABS_ESP32_BOARD':
            errors.append(
                f"USE_DYNAMIC_ANCHOR_POSITIONS is not supported on {board_define} "
                f"(insufficient DRAM for Eigen-based position calculator)"
            )

    return errors, warnings


def print_feature_summary(flags, source_file="user_defines.txt"):
    """Print a summary of enabled features."""
    if not flags:
        print("[FEATURES] Configuration: Using defaults (all features)")
        return

    print(f"[FEATURES] Configuration: {len(flags)} features from {source_file}")

    # Group by category for cleaner output
    categories = {
        'WiFi': [],
        'MAVLink': [],
        'Console': [],
        'UWB': [],
        'OTA': [],
        'Other': []
    }

    for flag in flags:
        name = extract_flag_name(flag)
        if name.startswith('USE_WIFI'):
            categories['WiFi'].append(name)
        elif name.startswith('USE_MAVLINK'):
            categories['MAVLink'].append(name)
        elif name.startswith('USE_CONSOLE'):
            categories['Console'].append(name)
        elif name.startswith('USE_UWB'):
            categories['UWB'].append(name)
        elif name.startswith('USE_OTA'):
            categories['OTA'].append(name)
        else:
            categories['Other'].append(name)

    for category, names in categories.items():
        if names:
            print(f"[FEATURES]   {category}: {', '.join(names)}")


# =============================================================================
# MAIN EXECUTION
# =============================================================================

project_dir = env.get('PROJECT_DIR', os.getcwd())

# Read per-board user_defines path from platformio.ini (custom_user_defines option).
# Falls back to root user_defines.txt for backward compatibility.
user_defines_file = env.GetProjectOption("custom_user_defines", "user_defines.txt")
user_defines_path = os.path.join(project_dir, user_defines_file)

# Parse user defines
user_flags = parse_user_defines(user_defines_path)

# Detect board define for board-specific validation
board_define = get_board_define(env)

# Validate configuration (including board compatibility)
errors, warnings = validate_features(user_flags, board_define)

# Print warnings (non-fatal)
for warn in warnings:
    print(f"[FEATURES] Warning: {warn}")

# Print errors and exit if any
if errors:
    print("[FEATURES] Configuration errors detected:")
    for err in errors:
        print(f"[FEATURES]   ERROR: {err}")
    print(f"[FEATURES] Please fix {user_defines_file} and rebuild")
    sys.exit(1)

# Print feature summary
print_feature_summary(user_flags, user_defines_file)

# Inject flags into build environment
if user_flags:
    env.Append(BUILD_FLAGS=user_flags)
    print(f"[FEATURES] Injected {len(user_flags)} build flags")
