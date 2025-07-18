#!/usr/bin/env python3
# Author: PamirAI
# Date: 2025-07-13
# Version: 0.2.3
# Description: Python-based upload script for RP2040 SAM firmware with robust UF2 flashing

import os
import time
import shutil
import subprocess
import signal
import sys
import argparse
from pathlib import Path

# Configuration Constants
UF2_DIRECTORY = (
    "/Users/chengmingzhang/CodingProjects/Software/Distiller-SAM-Firmware/ULP"
)
VOLUME_PATHS = ["/Volumes/RPI-RP2 1", "/Volumes/RPI-RP2"]  # Try numbered volume first
UART_PORT_PATTERN = "/dev/tty.usb*"

# Files to upload via ampy (located in same directory as this script)
PYTHON_FILES = [
    "bin/loading1.bin",
    "bin/loading2.bin",
    "eink_driver_sam.py",
    "pamir_uart_protocols.py",
    "neopixel_controller.py",
    "power_manager.py",
    "battery.py",
    "debug_handler.py",
    "uart_handler.py",
    "threaded_task_manager.py",
    "main.py",
]

# UF2 Files (located in UF2_DIRECTORY)
FLASH_NUKE_UF2 = "flash_nuke.uf2"
MICROPYTHON_UF2 = "RPI_PICO-20240222-v1.22.2.uf2"

# AppleScript to dismiss macOS notifications
APPLESCRIPT_CODE = """tell application "System Events"
    try
        set _groups to groups of UI element 1 of scroll area 1 of group 1 of window "Notification Center" of application process "NotificationCenter"
        repeat with _group in _groups
            set temp to value of static text 1 of _group
            if temp contains "Disk Not Ejected Properly" then
                perform (first action of _group where description is "Close")
            end if
        end repeat
    end try
end tell"""


def execute_applescript(code):
    """Execute AppleScript code to dismiss notifications"""
    subprocess.run(
        ["osascript", "-e", code], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


def exit_gracefully(signum, frame):
    """Handle Ctrl+C gracefully"""
    print("\nExiting...")
    sys.exit(0)


def check_dependencies(compile_mode=False):
    """Check if required tools are installed"""
    dependencies = ["ampy", "brew"]
    if compile_mode:
        dependencies.append("mpy-cross")

    missing = []

    for dep in dependencies:
        if not shutil.which(dep):
            missing.append(dep)

    if missing:
        print(f"Missing dependencies: {', '.join(missing)}")
        if "brew" in missing:
            print("Please install Homebrew first: https://brew.sh")
            return False
        if "ampy" in missing:
            print("Installing ampy...")
            try:
                subprocess.run(["pip3", "install", "adafruit-ampy"], check=True)
            except subprocess.CalledProcessError:
                print(
                    "Failed to install ampy. Please install manually: pip3 install adafruit-ampy"
                )
                return False
        if "mpy-cross" in missing:
            print("Installing mpy-cross...")
            try:
                subprocess.run(["pip3", "install", "mpy-cross"], check=True)
            except subprocess.CalledProcessError:
                print(
                    "Failed to install mpy-cross. Please install manually: pip3 install mpy-cross"
                )
                return False

    return True


def find_rp2_volume():
    """Find the active RPI-RP2 volume"""
    for volume_path in VOLUME_PATHS:
        if os.path.exists(volume_path):
            # Check if it's a valid RP2040 bootloader volume
            try:
                files = os.listdir(volume_path)
                if "INFO_UF2.TXT" in files:
                    return volume_path
            except (PermissionError, OSError):
                continue
    return None


def wait_for_rp2_device(timeout=60):
    """Wait for RPI-RP2 device to appear"""
    print("Waiting for RPI-RP2 device...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        volume_path = find_rp2_volume()
        if volume_path:
            print(f"RPI-RP2 device found at: {volume_path}")
            return volume_path
        time.sleep(1)

    print(f"Timeout: RPI-RP2 device not found after {timeout} seconds")
    return None


def wait_for_rp2_disappear(timeout=30):
    """Wait for RPI-RP2 device to disappear"""
    print("Waiting for RPI-RP2 device to disappear...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        if not find_rp2_volume():
            print("RPI-RP2 device disappeared")
            return True
        time.sleep(1)

    print(f"Timeout: RPI-RP2 device still present after {timeout} seconds")
    return False


def flash_uf2_file_applescript(uf2_filename, description):
    """Flash a UF2 file using AppleScript drag-and-drop automation"""
    uf2_path = os.path.join(UF2_DIRECTORY, uf2_filename)

    if not os.path.exists(uf2_path):
        print(f"Error: {uf2_filename} not found in {UF2_DIRECTORY}")
        return False

    # Find the current active volume
    volume_path = find_rp2_volume()
    if not volume_path:
        print("No RPI-RP2 volume found")
        return False

    print(f"Copying {description} using AppleScript to {volume_path}...")

    # AppleScript to automate drag and drop
    applescript_copy = f"""
    tell application "Finder"
        -- Ensure Finder is active
        activate
        
        -- Wait a moment for Finder to be ready
        delay 1
        
        -- Open the source directory containing the UF2 file
        set sourceFolder to POSIX file "{UF2_DIRECTORY}" as alias
        set sourceWindow to open sourceFolder
        
        -- Wait for source window to open
        delay 2
        
        -- Open the target volume using POSIX path
        set targetFolder to POSIX file "{volume_path}" as alias
        set targetWindow to open targetFolder
        
        -- Wait for target window to open
        delay 2
        
        -- Select the UF2 file in source window
        tell sourceWindow
            select file "{uf2_filename}"
        end tell
        
        -- Copy the file to the target
        tell sourceWindow
            duplicate selection to targetFolder
        end tell
        
        -- Wait for copy to complete
        delay 3
        
        -- Close windows
        close sourceWindow
        close targetWindow
        
        return true
    end tell
    """

    try:
        # Execute the AppleScript
        result = subprocess.run(
            ["osascript", "-e", applescript_copy],
            capture_output=True,
            text=True,
            timeout=60,
        )

        if result.returncode == 0:
            print(f"{description} copied successfully via AppleScript")
            return True
        else:
            print(f"AppleScript error: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print("AppleScript timeout - operation took too long")
        return False
    except Exception as e:
        print(f"Error executing AppleScript: {e}")
        return False


def flash_uf2_file(uf2_filename, description):
    """Flash a UF2 file to the RP2040 - tries AppleScript first, then fallback methods"""
    uf2_path = os.path.join(UF2_DIRECTORY, uf2_filename)

    if not os.path.exists(uf2_path):
        print(f"Error: {uf2_filename} not found in {UF2_DIRECTORY}")
        return False

    # Try AppleScript method first (most reliable for macOS)
    if flash_uf2_file_applescript(uf2_filename, description):
        return True

    print(f"AppleScript failed, trying manual copy methods...")
    print(f"Copying {description}...")

    try:
        # Fallback: Try multiple copy methods
        dest_path = os.path.join(VOLUME_PATH, uf2_filename)

        # Method 1: Try standard copy
        try:
            shutil.copy2(uf2_path, dest_path)
            os.chmod(dest_path, 0o644)
        except (PermissionError, OSError):
            # Method 2: Try using subprocess cp command
            print("Trying cp command...")
            result = subprocess.run(
                ["cp", uf2_path, dest_path], capture_output=True, text=True
            )
            if result.returncode != 0:
                raise PermissionError(f"Copy command failed: {result.stderr}")

        # Verify the file was copied
        if os.path.exists(dest_path):
            print(f"{description} copied successfully")
            return True
        else:
            raise FileNotFoundError("File copy appeared to succeed but file not found")

    except Exception as e:
        print(f"All copy methods failed: {e}")
        print("\nManual steps:")
        print(f"1. Open Finder and navigate to: {UF2_DIRECTORY}")
        print(f"2. Drag {uf2_filename} to RPI-RP2 volume")
        print("3. Press Enter when done, or Ctrl+C to exit")

        try:
            input()  # Wait for user confirmation
            return True
        except KeyboardInterrupt:
            return False


def flash_firmware_mode(wipe_first=False):
    """Flash firmware in --wipe or --first mode"""
    print("=" * 50)
    if wipe_first:
        print("WIPE MODE: Will flash nuke then MicroPython")
    else:
        print("FIRST MODE: Will flash MicroPython firmware")
    print("=" * 50)

    # Wait for device in bootloader mode
    volume_path = wait_for_rp2_device()
    if not volume_path:
        return False

    # Update global volume path for fallback methods
    global VOLUME_PATH
    VOLUME_PATH = volume_path

    if wipe_first:
        # Flash nuke first
        if not flash_uf2_file(FLASH_NUKE_UF2, "flash nuke"):
            return False

        # Wait for device to disappear and reappear
        if not wait_for_rp2_disappear():
            return False

        print("Waiting for device to reappear...")
        time.sleep(5)

        volume_path = wait_for_rp2_device()
        if not volume_path:
            return False
        VOLUME_PATH = volume_path

    # Flash MicroPython firmware
    if not flash_uf2_file(MICROPYTHON_UF2, "MicroPython firmware"):
        return False

    # Wait for device to disappear and initialize
    wait_for_rp2_disappear()
    print("Waiting for device to initialize...")
    time.sleep(3)

    print("Firmware flashing completed!")
    return True


def find_uart_port():
    """Find the UART port for ampy"""
    import glob

    ports = glob.glob(UART_PORT_PATTERN)

    if not ports:
        print(f"No UART ports found matching pattern: {UART_PORT_PATTERN}")
        return None

    # Return the first available port
    return ports[0]


def compile_python_files():
    """Compile Python files to .mpy bytecode using mpy-cross"""
    print("=" * 50)
    print("COMPILING PYTHON FILES TO BYTECODE")
    print("=" * 50)

    script_dir = Path(__file__).parent
    compiled_files = []

    for filename in PYTHON_FILES:
        file_path = script_dir / filename

        if not file_path.exists():
            print(f"Warning: {filename} not found, skipping...")
            continue

        # Only compile .py files
        if filename.endswith(".py"):
            print(f"Compiling {filename}...")

            try:
                # Run mpy-cross command
                mpy_filename = filename[:-3] + ".mpy"  # Replace .py with .mpy
                mpy_path = script_dir / mpy_filename

                cmd = ["mpy-cross", str(file_path), "-o", str(mpy_path)]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

                if result.returncode != 0:
                    print(f"Error compiling {filename}: {result.stderr}")
                    return None

                print(f"  -> {mpy_filename}")
                compiled_files.append(mpy_filename)

            except subprocess.TimeoutExpired:
                print(f"Timeout compiling {filename}")
                return None
            except Exception as e:
                print(f"Error compiling {filename}: {e}")
                return None
        else:
            # Non-Python files (like .bin) - keep as-is
            compiled_files.append(filename)

    print(
        f"\nCompilation completed! {len([f for f in compiled_files if f.endswith('.mpy')])} files compiled."
    )
    return compiled_files


def upload_python_files(file_list=None):
    """Upload Python files using ampy"""
    if file_list is None:
        file_list = PYTHON_FILES
        upload_type = "PYTHON FILES"
    else:
        upload_type = "COMPILED FILES"

    print("=" * 50)
    print(f"UPLOADING {upload_type}")
    print("=" * 50)

    # Find UART port
    port = find_uart_port()
    if not port:
        print("Please connect RP2040 via USB and ensure MicroPython is running")
        return False

    print(f"Using port: {port}")

    # Get script directory
    script_dir = Path(__file__).parent

    # Upload each file
    total_files = len(file_list)
    for i, filename in enumerate(file_list):
        file_path = script_dir / filename

        if not file_path.exists():
            print(f"Warning: {filename} not found, skipping...")
            continue

        print(f"Uploading {filename} ({i+1}/{total_files})")

        try:
            # Run ampy command
            cmd = ["ampy", "--port", port, "put", str(file_path)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode != 0:
                print(f"Error uploading {filename}: {result.stderr}")
                return False

            # Progress indicator
            progress = int((i + 1) * 50 / total_files)
            bar = "#" * progress + "-" * (50 - progress)
            print(f"Progress: [{bar}] {int((i+1)*100/total_files)}%")

        except subprocess.TimeoutExpired:
            print(f"Timeout uploading {filename}")
            return False
        except Exception as e:
            print(f"Error uploading {filename}: {e}")
            return False

    print("\nAll files uploaded successfully!")
    return True


def main():
    """Main function"""
    # Register signal handler
    signal.signal(signal.SIGINT, exit_gracefully)

    # Parse arguments
    parser = argparse.ArgumentParser(description="RP2040 SAM Firmware Upload Tool")
    parser.add_argument(
        "--wipe", action="store_true", help="Wipe flash and install MicroPython"
    )
    parser.add_argument(
        "--first",
        action="store_true",
        help="First time flash (install MicroPython only)",
    )
    parser.add_argument(
        "--compile",
        action="store_true",
        help="Compile Python files to bytecode (.mpy) before upload",
    )

    args = parser.parse_args()

    print("RP2040 SAM Firmware Upload Tool")
    print("Press CTRL+C to exit")
    print("Warning: Exiting while flashing could damage the RP2040")
    print()

    # Check dependencies
    if not check_dependencies(compile_mode=args.compile):
        sys.exit(1)

    # Check if UF2 directory exists
    if not os.path.exists(UF2_DIRECTORY):
        print(f"Error: UF2 directory not found: {UF2_DIRECTORY}")
        sys.exit(1)

    success = True

    # Handle firmware flashing modes
    if args.wipe or args.first:
        success = flash_firmware_mode(wipe_first=args.wipe)
        if not success:
            print("Firmware flashing failed!")
            sys.exit(1)

    # Handle compilation and upload
    if success:
        if args.compile:
            # Compile Python files first
            compiled_files = compile_python_files()
            if compiled_files is None:
                print("Compilation failed!")
                sys.exit(1)
            # Upload compiled files
            success = upload_python_files(compiled_files)
        else:
            # Upload source Python files
            success = upload_python_files()

    # Clean up notifications
    execute_applescript(APPLESCRIPT_CODE)

    if success:
        print("\n" + "=" * 50)
        print("UPLOAD COMPLETED SUCCESSFULLY!")
        print("=" * 50)
    else:
        print("\nUpload failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
