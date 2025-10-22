#!/bin/bash

# PamirAI Firmware Loader

# Configuration constants
readonly FLASH_NUKE_UF2="ULP/flash_nuke.uf2"
readonly MICROPYTHON_UF2="ULP/RPI_PICO-20240222-v1.22.2.uf2"

# Timing constants
readonly DEVICE_STABILIZATION_DELAY=3  # USB enumeration + MicroPython boot time
readonly FLASH_NUKE_DURATION=12        # Flash erase time (typically 10-15 seconds)
readonly NORMAL_FLASH_DURATION=5       # UF2 bootloader flash time

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
RESET='\033[0m'

# Logging function
log() {
	local level=$1
	shift
	local message="$*"

	case $level in
		INFO)
			echo -e "${BLUE}[INFO]${RESET} ${message}"
			;;
		SUCCESS)
			echo -e "${GREEN}[SUCCESS]${RESET} ${message}"
			;;
		WARN)
			echo -e "${YELLOW}[WARN]${RESET} ${message}"
			;;
		ERROR)
			echo -e "${RED}[ERROR]${RESET} ${message}" >&2
			;;
	esac
}

# Usage
show_usage() {
	cat <<EOF
Usage: $0 [OPTIONS]

RP2040 firmware and file uploader using uvx mpremote:

  $0 --version BHV              Upload .py files to BHV version
  $0 --version BHV --compiled   Upload .mpy files from mpy/ directory
  $0 --first                    Flash MicroPython firmware only
  $0 --wipe --version BHV       Wipe device, flash MicroPython, upload files
  $0 --dry-run                  Check device status

Options:
  -h, --help        Show this help
  --first           Flash MicroPython firmware only
  --wipe            Wipe device and flash MicroPython
  --dry-run         Check device status without making changes
  --version VER     Specify firmware version (required for upload)
  --compiled        Upload .mpy files instead of .py files

Requirements:
  - uvx (required for mpremote)
  - RP2040 device connected

Instructions:
  1. Hold BOOTSEL while connecting for firmware flashing (.uf2)
  2. Connect normally for MicroPython file upload

EOF
}

# Global variables
FIRST_TIME_FLASH=false
WIPE_AND_FLASH=false
DRY_RUN=false
FIRMWARE_VERSION=""
USE_COMPILED=false

# Argument parsing
while [[ $# -gt 0 ]]; do
	case $1 in
	-h | --help)
		show_usage
		exit 0
		;;
	--first)
		FIRST_TIME_FLASH=true
		shift
		;;
	--wipe)
		WIPE_AND_FLASH=true
		shift
		;;
	--dry-run)
		DRY_RUN=true
		shift
		;;
	--version)
		FIRMWARE_VERSION="$2"
		shift 2
		;;
	--compiled)
		USE_COMPILED=true
		shift
		;;
	*)
		log ERROR "Unknown option: $1"
		show_usage
		exit 1
		;;
	esac
done

# Validation
# Conflict check
if [[ "$FIRST_TIME_FLASH" == true && "$WIPE_AND_FLASH" == true ]]; then
	log ERROR "Cannot use both --first and --wipe"
	exit 1
fi

# Dependency checks
check_uvx() {
	if ! command -v uvx &>/dev/null; then
		log ERROR "uvx is not installed"
		log INFO "Install uv from: https://docs.astral.sh/uv/"
		exit 1
	fi
	log INFO "Found uvx: $(command -v uvx)"
}

# System detection
detect_os() {
	case "$(uname -s)" in
	Linux*) echo "Linux" ;;
	Darwin*) echo "macOS" ;;
	*) echo "Unknown" ;;
	esac
}

# Device detection

# Verify MicroPython device is ready
verify_micropython_ready() {
	local device=$1
	uvx mpremote connect "$device" exec "print('OK')" &>/dev/null
}

# Find RPI-RP2 bootloader drive for UF2 flashing
find_rp2_drive() {
	local os=$(detect_os)
	local paths=()

	if [[ "$os" == "macOS" ]]; then
		paths=("/Volumes/RPI-RP2" "/Volumes/RPI-RP2 1")
	else
		paths=("/media/$USER/RPI-RP2" "/mnt/RPI-RP2" "/run/media/$USER/RPI-RP2")
	fi

	# Check direct paths
	for path in "${paths[@]}"; do
		if [[ -d "$path" && -w "$path" ]]; then
			# Validate it's actually an RP2 drive
			if [[ -f "$path/INDEX.HTM" ]] || [[ -f "$path/INFO_UF2.TXT" ]]; then
				echo "$path"
				return 0
			fi
		fi
	done

	return 1
}

# Find MicroPython device using mpremote, filtered for RP2040/RP2350
find_micropython_device() {
	local os=$(detect_os)

	# Use mpremote to list devices
	local devices=$(uvx mpremote devs 2>/dev/null)

	if [[ -z "$devices" ]]; then
		return 1
	fi

	# Filter devices based on OS and device type
	local result=""
	if [[ "$os" == "macOS" ]]; then
		# On macOS, look for tty.usbmodem*
		result=$(echo "$devices" | grep -E "tty\.usbmodem" 2>/dev/null | head -1 | awk '{print $1}')
	else
		# On Linux, look for ttyACM*
		result=$(echo "$devices" | grep -E "ttyACM" 2>/dev/null | head -1 | awk '{print $1}')
	fi

	if [[ -n "$result" ]]; then
		echo "$result"
		return 0
	fi

	return 1
}

# Wait for device with timeout
wait_for_device() {
	local mode="$1"
	local timeout=30
	local count=0

	log INFO "Waiting for device in $mode mode (timeout: ${timeout}s)"

	while [[ $count -lt $timeout ]]; do
		local device=""
		if [[ "$mode" == "bootloader" ]]; then
			device=$(find_rp2_drive)
			if [[ -n "$device" ]]; then
				log SUCCESS "Device found in bootloader mode: $device"
				return 0
			fi
		else
			device=$(find_micropython_device)
			if [[ -n "$device" ]]; then
				# Verify device is actually ready
				sleep $DEVICE_STABILIZATION_DELAY
				if verify_micropython_ready "$device"; then
					log SUCCESS "MicroPython device ready: $device"
					return 0
				fi
			fi
		fi

		sleep 1
		((count++))
		if [[ $((count % 10)) -eq 0 ]]; then
			log INFO "Still waiting... ($count/$timeout seconds)"
		fi
	done

	log ERROR "Timeout waiting for $mode device"
	return 1
}

# Check device status
check_device_status() {
	log INFO "=== DEVICE STATUS ==="

	# Check bootloader mode
	local rp2_drive=$(find_rp2_drive)
	if [[ -n "$rp2_drive" ]]; then
		log SUCCESS "RP2040 in BOOTLOADER mode: $rp2_drive"
		return 0
	fi

	# Check MicroPython mode
	local device=$(find_micropython_device)
	if [[ -n "$device" ]]; then
		log SUCCESS "MicroPython device found: $device"

		# Test connection
		if verify_micropython_ready "$device"; then
			log SUCCESS "MicroPython responding"
			return 0
		else
			log ERROR "Device found but not responding"
			return 1
		fi
	fi

	log ERROR "No RP2040 device found"
	log INFO "Connect device: BOOTSEL for firmware, normal for files"
	return 1
}

# UF2 flashing
flash_uf2() {
	local file="$1"
	local is_flash_nuke=false

	# Check if this is flash_nuke
	if [[ "$file" == "$FLASH_NUKE_UF2" ]]; then
		is_flash_nuke=true
	fi

	# Validate file exists
	if [[ ! -f "$file" ]]; then
		log ERROR "UF2 file not found: $file"
		return 1
	fi

	# Find bootloader drive
	local rp2_drive=$(find_rp2_drive)
	if [[ -z "$rp2_drive" ]]; then
		log ERROR "Device not in bootloader mode"
		log INFO "Hold BOOTSEL button while connecting USB"
		return 1
	fi

	echo -ne "${BLUE}[INFO]${RESET} Flashing $(basename "$file")"

	if cp "$file" "$rp2_drive/"; then
		echo ""

		# Flash nuke takes longer to erase
		if [[ "$is_flash_nuke" == true ]]; then
			log INFO "Erasing flash memory (this takes ${FLASH_NUKE_DURATION} seconds)"
			# Show progress during erase
			for ((i=0; i<FLASH_NUKE_DURATION; i++)); do
				sleep 1
				echo -n "."
			done
			echo ""
		else
			# Show progress during normal flash
			for ((i=0; i<NORMAL_FLASH_DURATION; i++)); do
				sleep 1
				echo -n "."
			done
			echo ""
		fi

		log SUCCESS "Flashed successfully"
		return 0
	else
		echo ""
		log ERROR "Flash failed"
		return 1
	fi
}

# File upload with mpremote
get_source_directory() {
	if [[ "$USE_COMPILED" == true ]]; then
		echo "src/${FIRMWARE_VERSION}/mpy"
	else
		echo "src/${FIRMWARE_VERSION}"
	fi
}

get_file_extension() {
	if [[ "$USE_COMPILED" == true ]]; then
		echo "mpy"
	else
		echo "py"
	fi
}

# Upload files using mpremote
upload_files() {
	local source_dir=$(get_source_directory)
	local file_ext=$(get_file_extension)

	# Validate source directory
	if [[ ! -d "$source_dir" ]]; then
		log ERROR "Source directory not found: $source_dir"
		return 1
	fi

	# Find device
	local device=$(find_micropython_device)
	if [[ -z "$device" ]]; then
		log ERROR "MicroPython device not found"
		return 1
	fi

	log INFO "Using device: $device"
	log INFO "Source directory: $source_dir"

	# Get list of files to upload
	local python_files=()
	while IFS= read -r -d '' file; do
		python_files+=("$file")
	done < <(find "$source_dir" -maxdepth 1 -name "*.${file_ext}" -type f -print0)

	# Get list of binary files (always from bin/)
	local bin_dir="${source_dir%/mpy}/bin"
	[[ "$USE_COMPILED" == true ]] && bin_dir="src/${FIRMWARE_VERSION}/bin"

	local bin_files=()
	if [[ -d "$bin_dir" ]]; then
		while IFS= read -r -d '' file; do
			bin_files+=("$file")
		done < <(find "$bin_dir" -name "*.bin" -type f -print0)
	fi

	local total_files=$((${#python_files[@]} + ${#bin_files[@]}))

	if [[ $total_files -eq 0 ]]; then
		log ERROR "No files found to upload"
		return 1
	fi

	log INFO "Found $total_files files to upload"

	# Upload Python/MicroPython files to root
	for file in "${python_files[@]}"; do
		local filename=$(basename "$file")
		echo -ne "${BLUE}[INFO]${RESET} Uploading $filename"

		if uvx mpremote connect "$device" cp "$file" ":$filename" &>/dev/null; then
			echo -n "."
			echo ""
		else
			echo ""
			log ERROR "Failed to upload $filename"
			return 1
		fi
	done

	# Upload bin directory if it exists
	if [[ ${#bin_files[@]} -gt 0 ]]; then
		log INFO "Uploading bin/ directory"

		# Create bin directory on device
		if ! uvx mpremote connect "$device" mkdir :bin &>/dev/null; then
			log WARN "bin/ directory may already exist"
		fi

		# Upload each binary file
		for file in "${bin_files[@]}"; do
			local filename=$(basename "$file")
			echo -ne "${BLUE}[INFO]${RESET} Uploading bin/$filename"

			if uvx mpremote connect "$device" cp "$file" ":bin/$filename" &>/dev/null; then
				echo -n "."
				echo ""
			else
				echo ""
				log ERROR "Failed to upload bin/$filename"
				return 1
			fi
		done
	fi

	log SUCCESS "All files uploaded successfully"
	return 0
}

# Compile Python files to .mpy bytecode if needed
compile_if_needed() {
	# Only compile if --compiled flag is used
	if [[ "$USE_COMPILED" != true ]]; then
		return 0
	fi

	local source_dir="src/${FIRMWARE_VERSION}"
	local mpy_dir="src/${FIRMWARE_VERSION}/mpy"

	# Check if mpy/ directory already exists
	if [[ -d "$mpy_dir" ]]; then
		log INFO "Using existing compiled files in $mpy_dir"
		return 0
	fi

	log INFO "Compiled files not found, compiling Python source to bytecode"

	# Check if uvx is available (already checked in check_uvx, but verify again)
	if ! command -v uvx &>/dev/null; then
		log ERROR "uvx not found"
		log INFO "Install from: https://docs.astral.sh/uv/"
		return 1
	fi

	# Validate source directory exists
	if [[ ! -d "$source_dir" ]]; then
		log ERROR "Source directory not found: $source_dir"
		return 1
	fi

	# Create mpy directory
	mkdir -p "$mpy_dir"
	log INFO "Created directory: $mpy_dir"

	# Find and compile all .py files (excluding upload.py)
	local py_files=()
	while IFS= read -r -d '' file; do
		local filename=$(basename "$file")
		# Skip upload.py and any files starting with test_
		if [[ "$filename" != "upload.py" && "$filename" != test_* ]]; then
			py_files+=("$file")
		fi
	done < <(find "$source_dir" -maxdepth 1 -name "*.py" -type f -print0)

	if [[ ${#py_files[@]} -eq 0 ]]; then
		log ERROR "No Python files found in $source_dir"
		return 1
	fi

	log INFO "Compiling ${#py_files[@]} Python files..."

	# Compile each file
	local compiled_count=0
	for file in "${py_files[@]}"; do
		local filename=$(basename "$file")
		local mpy_file="$mpy_dir/${filename%.py}.mpy"

		if uvx mpy-cross "$file" -o "$mpy_file" 2>/dev/null; then
			((compiled_count++))
			log INFO "Compiled: $filename → $(basename "$mpy_file")"
		else
			log WARN "Failed to compile: $filename"
		fi
	done

	if [[ $compiled_count -eq 0 ]]; then
		log ERROR "No files were compiled successfully"
		rm -rf "$mpy_dir"
		return 1
	fi

	log SUCCESS "Compiled $compiled_count/${#py_files[@]} files to $mpy_dir"
	return 0
}

# Workflow handlers
handle_wipe_flash() {
	log INFO "=== WIPE AND FLASH WORKFLOW ==="

	# Validate version is specified
	if [[ -z "$FIRMWARE_VERSION" ]]; then
		log ERROR "Version must be specified with --version"
		exit 1
	fi

	# Step 1: Wipe
	log INFO "Step 1/5: Wiping device"
	flash_uf2 "$FLASH_NUKE_UF2"

	# Step 2: Wait for reboot to bootloader
	log INFO "Step 2/5: Waiting for device to reboot to bootloader"
	log INFO "Device should automatically remount after erase completes"
	if ! wait_for_device "bootloader"; then
		log WARN "Device did not automatically remount"
		log INFO "Please hold BOOTSEL button and reconnect device"
		wait_for_device "bootloader" || {
			log ERROR "Failed to enter bootloader mode after wipe"
			exit 1
		}
	fi

	# Step 3: Flash MicroPython
	log INFO "Step 3/5: Installing MicroPython firmware"
	flash_uf2 "$MICROPYTHON_UF2"

	# Step 4: Wait for MicroPython
	log INFO "Step 4/5: Waiting for MicroPython to initialize"
	wait_for_device "micropython"

	# Step 5: Compile if needed and upload files
	log INFO "Step 5/5: Uploading firmware files"
	compile_if_needed || {
		log ERROR "Compilation failed"
		exit 1
	}
	upload_files

	log SUCCESS "=== WIPE AND FLASH COMPLETE ==="
}

handle_first_flash() {
	log INFO "=== FIRST TIME FLASH ==="

	# Validate UF2 file exists
	if [[ ! -f "$MICROPYTHON_UF2" ]]; then
		log ERROR "MicroPython firmware not found: $MICROPYTHON_UF2"
		exit 1
	fi

	flash_uf2 "$MICROPYTHON_UF2"
	log SUCCESS "=== FLASH COMPLETE ==="
	log INFO "Device will reboot. Run '$0 --version <VERSION>' to upload files"
}

handle_upload() {
	log INFO "=== UPLOAD FIRMWARE ==="

	# Validate version is specified
	if [[ -z "$FIRMWARE_VERSION" ]]; then
		log ERROR "Version must be specified with --version"
		log INFO "Example: $0 --version BHV"
		exit 1
	fi

	compile_if_needed || {
		log ERROR "Compilation failed"
		exit 1
	}
	upload_files
	log SUCCESS "=== UPLOAD COMPLETE ==="
}

# Main execution
main() {
	# Check dependencies
	check_uvx

	# Handle special modes
	if [[ "$DRY_RUN" == true ]]; then
		check_device_status
		exit $?
	fi

	if [[ "$WIPE_AND_FLASH" == true ]]; then
		handle_wipe_flash
		exit 0
	fi

	if [[ "$FIRST_TIME_FLASH" == true ]]; then
		handle_first_flash
		exit 0
	fi

	# Default: upload files
	handle_upload
}

main
