#!/usr/bin/env bash

# RP2040 Firmware Flash Tool

IFS=$'\n\t'

# Constants
SCRIPT_NAME="${0##*/}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLASH_TIMEOUT=60
MICROPYTHON_DOWNLOAD_URL="https://micropython.org/download/RPI_PICO/"

# Global flags
VERBOSE=false
FIRST_TIME_FLASH=false
WIPE_AND_FLASH=false
FIRMWARE_VERSION=""
QUIET=false
DOWNLOAD_LATEST=false
LIST_VERSIONS=false

# Colors
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
CYAN='\033[36m'
RESET='\033[0m'

# Logging
log() {
	local level="$1"
	shift
	[[ "$QUIET" == true ]] && return

	case "$level" in
	ERROR) echo -e "${RED}[ERROR]${RESET} $*" >&2 ;;
	WARN) echo -e "${YELLOW}[WARNING]${RESET} $*" >&2 ;;
	SUCCESS) echo -e "${GREEN}[SUCCESS]${RESET} $*" >&2 ;;
	INFO) echo -e "${BLUE}[INFO]${RESET} $*" >&2 ;;
	*) echo "$*" >&2 ;;
	esac
}

# Simple spinner
SPINNER_PID=""

start_spinner() {
	local msg="${1:-Working}"
	[[ "$QUIET" == true ]] && return

	(
		local spinner=('|' '/' '-' '\\')
		local i=0
		while true; do
			echo -ne "\r${CYAN}${spinner[$i]} ${msg}...${RESET}" >&2
			i=$(((i + 1) % ${#spinner[@]}))
			sleep 0.1
		done
	) &
	SPINNER_PID=$!
}

stop_spinner() {
	if [[ -n "$SPINNER_PID" ]] && kill -0 "$SPINNER_PID" 2>/dev/null; then
		kill "$SPINNER_PID" 2>/dev/null
		wait "$SPINNER_PID" 2>/dev/null
		echo -ne "\r\033[K" >&2
	fi
	SPINNER_PID=""
}

cleanup() {
	stop_spinner
}

trap cleanup EXIT
trap 'echo -e "\n${YELLOW}Interrupted${RESET}" >&2; cleanup; exit 130' INT

# Help
show_usage() {
	cat <<EOF
Usage: $SCRIPT_NAME [OPTIONS]

RP2040 firmware flash tool

MODES:
  $SCRIPT_NAME                    Upload firmware files to RP2040
  $SCRIPT_NAME --first           Flash MicroPython firmware only
  $SCRIPT_NAME --wipe            Wipe device and flash MicroPython
  $SCRIPT_NAME --download-latest Download latest MicroPython firmware
  $SCRIPT_NAME --list-versions   List available firmware versions

OPTIONS:
  -h, --help              Show this help
  -v, --verbose           Enable verbose output
  -q, --quiet             Suppress output
  --version VERSION       Specify firmware version (e.g., V0.2.4, BHV)

REQUIREMENTS:
  - uv: curl -LsSf https://astral.sh/uv/install.sh | sh
EOF
}

# Parse arguments
parse_arguments() {
	while [[ $# -gt 0 ]]; do
		case $1 in
		-h | --help)
			show_usage
			exit 0
			;;
		-v | --verbose)
			VERBOSE=true
			shift
			;;
		-q | --quiet)
			QUIET=true
			shift
			;;
		--first)
			FIRST_TIME_FLASH=true
			shift
			;;
		--wipe)
			WIPE_AND_FLASH=true
			shift
			;;
		--download-latest)
			DOWNLOAD_LATEST=true
			shift
			;;
		--list-versions)
			LIST_VERSIONS=true
			shift
			;;
		--version)
			if [[ $# -lt 2 ]]; then
				log ERROR "--version requires a value"
				exit 1
			fi
			FIRMWARE_VERSION="$2"
			shift 2
			;;
		-*)
			log ERROR "Unknown option: $1"
			show_usage >&2
			exit 1
			;;
		*)
			log ERROR "Unexpected argument: $1"
			exit 1
			;;
		esac
	done

	if [[ "$FIRST_TIME_FLASH" == true && "$WIPE_AND_FLASH" == true ]]; then
		log ERROR "Cannot use both --first and --wipe"
		exit 1
	fi
}

# Check dependencies
check_dependencies() {
	if ! command -v python3 >/dev/null 2>&1; then
		log ERROR "python3 not found"
		return 1
	fi

	if ! command -v uvx >/dev/null 2>&1; then
		log ERROR "uv not found. Install: curl -LsSf https://astral.sh/uv/install.sh | sh"
		return 1
	fi

	return 0
}

# Device detection
find_rp2040_bootloader() {
	local os="$(uname -s)"
	local -a search_paths

	if [[ "$os" == "Darwin" ]]; then
		search_paths=("/Volumes/RPI-RP2" "/Volumes")
	else
		search_paths=("/media/${USER:-}/RPI-RP2" "/mnt/RPI-RP2" "/run/media/${USER:-}/RPI-RP2")
	fi

	for path in "${search_paths[@]}"; do
		if [[ -d "$path" && -w "$path" ]]; then
			if [[ -f "$path/INDEX.HTM" ]] || [[ -f "$path/INFO_UF2.TXT" ]]; then
				echo "$path"
				return 0
			fi
		fi
	done

	return 1
}

test_micropython_connection() {
	if timeout 5 uvx mpremote exec "print('test')" 2>/dev/null | grep -q "test"; then
		return 0
	fi
	return 1
}

# Version management
list_versions() {
	log INFO "Available firmware versions:"
	echo

	if [[ -d "src/BHV" ]]; then
		echo "  • BHV (Latest)"
	fi

	# Simple version listing
	for dir in src/V*; do
		if [[ -d "$dir" ]]; then
			echo "  • $(basename "$dir")"
		fi
	done | sort -rV

	echo
}

get_firmware_version() {
	if [[ -n "$FIRMWARE_VERSION" ]]; then
		echo "$FIRMWARE_VERSION"
		return 0
	fi

	if [[ -d "src/BHV" ]]; then
		echo "BHV"
		return 0
	fi

	# Find latest V* version
	local latest
	latest=$(ls -d src/V* 2>/dev/null | sort -rV | head -1)
	if [[ -n "$latest" ]]; then
		basename "$latest"
	else
		log ERROR "No version directories found"
		exit 1
	fi
}

get_micropython_firmware() {
	local firmware
	firmware=$(find UF2/ -name "RPI_PICO*.uf2" -type f 2>/dev/null | sort | tail -1)

	if [[ -n "$firmware" ]]; then
		echo "$firmware"
		return 0
	fi

	return 1
}

# Download latest MicroPython - FIXED
download_latest_micropython() {
	log INFO "Fetching latest MicroPython firmware..."

	mkdir -p UF2

	# Fetch the download page and extract latest firmware link
	local page_html
	if command -v curl >/dev/null 2>&1; then
		page_html=$(curl -s "$MICROPYTHON_DOWNLOAD_URL") || {
			log ERROR "Failed to fetch firmware list"
			return 1
		}
	elif command -v wget >/dev/null 2>&1; then
		page_html=$(wget -qO- "$MICROPYTHON_DOWNLOAD_URL") || {
			log ERROR "Failed to fetch firmware list"
			return 1
		}
	else
		log ERROR "Neither curl nor wget found"
		return 1
	fi

	# Extract the first (latest) firmware link
	local firmware_path
	firmware_path=$(echo "$page_html" | grep -oE 'href="/resources/firmware/RPI_PICO-[0-9]{8}-v[0-9]+\.[0-9]+\.[0-9]+\.uf2"' | head -1 | cut -d'"' -f2)

	if [[ -z "$firmware_path" ]]; then
		log ERROR "Could not find firmware download link"
		log INFO "Visit: $MICROPYTHON_DOWNLOAD_URL"
		return 1
	fi

	local filename=$(basename "$firmware_path")
	local version=$(echo "$filename" | grep -oE 'v[0-9]+\.[0-9]+\.[0-9]+')
	local download_url="https://micropython.org${firmware_path}"

	# Check if already downloaded
	if [[ -f "UF2/$filename" ]]; then
		log INFO "Already have latest firmware: $filename"
		echo "UF2/$filename"
		return 0
	fi

	log INFO "Downloading MicroPython ${version}..."
	[[ "$VERBOSE" == true ]] && log INFO "URL: $download_url"

	# Download the firmware
	if command -v curl >/dev/null 2>&1; then
		if curl -L -o "UF2/$filename" "$download_url"; then
			log SUCCESS "Downloaded: $filename"
			echo "UF2/$filename"
			return 0
		fi
	elif command -v wget >/dev/null 2>&1; then
		if wget -O "UF2/$filename" "$download_url"; then
			log SUCCESS "Downloaded: $filename"
			echo "UF2/$filename"
			return 0
		fi
	fi

	log ERROR "Failed to download firmware"
	return 1
}

# Device status
check_device_status() {
	log INFO "Device Status Check"

	local bootloader_device
	if bootloader_device="$(find_rp2040_bootloader)"; then
		log SUCCESS "RP2040 in BOOTLOADER mode: $bootloader_device"
		return 0
	fi

	if test_micropython_connection; then
		log SUCCESS "MicroPython is responding"
		return 0
	fi

	log ERROR "No RP2040 device found"
	log INFO "• For firmware flashing: Hold BOOTSEL while connecting USB"
	log INFO "• For file upload: Connect normally"
	return 1
}

# File operations
flash_uf2_file() {
	local file="$1"

	if [[ ! -f "$file" ]]; then
		log ERROR "UF2 file not found: $file"
		return 1
	fi

	local device
	if ! device="$(find_rp2040_bootloader)"; then
		log ERROR "RP2040 not in bootloader mode"
		log INFO "Hold BOOTSEL button while connecting USB"
		return 1
	fi

	local filename=$(basename "$file")
	echo -ne "${CYAN}Flashing ${filename}...${RESET}" >&2

	if cp "$file" "$device/$(basename "$file")" 2>/dev/null; then
		echo -e " ${GREEN}[OK]${RESET}" >&2
		sleep 3 # Wait for device to reboot
		return 0
	else
		echo -e " ${RED}[FAIL]${RESET}" >&2
		return 1
	fi
}

upload_file() {
	local file="$1"

	if [[ ! -f "$file" ]]; then
		return 1
	fi

	local version="$(get_firmware_version)"
	local version_dir="src/$version"
	local relative_path="${file#$version_dir/}"
	local target_path=":${relative_path}"

	timeout "$FLASH_TIMEOUT" uvx mpremote cp "$file" "$target_path" 2>/dev/null
}

# Wipe and flash
handle_wipe_and_flash() {
	log INFO "WIPE AND FLASH OPERATION"

	# Step 1: Wipe
	log INFO "Step 1/3: Wiping device"
	local retries=3
	local success=false

	for ((i = 1; i <= retries; i++)); do
		if flash_uf2_file "UF2/flash_nuke.uf2"; then
			success=true
			break
		fi
		[[ $i -lt $retries ]] && sleep 2
	done

	if [[ "$success" != true ]]; then
		log ERROR "Failed to wipe device"
		return 1
	fi

	# Step 2: Wait for bootloader
	log INFO "Step 2/3: Device wiped"
	log WARN "Please hold BOOTSEL and reconnect the device"

	start_spinner "Waiting for device"
	local wait_time=0
	while [[ $wait_time -lt 30 ]]; do
		if find_rp2040_bootloader >/dev/null 2>&1; then
			stop_spinner
			log SUCCESS "Device detected"
			break
		fi
		sleep 1
		wait_time=$((wait_time + 1))
	done
	stop_spinner

	if [[ $wait_time -ge 30 ]]; then
		log ERROR "Timeout waiting for device"
		return 1
	fi

	# Step 3: Flash MicroPython
	log INFO "Step 3/3: Installing MicroPython"
	local firmware
	firmware=$(get_micropython_firmware)

	if [[ -z "$firmware" ]]; then
		log ERROR "No MicroPython firmware found"
		return 1
	fi

	if flash_uf2_file "$firmware"; then
		log SUCCESS "Device successfully wiped and flashed"
		return 0
	else
		log ERROR "Failed to install MicroPython"
		return 1
	fi
}

# File processing
get_files_to_process() {
	local -a files

	if [[ "$FIRST_TIME_FLASH" == true ]]; then
		local firmware=$(get_micropython_firmware)
		if [[ -n "$firmware" ]]; then
			files=("$firmware")
		else
			log ERROR "No MicroPython firmware found"
			return 1
		fi
	elif [[ "$WIPE_AND_FLASH" == true ]]; then
		if [[ -f "UF2/flash_nuke.uf2" ]]; then
			files=("UF2/flash_nuke.uf2")
		else
			log ERROR "flash_nuke.uf2 not found"
			return 1
		fi
	else
		local version="$(get_firmware_version)"
		local version_dir="src/$version"

		if [[ ! -d "$version_dir" ]]; then
			log ERROR "Version directory not found: $version_dir"
			return 1
		fi

		# Find Python and binary files
		files=($(find "$version_dir" \( -name "*.py" -o -name "*.bin" \) 2>/dev/null))
	fi

	printf '%s\n' "${files[@]}"
}

# Create required directories on the device
create_directories() {
	local -a files=("$@")
	local -a dirs=()
	local version="$(get_firmware_version)"
	local version_dir="src/$version"

	# Extract unique directories from file paths
	for file in "${files[@]}"; do
		# Skip UF2 files (they don't need directories)
		[[ "$file" == *.uf2 ]] && continue

		# Get relative path and extract directory
		local relative_path="${file#$version_dir/}"
		local dir=$(dirname "$relative_path")

		# Add to dirs array if not "." and not already added
		if [[ "$dir" != "." ]] && [[ ! " ${dirs[@]} " =~ " ${dir} " ]]; then
			dirs+=("$dir")
		fi
	done

	# Create directories on device if any found
	if [[ ${#dirs[@]} -gt 0 ]]; then
		[[ "$VERBOSE" == true ]] && log INFO "Creating directories: ${dirs[*]}"
		for dir in "${dirs[@]}"; do
			uvx mpremote mkdir ":$dir" 2>/dev/null || true
		done
	fi
}

process_files() {
	local -a files=("$@")
	local total=${#files[@]}

	if [[ $total -eq 0 ]]; then
		log ERROR "No files to process"
		return 1
	fi

	log INFO "Processing $total files..."

	# Create required directories first (for non-UF2 files)
	if [[ "$FIRST_TIME_FLASH" != true ]] && [[ "$WIPE_AND_FLASH" != true ]]; then
		create_directories "${files[@]}"
	fi

	# Process each file
	local success=0
	local count=0

	for file in "${files[@]}"; do
		count=$((count + 1))
		local filename=$(basename "$file")

		echo -ne "${CYAN}[$count/$total] ${filename:0:40}...${RESET}" >&2

		local upload_ok=false
		case "$file" in
		*.uf2)
			flash_uf2_file "$file" && upload_ok=true
			;;
		*.py | *.bin)
			upload_file "$file" && upload_ok=true
			;;
		*)
			echo -e " ${YELLOW}[SKIP]${RESET}" >&2
			continue
			;;
		esac

		if [[ "$upload_ok" == true ]]; then
			echo -e " ${GREEN}[OK]${RESET}" >&2
			success=$((success + 1))
		else
			echo -e " ${RED}[FAIL]${RESET}" >&2
		fi
	done

	echo >&2
	if [[ $success -eq $total ]]; then
		log SUCCESS "All files processed successfully"
		return 0
	else
		log WARN "Processed: $success/$total successful"
		return 1
	fi
}

# Main
main() {
	cd "$SCRIPT_DIR" || exit 1
	parse_arguments "$@"

	# Handle special modes
	if [[ "$LIST_VERSIONS" == true ]]; then
		list_versions
		exit 0
	fi

	if [[ "$DOWNLOAD_LATEST" == true ]]; then
		if download_latest_micropython; then
			[[ "$FIRST_TIME_FLASH" != true ]] && exit 0
		else
			exit 1
		fi
	fi

	if ! check_dependencies; then
		exit 1
	fi

	if [[ "$WIPE_AND_FLASH" == true ]]; then
		handle_wipe_and_flash
		exit $?
	fi

	# Show mode
	if [[ "$FIRST_TIME_FLASH" == true ]]; then
		log INFO "MODE: First-time firmware flash"
	else
		log INFO "MODE: File upload to MicroPython"
		log INFO "Version: $(get_firmware_version)"
	fi

	# Check device
	if [[ "$FIRST_TIME_FLASH" != true ]]; then
		if ! test_micropython_connection; then
			log ERROR "No MicroPython device found"

			if find_rp2040_bootloader >/dev/null 2>&1; then
				log INFO "Device is in bootloader mode"
				log INFO "Run: $SCRIPT_NAME --first"
			else
				log INFO "Please connect device with MicroPython"
			fi
			exit 1
		fi
	fi

	# Process files
	local -a files
	readarray -t files < <(get_files_to_process)

	if [[ ${#files[@]} -eq 0 ]]; then
		log ERROR "No files found to process"
		exit 1
	fi

	process_files "${files[@]}"
	exit $?
}

main "$@"
