# Distiller-SAM-Firmware

## Overview

Distiller-SAM-Firmware is a firmware project for a device that utilizes a combination of a Raspberry Pi Pico (RP2040) and a SAM (System on Module) to control an e-ink display, manage power, handle user input, and communicate between components. The firmware manages various hardware components including buttons, an e-ink display, LEDs, and a battery management system.

## Architecture

The system consists of:

1. **RP2040 Microcontroller** - Handles the primary firmware logic, button inputs, and communication with other components
2. **E-ink Display** - A 240x416 pixel display managed through a custom driver
3. **Button Interface** - Three physical buttons (Up, Down, Select) for user input
4. **Communication** - UART interface between RP2040 and SAM
5. **Battery Management** - Interface for battery monitoring
6. **NeoPixel LED** - For visual status indicators
7. **Fan Control** - Components for temperature management

## Key Components

### Main Controller

The main controller manages power states, button input, and communication between components. It implements:
- Button debouncing
- Power management routines
- E-ink display sequencing
- UART communication with the SAM module

### E-ink Driver

A custom driver (`eink_driver_sam.py`) manages the e-ink display through SPI communication, providing:
- Display initialization (full and partial refresh modes)
- Image loading and display from binary files
- Power management for the display

### Battery Management

The `battery.py` module provides an interface to the battery management system, allowing:
- Voltage monitoring
- Temperature reading
- Remaining capacity estimation

## Development

### Setup

1. Clone the repository:
```bash
git clone https://github.com/your-username/Distiller-SAM-Firmware.git
cd Distiller-SAM-Firmware
```

2. Install the required MicroPython firmware to the RP2040:
   - Use the firmware in the ULP folder: `RPI_PICO-20240222-v1.22.2.uf2`

3. Deploy the firmware files to the RP2040 using your preferred MicroPython deployment method

### Script Uploader

The project includes a script uploader (`scriptloarder.sh`) to simplify the firmware deployment process. This script automates uploading the firmware to the RP2040 device.

#### Prerequisites

- macOS environment (the script uses macOS-specific paths)
- Homebrew installed
- The `ampy` utility for uploading files to MicroPython devices

#### Installation

1. Install the required dependencies:
```bash
pip install adafruit-ampy
```

2. Make the script executable:
```bash
chmod +x scriptloarder.sh
```

#### Usage

The script uploader supports several modes:

1. **Basic Upload** - Upload the firmware files to the RP2040:
```bash
./scriptloarder.sh
```

2. **First Time Flash** - Flash the MicroPython firmware to a new RP2040:
```bash
./scriptloarder.sh --first
```

3. **Wipe and Flash** - Wipe the RP2040 and flash a new firmware:
```bash
./scriptloarder.sh --wipe
```

The script will:
1. Wait for the RP2040 device to be detected
2. Copy the necessary firmware files
3. Upload the application files (`main.py`, `eink_driver_sam.py`, and binary files)
4. Show a progress bar during the upload process

### Hardware Requirements

- Raspberry Pi Pico (RP2040)
- SAM Module
- E-ink Display (240x416)
- Battery and BMS
- Button inputs
- NeoPixel RGB LED

## Configuration

The system has configurable parameters in the main scripts:
- `PRODUCTION` flag to enable/disable USB debugging
- `UART_DEBUG` flag for UART debugging output
- Watchdog timeout settings
- Button debounce timing

## BHV (RP2040 SAM) Firmware Features

The BHV firmware provides comprehensive hardware control and system management capabilities:

### Power Management
- **Smart Power Control**: Long press (3s) SELECT button to power on CM5, with graceful shutdown support
- **Battery Monitoring**: Real-time monitoring of battery percentage, voltage, current, and temperature via BQ27441 IC
- **Low Battery Protection**: Automatic display of battery warnings on e-ink when CM5 is off
- **Emergency Shutdown**: Automatic power-off at 1% battery (when not charging)
- **Force Shutdown**: UP+DOWN buttons (5s) for forced power cycling

### Display System
- **E-ink Control**: 240x416 pixel display with both LUT and fast refresh modes
- **Smart Display Management**: Automatic handoff between RP2040 and CM5
- **Boot Animations**: Loading screens during CM5 startup
- **Battery Status Display**: Low/dead battery warnings when powered off

### Communication Protocol
- **Pamir UART Protocol v0.2.0**: Reliable 115200 baud communication with CRC8 validation
- **Message Types**: Button events, LED control, power management, display control, debug info
- **Multi-Core Architecture**: Dedicated UART handling on Core 1 for reliable communication

### User Interface
- **Three Buttons**: SELECT, UP, DOWN with hardware debouncing
- **Special Modes**: 
  - UP button at boot: CM5 firmware update mode
  - DOWN button at boot: SAM debug mode
- **LED Feedback**: 7 NeoPixel RGB LEDs with animations (static, blink, fade, rainbow)

### System Monitoring
- **Debug System**: Multi-level logging with category filtering
- **Performance Tracking**: UART statistics, task performance metrics
- **Health Monitoring**: Watchdog timer (2s), system heartbeat (10s)
- **Status LED**: RGB debug LED showing system state

### Hardware Control
- **USB Switching**: Dynamic routing between SAM and SOM USB
- **GPIO Management**: Button inputs, power controls, display multiplexer
- **Temperature Monitoring**: Via battery management IC
- **Interrupt System**: Hardware interrupts for buttons and critical events

## Files Structure

- `src/BHV/` - BHV (RP2040 SAM) firmware
  - `main.py` - Primary firmware for the RP2040
  - `eink_driver_sam.py` - E-ink display driver
  - `battery.py` - Battery management system interface
  - `power_manager.py` - Power state and battery management
  - `bin/` - Binary image files for e-ink display
  - `asset/` - Source PNG images for display
- `src/CM5/` - Raspberry Pi CM5 module firmware
- `Tools/` - Development utilities
  - `battery_remote_monitoring_tool/` - Real-time battery monitoring dashboard
  - `image_processor/` - E-ink image conversion tools
- `UF2/` - MicroPython firmware files
- `scriptloarder.sh` - Script to automate firmware deployment

## License

[Specify the license here]

## Contributing

[Contribution guidelines if applicable]

## Contact

[Contact information]