# RP2040 SAM Firmware (BHV)

This directory contains the MicroPython firmware for the RP2040-based System Attached Module (SAM) that manages power, battery monitoring, button inputs, LED outputs, and e-ink display for the Pamir Distiller device.

## Table of Contents
- [Upload Tool Usage](#upload-tool-usage)
- [System Architecture](#system-architecture)
- [Core Components](#core-components)
- [Hardware Interfaces](#hardware-interfaces)
- [Communication Protocol](#communication-protocol)
- [Power Management](#power-management)
- [Debug System](#debug-system)

## Upload Tool Usage

The `upload.py` script is a comprehensive tool for flashing and uploading firmware to the RP2040 SAM.

### Prerequisites
- Python 3.x
- `ampy` - Installed automatically by the script if missing
- `mpy-cross` - Only required for `--compile` mode

### Basic Usage

#### 1. First-time Setup (Flash MicroPython)
```bash
# Wipe flash and install MicroPython
./upload.py --wipe

# Or just install MicroPython (no wipe)
./upload.py --first
```

#### 2. Upload Source Files
```bash
# Upload all .py files as source code
./upload.py
```

#### 3. Compile and Upload
```bash
# Compile .py files to .mpy bytecode and upload
./upload.py --compile
```
This creates an `mpy/` directory with compiled files that are retained for future use.

#### 4. Upload Pre-compiled Files
```bash
# Upload existing .mpy files from mpy/ directory
./upload.py --mpy
```
This is useful for quick uploads when compiled files already exist.

### Upload Modes
- `--wipe`: Performs a full flash erase before installing MicroPython
- `--first`: Installs MicroPython without erasing (for virgin devices)
- `--compile`: Compiles Python files to bytecode (.mpy) before uploading
- `--mpy`: Uploads pre-compiled .mpy files from the mpy/ directory

### Special Handling
- The script handles the RP2040's auto-unmount behavior after UF2 flashing
- For compiled mode, `main.py` is compiled to `app.mpy` with a minimal loader
- All files are uploaded to the root directory on the device

## System Architecture

The firmware implements a multi-threaded architecture using RP2040's dual cores:

- **Core 0**: Main loop, button handling, power management, LED control
- **Core 1**: Dedicated UART communication (critical priority)

### Boot Sequence
1. Initialize hardware pins and watchdog
2. Check for special button combinations (firmware upload modes)
3. Initialize debug system and components
4. Start UART communication on Core 1
5. Enter main loop on Core 0

## Core Components

### main.py
The main orchestrator that:
- Manages system initialization and boot sequence
- Handles button inputs with interrupts
- Controls CM5 power states
- Manages battery monitoring and display
- Implements the main event loop
- Coordinates all subsystems

Key features:
- Power management with long-press triggers
- Battery status monitoring and warnings
- E-ink boot animation control
- USB switch management

### pamir_uart_protocols.py
Implements the UART communication protocol between RP2040 and the Raspberry Pi CM5.

Packet Types:
- **Button** (0x00): Button state changes
- **LED** (0x20): LED control commands
- **Power** (0x40): Power management and metrics
- **Display** (0x60): E-ink display control
- **Debug** (0x80/0xA0): Debug codes and messages
- **System** (0xC0): System control commands

Features:
- 4-byte packet structure with CRC8 checksum
- Bidirectional communication
- Type-safe packet parsing and creation

### uart_handler.py
Robust UART packet handler with:
- Circular buffer for incoming data
- Frame synchronization and error recovery
- Automatic resynchronization on errors
- Statistics tracking
- Health monitoring

### power_manager.py
Interfaces with the TI BQ27441 battery fuel gauge IC to provide:
- Battery voltage monitoring
- Current draw measurement (charging/discharging)
- Temperature monitoring
- State of charge percentage
- Power state management

### battery.py
Low-level driver for the BQ27441 battery management IC:
- I2C communication on pins 24 (SDA) and 25 (SCL)
- Battery calibration and configuration
- Direct register access methods
- One-shot initialization

### eink_driver_sam.py
Driver for the GDEY0213B74 e-ink display:
- SPI communication interface
- Multiple display modes (LUT, fast, partial)
- Image display from binary files
- Boot animation support
- Power management integration

### neopixel_controller.py
Controls the 7-LED NeoPixel strip with:
- Animation support (blink, fade, rainbow)
- RGB444 to RGB888 color conversion
- Queue-based command execution
- Completion callbacks for acknowledgments

### threaded_task_manager.py
Non-blocking task management system:
- Priority-based scheduling (CRITICAL, HIGH, NORMAL, LOW)
- Core-specific task assignment
- Task state tracking
- Performance statistics
- Thread-safe operations

### debug_handler.py
Centralized debug system with:
- Multiple log levels (OFF, ERROR, INFO, VERBOSE)
- Category-based filtering
- Circular message buffer
- Statistics tracking
- Minimal performance overhead

## Hardware Interfaces

### GPIO Pins
- **Buttons**: 
  - SELECT: Pin 16
  - UP: Pin 17
  - DOWN: Pin 18
- **Power Control**:
  - PMIC Enable: Pin 3 (CM5 power control)
  - CM5 Power Button: Pin 4
- **Display**:
  - E-ink Status: Pin 9
  - E-ink Mux: Pin 22
  - SPI pins for display communication
- **USB Switch**: Pin 23
- **Debug RGB LED**: Pin 20
- **LED Bar**: Pin 6 (7 NeoPixels)
- **UART**: Pins 0 (TX) and 1 (RX)
- **I2C**: Pins 24 (SDA) and 25 (SCL)
- **SAM Interrupt**: Pin 2

### Communication Interfaces
- **UART**: 115200 baud for CM5 communication
- **I2C**: Battery fuel gauge communication
- **SPI**: E-ink display control

## Communication Protocol

The system uses a custom 4-byte packet protocol:

```
[Type+Flags][Data0][Data1][CRC8]
```

- **Type**: 3 MSB identify packet type
- **Flags**: 5 LSB for type-specific flags
- **Data**: 2 bytes of payload
- **CRC8**: Checksum using polynomial 0x07

### Example Packets
- Button press: `[0x01][0x00][0x00][CRC]` (SELECT pressed)
- LED control: `[0x2F][0xFF][0xF0][CRC]` (All LEDs white)
- Power query: `[0x40][0x00][0x00][CRC]`

## Power Management

The system implements sophisticated power management:

### Power States
- **OFF**: CM5 powered down, SAM in low-power mode
- **RUNNING**: Full system operation
- **SUSPEND**: Reduced power state
- **SLEEP**: Minimal power consumption

### Power Control
- **Power On**: 3-second SELECT button hold
- **Normal Shutdown**: CM5 initiates via UART
- **Force Shutdown**: 5-second UP+DOWN hold
- **Emergency Shutdown**: Battery critical (1%)

### Battery Management
- Continuous monitoring via BQ27441
- Low battery warnings at 5%
- Critical battery shutdown at 1%
- Charging detection and status
- Temperature monitoring

## Debug System

The debug system provides runtime diagnostics:

### Debug Levels
1. **OFF**: No debug output
2. **ERROR**: Critical errors only
3. **INFO**: General information
4. **VERBOSE**: Detailed debugging

### Debug Categories
- `SYS`: System events
- `ERR`: Error conditions
- `BTN`: Button events
- `LED`: LED operations
- `PWR`: Power management
- `DSP`: Display operations
- `UART`: Communication events
- `PERF`: Performance metrics

### Debug Features
- Runtime level adjustment
- Category filtering
- Circular message buffer
- Performance statistics
- UART output control

## Error Handling

The system includes comprehensive error handling:
- Watchdog timer (2 seconds)
- UART frame resynchronization
- I2C communication retries
- Graceful degradation on sensor failures
- Emergency shutdown procedures

## Performance Considerations

- UART on dedicated Core 1 for reliability
- Interrupt-driven button handling
- Efficient circular buffers
- Minimal debug overhead
- Task prioritization
- Non-blocking operations where possible

## Development Notes

### Adding New Features
1. Update the protocol in `pamir_uart_protocols.py`
2. Add packet handlers in `main.py`
3. Implement feature logic
4. Add debug logging
5. Test with both cores active

### Debugging Tips
- Use `--compile` mode for production (smaller and faster)
- Enable VERBOSE debug level during development
- Monitor UART statistics for communication health
- Check power manager status for battery issues
- Use the task manager statistics to identify bottlenecks

### Known Limitations
- E-ink updates block during refresh
- Some I2C operations may fail temporarily
- Button interrupts may miss very fast presses
- Watchdog requires regular feeding in all loops