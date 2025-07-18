# Pamir SAM UART Protocol (Overview)

Category: kernel_driver
Created by: Utsav Balar
Description: Ultra optimized bi-directional UART communication protocol
Status: Not started

# Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual

**Document Classification:** Technical Reference Manual

**Document Version:** 0.1.0

**Last Updated:** 2025-05-19

**Target Audience:** System Integrators, Embedded Developers, Kernel Maintainers

## Table of Contents

1. [Introduction](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    1. [Purpose](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    2. [Scope](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    3. [Target Devices](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
2. [Architecture Overview](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    1. [Component Interactions](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    2. [Implementation Status](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
3. [Protocol Specification](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    1. [Packet Structure](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    2. [Field Encoding](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    3. [Message Types](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    4. [Checksum Algorithm](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    5. [Endianness Considerations](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    6. [Timing Constraints](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
4. [Interface Specifications](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    1. [Button Interface](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    2. [LED Control](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    3. [Power Management](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    4. [Display Control](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    5. [Debug Interface](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    6. [System Commands](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    7. [Extended Commands](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
5. [Versioning and Compatibility](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
6. [Device Integration](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    1. [Userspace Interface](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    2. [Device Tree Configuration](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
    3. [Required Kernel Modifications](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
7. [Error Handling and Recovery](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
8. [Debug and Troubleshooting](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
9. [Performance Considerations](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
10. [Packet Reference](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)
11. [Implementation Notes](https://www.notion.so/Pamir-SAM-UART-Protocol-Overview-1eca32a15ee180c1b6edc4073b87f29e?pvs=21)

## Introduction

### Purpose

The Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual provides comprehensive technical specifications for the communication protocol used to interface between a Linux host system and the RP2040 microcontroller in Pamir AI CM5 devices. This protocol enables bidirectional communication for hardware control and status reporting via UART.

### Scope

This document covers the complete technical specification of the SAM protocol, including packet formats, command structures, error handling mechanisms, and integration requirements. It serves as the authoritative reference for both hardware and software implementations of the protocol.

### Target Devices

- **Primary Target:** Pamir AI CM5 devices containing RP2040 microcontrollers
- **Secondary Targets:** Compatible embedded devices that implement the SAM protocol

## Architecture Overview

The SAM driver implements a modular architecture with clearly defined components that interact through standardized interfaces. The following diagram illustrates the high-level architecture:

```
+--------------------+     +---------------------+
| Linux Host System  |     | RP2040 MCU          |
|                    |     |                     |
|  +---------------+ |     | +---------------+   |
|  | Userspace API | |     | | Firmware      |   |
|  +-------+-------+ |     | | Components    |   |
|          |         |     | +-------+-------+   |
|  +-------+-------+ |     | +-------+-------+   |
|  | Kernel Driver | <---->| | Protocol      |   |
|  +---------------+ |UART | | Handler       |   |
+--------------------+     +---------------------+
         |                           |
    +----+---------------------------+----+
    |                                     |
+---+---+  +-------+  +------+  +--------+
|Buttons|  |  LEDs |  |Power |  |Display |
+-------+  +-------+  +------+  +--------+

```

### Component Interactions

The SAM driver architecture consists of the following modular components:

| Component | Description | Primary Responsibilities |
| --- | --- | --- |
| Protocol Core | Lowest-level packet handling and validation | Packet parsing, validation, checksum calculation, dispatching |
| Input Handler | Button event processing and Linux input subsystem integration | Debouncing, key mapping, event generation |
| LED Handler | RGB LED control with multiple animation modes | Color conversion, animation sequencing, brightness control |
| Power Manager | Power state management and coordination | Boot/shutdown notifications, power metrics collection, state transitions |
| Display Controller | E-ink display interfacing and management | Status reporting, refresh control, mode selection |
| Debug Interface | Diagnostic capabilities for troubleshooting | Error logging, state reporting, diagnostic code generation |
| System Commands | Core system control functions | Version exchange, ping/connectivity, configuration |
| Character Device | Userspace interface for direct protocol access | Raw packet transmission/reception, debugging access |

### Implementation Status

The following table provides a detailed overview of the implementation status for each component:

| Component | Kernel Implementation Status | Feature Completeness | API Stability | Notes |
| --- | --- | --- | --- | --- |
| Protocol Core | Complete | 100% | Stable | Core packet processing functionality fully implemented and tested |
| Input Handler | Complete | 100% | Stable | Button events fully supported with Linux input subsystem integration |
| LED Handler | Partial | 75% | Beta | Basic control implemented; LED brightness control from host is pending |
| Power Manager | Complete | 100% | Stable | Boot/shutdown notifications and poll-based power metrics fully implemented |
| Display Controller | Minimal | 25% | Alpha | Status reporting only; active display control pending |
| Debug Interface | Complete | 100% | Stable | Both debug codes and text messages fully supported |
| System Commands | Complete | 100% | Stable | All core system commands implemented, including versioning |
| Character Device | Complete | 100% | Stable | Userspace interface fully functional with proper error handling |

## Protocol Specification

### Packet Structure

The SAM protocol employs a fixed-size 4-byte packet format optimized for minimal overhead while maintaining reliable operation. Each packet consists of the following fields:

| Field | Size | Offset | Description |
| --- | --- | --- | --- |
| type_flags | 1 byte | 0 | Message type (3 most significant bits) and command-specific flags (5 least significant bits) |
| data[0] | 1 byte | 1 | First data byte (interpretation depends on message type) |
| data[1] | 1 byte | 2 | Second data byte (interpretation depends on message type) |
| checksum | 1 byte | 3 | XOR checksum of all previous bytes for error detection |

This corresponds to the C structure:

```c
struct sam_protocol_packet {
    uint8_t type_flags;
    uint8_t data[2];
    uint8_t checksum;
} __packed;

```

### Field Encoding

The `type_flags` byte uses the following bit layout:

```
+---+---+---+---+---+---+---+---+
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
+---+---+---+---+---+---+---+---+
|   Type    |    Command Flags   |
+---+---+---+---+---+---+---+---+

```

- **Bits 7-5**: Message type (3 bits, 8 possible types)
- **Bits 4-0**: Command-specific flags (5 bits, interpretation depends on message type)

The data bytes' interpretation is entirely dependent on the message type and command. Specific encodings are detailed in the respective command sections.

### Message Types

The 3 most significant bits of the `type_flags` byte define the message type:

| Type Value (Binary) | Type Value (Hex) | Name | Direction | Description |
| --- | --- | --- | --- | --- |
| `0b000xxxxx` | `0x00` | TYPE_BUTTON | MCU → Host | Button state change events |
| `0b001xxxxx` | `0x20` | TYPE_LED | Host ↔ MCU | LED control commands and status |
| `0b010xxxxx` | `0x40` | TYPE_POWER | Host ↔ MCU | Power management and metrics |
| `0b011xxxxx` | `0x60` | TYPE_DISPLAY | Host ↔ MCU | E-ink display control and status |
| `0b100xxxxx` | `0x80` | TYPE_DEBUG_CODE | MCU → Host | Numeric debug codes for diagnostics |
| `0b101xxxxx` | `0xA0` | TYPE_DEBUG_TEXT | MCU → Host | Text debug messages (can span packets) |
| `0b110xxxxx` | `0xC0` | TYPE_SYSTEM | Host ↔ MCU | Core system control commands |
| `0b111xxxxx` | `0xE0` | TYPE_EXTENDED | Host ↔ MCU | Extended commands (future expansion) |

### Checksum Algorithm

The protocol employs a simple XOR checksum algorithm to ensure packet integrity:

```c
uint8_t calculate_checksum(const struct sam_protocol_packet *packet)
{
    /* XOR of first 3 bytes */
    return packet->type_flags ^ packet->data[0] ^ packet->data[1];
}

```

This algorithm has been chosen for its minimal computational requirements while providing basic error detection capabilities. It can detect any single-bit error and many multi-bit errors, although it cannot detect all possible error patterns.

### Endianness Considerations

All multi-byte values in the protocol (such as 16-bit power metrics) are transmitted in little-endian format, with the least significant byte first. This matches the native byte order of both the RP2040 microcontroller and typical x86-based host systems.

For example, a 16-bit power metric value of 0x1234 would be transmitted as:

- data[0] = 0x34 (low byte)
- data[1] = 0x12 (high byte)

### Timing Constraints

The SAM protocol has the following timing characteristics and constraints:

| Parameter | Value | Description |
| --- | --- | --- |
| Baud Rate | 115200 bps | Standard UART communication speed |
| Character Time | ~87 μs | Time to transmit one byte at 115200 baud |
| Packet Transmission Time | ~348 μs | Time to transmit a complete 4-byte packet |
| Minimum Inter-Packet Gap | 100 μs | Minimum delay between consecutive packets |
| Maximum Response Latency | 10 ms | Maximum acceptable delay for time-critical responses |
| Timeout for Acknowledgment | 100 ms | Time to wait for acknowledgment before timeout |
| Recovery Timeout | 1000 ms | Default timeout for protocol recovery after synchronization loss |

**Note**: These timing parameters may be adjusted based on specific implementation requirements. In particular, the timeout values can be configured via device tree properties.

## Interface Specifications

### Button Interface

Button events are generated by the RP2040 microcontroller when physical buttons on the CM5 device are pressed or released. These events are sent to the Linux host system, which processes them through the Linux input subsystem.

**Technical Specifications:**

- **Communication Direction**: Unidirectional (RP2040 → Linux host)
- **Debounce Method**: Hardware-assisted with software validation
- **Debounce Time**: 50ms (configurable in firmware)
- **Interrupt Triggering**: Both rising and falling edges
- **Event Rate Limiting**: No more than 20 events per second per button

**Data Format:**

Button events use the 5 least significant bits of the `type_flags` byte to indicate button state:

| Bit | Mask | Button | Linux Key Mapping |
| --- | --- | --- | --- |
| 0 | 0x01 | UP button | KEY_UP (103) |
| 1 | 0x02 | DOWN button | KEY_DOWN (108) |
| 2 | 0x04 | SELECT button | KEY_ENTER (28) |
| 3 | 0x08 | POWER button | KEY_POWER (116) |
| 4 | 0x10 | Reserved | N/A |

**Packet Structure:**

```
+---+---+---+---+---+---+---+---+
| 0 | 0 | 0 | 0 | P | S | D | U |
+---+---+---+---+---+---+---+---+
  Type=0      | Button State Bits

```

Where:

- U: UP button state (1=pressed, 0=released)
- D: DOWN button state (1=pressed, 0=released)
- S: SELECT button state (1=pressed, 0=released)
- P: POWER button state (1=pressed, 0=released)

The data bytes (data[0] and data[1]) are reserved for future extensions and should be set to 0x00.

**Implementation Details:**

The Linux kernel driver connects button events to the input subsystem through the following sequence:

1. The driver registers an input device during initialization
2. When a button packet is received, the driver extracts the button state
3. The driver reports the current state of each button via `input_report_key()`
4. The driver calls `input_sync()` to signal that a complete event has been reported
5. The Linux input subsystem propagates the event to userspace applications

### LED Control

LED commands control the RGB LEDs on the CM5 device. These commands are primarily sent from the Linux host to the RP2040 microcontroller, which then drives the physical LEDs.

**Technical Specifications:**

- **Communication Direction**: Primarily Host → RP2040, with acknowledgments RP2040 → Host
- **Color Depth**: 4 bits per channel (RGB444), providing 4096 possible colors
- **Response Time**: <10ms from command to visible change
- **Maximum Queue Length**: 16 color instructions per LED
- **Supported LEDs**: Up to 16 LEDs (0-15)
- **LED Addressing**: Individual or broadcast (all LEDs)

**Command Structure:**

LED commands use the following format in the `type_flags` byte:

```
+---+---+---+---+---+---+---+---+
| 0 | 0 | 1 | E |    LED ID     |
+---+---+---+---+---+---+---+---+
  Type=1      |Q|  ID (0-15)    |

```

Where:

- Type=1 (001): Indicates LED command
- E: Command type (0=Queue instruction, 1=Execute sequence)
- LED ID: LED identifier (0-15)

**Data Format:**

The 2 data bytes contain color and timing information:

```
data[0]:
+---+---+---+---+---+---+---+---+
| R | R | R | R | G | G | G | G |
+---+---+---+---+---+---+---+---+
  Red component  | Green component

data[1]:
+---+---+---+---+---+---+---+---+
| B | B | B | B | T | T | T | T |
+---+---+---+---+---+---+---+---+
 Blue component  | Time value

```

- Red/Green/Blue components: 4-bit values (0-15) for each color channel
- Time value: 4-bit value (0-15) representing delay multiplier

The actual time delay is calculated as: `delay_ms = (time_value + 1) * 100`

**Command Sequence:**

LED control uses a queue-based execution model:

1. Send one or more commands with E=0 to queue color instructions
2. Send a final command with E=1 to execute the entire sequence
3. The RP2040 executes the sequence and sends a completion acknowledgment

**Completion Acknowledgment:**

When an LED sequence completes, the RP2040 sends an acknowledgment packet:

- type_flags: TYPE_LED | LED_CMD_EXECUTE | LED_ID (the specific LED that completed)
- data[0]: 0xFF (completion indicator)
- data[1]: Sequence length that was executed

**Implementation Limitations:**

- Current implementation only supports the first LED (LED 0) on most hardware
- LED brightness control from the host requires additional kernel modifications
- Complex animations should be limited to no more than 16 steps to avoid buffer constraints

### Power Management

Power management commands coordinate power states between the Linux host and the RP2040 microcontroller. They ensure proper system shutdown and sleep states.

**When Used:**

- During system boot to notify the RP2040 that the Linux host has booted
- During system shutdown to notify the RP2040 to prepare for power-off
- When entering/exiting sleep modes
- For periodic reporting of power metrics (current, battery, temperature, voltage)

**Implementation Note:**

- **Fully implemented in the kernel**
- Boot notification sent automatically during driver initialization
- Shutdown notification sent automatically during system shutdown via Linux reboot notifier
- Sleep mode transitions supported for power management
- RP2040 firmware provides visual feedback for power state transitions
- Power metrics reported periodically and exposed via sysfs

### Power Command Types

| Value | Command | Description |
| --- | --- | --- |
| 0x00 | POWER_CMD_QUERY | Query current power status |
| 0x10 | POWER_CMD_SET | Set power state |
| 0x20 | POWER_CMD_SLEEP | Enter sleep mode |
| 0x30 | POWER_CMD_SHUTDOWN | Shutdown system |
| 0x40 | POWER_CMD_CURRENT | Current draw reporting |
| 0x50 | POWER_CMD_BATTERY | Battery state reporting |
| 0x60 | POWER_CMD_TEMP | Temperature reporting |
| 0x70 | POWER_CMD_VOLTAGE | Voltage reporting |

### Boot and Shutdown Notifications

**Boot Notification:**

- The kernel automatically sends a `POWER_CMD_SET` packet to the microcontroller during boot
- This notifies the RP2040 that the host is now running
- Example packet: `{0x50, 0x01, 0x00, 0x51}` (Set to running state)
- Sent during driver initialization in `sam_protocol_probe()`

**Shutdown Notification:**

- The kernel automatically sends a `POWER_CMD_SHUTDOWN` packet before system shutdown
- This allows the RP2040 to prepare for power loss
- Example packet: `{0x70, 0x00, 0x00, 0x70}` (Normal shutdown)
- Implemented via Linux reboot notifier system
- RP2040 acknowledges receipt and performs necessary shutdown preparations

**Kernel Implementation:**

- Boot notification sent in `sam_protocol_probe()` via `send_boot_notification()`
- Shutdown notification handled by registering a reboot notifier
- `sam_reboot_notifier_call()` callback sends the shutdown notification when triggered
- Separate handlers for normal and emergency shutdown modes

### Power Metrics Reporting

The RP2040 microcontroller reports power-related metrics to the Linux host:

| Metric | Description | Resolution | Range | Packet Format |
| --- | --- | --- | --- | --- |
| Current | Power draw in mA | 1 mA | 0-65535 mA | `{0x40, low, high, checksum}` |
| Battery | State of charge | 1% | 0-100% | `{0x50, low, high, checksum}` |
| Temperature | System temperature | 0.1°C | -3276.8°C to 3276.7°C | `{0x60, low, high, checksum}` |
| Voltage | System voltage | 1 mV | 0-65535 mV | `{0x70, low, high, checksum}` |

All values are sent as 16-bit little-endian integers (low byte first, high byte second).

**RP2040 Implementation:**

- Metrics are read from sensors (or simulated in the reference implementation)
- Reports are sent in response to poll requests from the Linux driver
- Polling is initiated by the Linux host via the `POWER_CMD_REQUEST_METRICS` command

**Linux Implementation:**

- Metrics are logged to kernel log (visible via `dmesg`)
- Values are stored in the driver and exposed via sysfs
- Available at `/sys/devices/.../power_metrics/`
- Polling interval is configurable via device tree

**Polling Mechanism:**

- The Linux driver polls for metrics at configurable intervals (default: 1 second)
- The polling interval is set via the `power-poll-interval-ms` device tree property
- Each poll request sends a `POWER_CMD_REQUEST_METRICS` packet (0x80)
- The RP2040 responds by sending all four metrics sequentially

**Sysfs Attributes:**

| Attribute | Example | Description |
| --- | --- | --- |
| current_ma | 250 | Current draw in mA |
| battery_percent | 75 | Battery charge in percent |
| temperature | 25.5 | Temperature in °C |
| voltage_mv | 3800 | Voltage in mV |
| metrics_last_update | 123 ms ago | Time since last update |

To read current values:

```bash
cat /sys/devices/platform/serial@*/power_metrics/current_ma
cat /sys/devices/platform/serial@*/power_metrics/battery_percent
cat /sys/devices/platform/serial@*/power_metrics/temperature
cat /sys/devices/platform/serial@*/power_metrics/voltage_mv

```

### Power Data Format

The data bytes' interpretation depends on the specific power command.

For POWER_CMD_QUERY responses:

- data\[0\]: Power state (1 = Running, 0 = Off, 2 = Suspended)
- data\[1\]: Reserved

For POWER_CMD_SET requests:

- data\[0\]: Desired power state (1 = Running, 0 = Off, 2 = Low power)
- data\[1\]: Flags (reserved for future use)

For POWER_CMD_SHUTDOWN requests:

- data\[0\]: Shutdown mode (0 = Normal, 1 = Emergency, 2 = Reboot)
- data\[1\]: Reason code (optional diagnostic information)

### System Commands

System commands provide core control and monitoring functions for the RP2040 microcontroller. They are used for basic communication testing, configuration, and health monitoring.

**When Used:**

- During driver initialization to test communication
- For health checking and connectivity testing
- To configure protocol parameters
- To query or reset the microcontroller

**Implementation Note:**

- Fully implemented in the kernel
- Used internally by the driver
- The ping command is particularly useful for debugging communication issues

**System Action Types**

| Value | Action | Description |
| --- | --- | --- |
| 0x00 | SYSTEM_PING | Ping for connectivity test |
| 0x01 | SYSTEM_RESET | Reset the microcontroller |
| 0x02 | SYSTEM_VERSION | Request/report version information |
| 0x03 | SYSTEM_STATUS | Request/report system status |
| 0x04 | SYSTEM_CONFIG | Get/set configuration parameters |

**Sending System Commands**

```c
int send_system_command(struct sam_protocol_data *priv, uint8_t action,
                        uint8_t command, uint8_t subcommand);

```

Example: Send ping command

```
Action: SYSTEM_PING (0x00)
Command: 0
Subcommand: 0

```

Example: Request version information

```
Action: SYSTEM_VERSION (0x02)
Command: 0
Subcommand: 0

```

### Display Control

Display control commands manage the E-ink display on the CM5 device. This includes refreshing the display, changing display modes, and monitoring display status.

**When Used:**

- To update the E-ink display with new content
- To change display modes (full refresh, partial update)
- To monitor display refresh status
- To put the display in power-saving modes

**Implementation Note:**

- **Minimal implementation** - Currently only supports status reporting
- **Pending:** Active display control needs to be implemented
- Integration with Linux framebuffer or DRM subsystems needed for full functionality
- Currently, status updates from the display are received but controlling the display needs further development

**Display Command Format**

The Display Control module manages communication with the E-ink display controller. The format of display commands uses the following structure:

| Bits | Field | Description |
| --- | --- | --- |
| 7-5 | Type | Always 0b011 for display commands |
| 4-0 | Command | Display-specific command |

Display commands currently support status reporting, with a display refresh completion notification (command 0x01, data1 0xFF).

### Debug Interface

The debug interface provides diagnostic capabilities for both the RP2040 firmware and the Linux driver. It supports compact debug codes for predefined events and text messages for more detailed information.

**When Used:**

- During development and testing
- For runtime diagnostics
- To report firmware errors or unexpected conditions
- For logging critical system events (including boot/shutdown sequences)
- To monitor communication issues between host and microcontroller

**Implementation Note:**

- Fully implemented in the kernel
- Debug messages are stored in a circular buffer
- Can be used to diagnose power management and boot/shutdown issues
- Debug categories are defined to separate different subsystems

**Debug Code Categories**

Debug codes provide a compact way to report predefined diagnostic information:

| Category Value | Category | Purpose |
| --- | --- | --- |
| 0 | System | General system events, initialization, etc. |
| 1 | Error | Error conditions and exceptions |
| 2 | Button | Button-related events and errors |
| 3 | LED | LED subsystem events and errors |
| 4 | Power | Power state changes |
| 5 | Display | Display-related events and errors |
| 6 | Communication | UART communication events and errors |
| 7 | Performance | Performance metrics and resource usage |
| 8-31 | Reserved | For future expansion |

**Power-Related Debug Codes**

The following debug codes in Category 4 (Power) can be used to diagnose power management issues:

| Code | Meaning | Example Packet |
| --- | --- | --- |
| 0x01 | Boot Started | `{0x84, 0x01, 0x00, 0x85}` |
| 0x02 | Boot Complete | `{0x84, 0x02, 0x00, 0x86}` |
| 0x03 | Shutdown Initiated | `{0x84, 0x03, 0x00, 0x87}` |
| 0x04 | Shutdown Complete | `{0x84, 0x04, 0x00, 0x80}` |
| 0x20 | Sleep Mode Entered | `{0x84, 0x20, 0x00, 0xA4}` |

**Debug Text Format**

Debug text allows sending longer text messages, potentially split across multiple packets:

| Bit | Flag | Description |
| --- | --- | --- |
| 4 | DEBUG_FIRST_CHUNK | Indicates first chunk of a message |
| 3 | DEBUG_CONTINUE | More chunks follow this one |
| 2-0 | DEBUG_CHUNK_MASK | Chunk sequence number |

## Extended Commands

Extended commands provide a framework for future expansion of the protocol without changing the basic packet structure. They are reserved for specialized or advanced functionality.

**When Used:**

- Reserved for future extensions
- For device-specific features not covered by other command types
- For experimental functionality

**Implementation Note:**

- Basic structure implemented
- Content is currently reserved for future use
- Requires updates to both firmware and driver when new commands are defined

## Versioning System

The SAM driver implements a versioning system to ensure compatibility between the Linux kernel driver and the RP2040 firmware.

**Version Format**

The version follows a standard semantic versioning format with three components:

- **Major Version**: Incremented for incompatible API changes
- **Minor Version**: Incremented for new functionality in a backward-compatible manner
- **Patch Version**: Incremented for backward-compatible bug fixes

The full version string is formatted as `MAJOR.MINOR.PATCH` (e.g., "1.0.0").

**Version Exchange Protocol**

During driver initialization, the Linux kernel sends its version information to the RP2040:

1. **Basic Version Information**: The driver sends a `SYSTEM_VERSION` packet containing the major and minor version numbers
2. **Extended Version Information**: The driver follows with an extended packet containing the patch version

This exchange allows both sides to maintain version compatibility and potentially adapt behavior based on the detected version.

**Implementation Details**

- The kernel driver defines its version in `pamir-sam.h` with the constants `PAMIR_SAM_VERSION_*`
- Version information is sent to the RP2040 during boot notification via `send_boot_notification()`
- The RP2040 firmware maintains a record of the host driver version for potential compatibility checks
- Both sides can implement version-specific behavior to maintain backward compatibility

## Userspace Interface

### Character Device

The driver creates a character device at `/dev/pamir-sam` for direct communication with the protocol handler. This allows userspace applications to:

1. Send raw packets to the microcontroller
2. Receive debug information from the microcontroller

### Input Events

Button events are reported through the Linux input subsystem and can be read from `/dev/input/event*` devices using standard input event handling.

### LED Control

LED status is exposed through the Linux LED class subsystem at `/sys/class/leds/pamir:status/`.

## Debug and Troubleshooting

### Debug Parameter

The driver supports a `debug` module parameter to control logging verbosity:

| Value | Level | Description |
| --- | --- | --- |
| 0 | `SAM_DEBUG_OFF` | No debug messages (default) |
| 1 | `SAM_DEBUG_ERROR` | Error messages only |
| 2 | `SAM_DEBUG_INFO` | Basic informational messages + errors |
| 3 | `SAM_DEBUG_VERBOSE` | Verbose debugging with detailed protocol information |

To enable debugging when loading the module, use:

```
sudo modprobe pamir-ai-sam debug=3

```

Or if built into the kernel, append to the kernel command line:

```
pamir-ai-sam.debug=3

```

### Debugging Button Input Events

To verify that button events are correctly detected and propagated through the system:

1. **Check if the SAM driver is receiving button events from the RP2040**:
    
    ```
    sudo modprobe pamir-ai-sam debug=3
    sudo dmesg -w | grep -i "button\\|input"
    
    ```
    
    When you press buttons on the device, you should see debug messages like:
    
    ```
    pamir_sam_protocol: Processing button packet
    pamir_sam_protocol: Button state: 0x01 (UP button pressed)
    
    ```
    
2. **Verify input device registration**:
    
    ```
    ls -la /dev/input/by-id/ | grep pamir
    
    ```
    
    You should see a device like:
    
    ```
    lrwxrwxrwx 1 root root 9 Jan  1 00:00 input-pamir-ai-signal-aggregation-module -> ../event5
    
    ```
    
3. **Monitor raw input events directly**:
    
    ```
    sudo apt install evtest
    sudo evtest /dev/input/by-id/input-pamir-ai-signal-aggregation-module
    
    ```
    
    Or if the by-id link isn't available, find the device by listing all input devices:
    
    ```
    sudo evtest
    # Select the "Pamir AI Signal Aggregation Module" from the list
    
    ```
    
    Then press buttons and verify events are registered.
    
4. **Check the input mapping**:
The driver maps hardware buttons to the following Linux key codes:
    
    
    | Hardware Button | Linux Key Code | Key Code Value |
    | --- | --- | --- |
    | UP | KEY_UP | 103 |
    | DOWN | KEY_DOWN | 108 |
    | SELECT | KEY_ENTER | 28 |
    | POWER | KEY_POWER | 116 |

### Button Troubleshooting

If button events aren't being detected:

1. **Hardware connection issues**:
    - Verify that the RP2040 is properly connected to the buttons
    - Check button pin configuration in the RP2040 firmware
2. **Protocol issues**:
    - Check if button packets are being sent by the RP2040:
        
        ```
        sudo cat /dev/pamir-sam | hexdump -C
        
        ```
        
        Button packets have type 0x00-0x0F in the first byte
        
    - Verify packet checksum is correct (should be XOR of first 3 bytes)
3. **Input subsystem issues**:
    - Check if input device is created properly:
        
        ```
        grep -i "pamir\\|input" /proc/devices
        
        ```
        
    - Verify input event handling in the kernel:
        
        ```
        grep -i "input" /proc/interrupts
        
        ```
        
    - Check permissions on input device:
        
        ```
        ls -la /dev/input/event* | grep -i pamir
        
        ```
        

### Testing Button Events from Userspace

You can simulate button events from userspace for testing:

1. **Find the event device ID**:
    
    ```
    grep -i "pamir" /proc/bus/input/devices
    
    ```
    
    Note the `Handlers=` line to find the event number (e.g., `event5`)
    
2. **Simulate button events** using the `evemu-event` tool:
    
    ```
    sudo apt install evemu
    sudo evemu-event /dev/input/event5 --type EV_KEY --code KEY_UP --value 1   # Press UP
    sudo evemu-event /dev/input/event5 --type EV_KEY --code KEY_UP --value 0   # Release UP
    
    ```
    

### Communication Issues

For communication problems between the driver and RP2040:

1. Enable verbose logging:
    
    ```
    sudo rmmod pamir-ai-sam
    sudo modprobe pamir-ai-sam debug=3
    
    ```
    
2. Look for checksum errors in the logs:
    
    ```
    dmesg | grep -i "checksum\\|uart\\|packet"
    
    ```
    
3. Verify that the correct UART is being used:
    
    ```
    dmesg | grep -i "serdev\\|serial"
    
    ```
    

## Example Usage

### Sending a Ping Command

To test communication with the microcontroller:

1. Open `/dev/pamir-sam`
2. Create a packet with:
    - type_flags = 0xC0 (TYPE_SYSTEM | SYSTEM_PING)
    - data[0] = 0x00
    - data[1] = 0x00
    - checksum = 0xC0 (calculated as XOR of previous bytes)
3. Write the 4-byte packet to the device
4. Read response (should be a ping acknowledgment)

### Setting LED Color

To set the LED to a specific color:

1. Open `/dev/pamir-sam`
2. Create a packet with:
    - type_flags = 0x20 (TYPE_LED | LED_MODE_STATIC)
    - data[0] = (r \<< 4) | g (where r and g are 4-bit values)
    - data[1] = (b \<< 4) | value (where b and value are 4-bit values)
    - checksum = Calculate XOR of previous bytes
3. Write the packet to the device

### Reading Button Events

Button events can be read using standard Linux input event handling libraries by opening the input device and listening for KEY_UP, KEY_DOWN, KEY_ENTER, and KEY_POWER events.

## Device Tree Configuration

Add the following to your device tree to enable the SAM driver:

```
&serial1 {
    status = "okay";
    current-speed = <115200>;

    pamir_sam: pamir-sam {
        compatible = "pamir-ai,sam";
        debug-level = <1>;                /* Optional: 0=off, 1=error, 2=info, 3=verbose */
        ack-required = <0>;               /* Optional: Whether commands require ACK */
        recovery-timeout-ms = <1000>;     /* Optional: Recovery timeout in milliseconds */
        power-poll-interval-ms = <1000>;  /* Optional: Poll power metrics every 1000ms (0 to disable) */
    };
};

```

### Device Tree Properties

| Property | Type | Default | Description |
| --- | --- | --- | --- |
| debug-level | u32 | 0 | Debug level (0=off, 1=error, 2=info, 3=verbose) |
| ack-required | boolean | false | Whether commands require acknowledgment |
| recovery-timeout-ms | u32 | 1000 | Timeout for protocol recovery in milliseconds |
| power-poll-interval-ms | u32 | 1000 | Interval for polling power metrics (ms, 0 to disable) |

## Required Kernel Modifications

To fully implement all features of the SAM protocol, the following kernel modifications are needed:

1. **LED Control**:
    - Implement the `brightness_set` callback in the LED class device
    - Add code to translate brightness values to RGB colors
2. **Power Management**:
    - Add hooks to the Linux power management subsystem to send shutdown notifications
    - Modify `/kernel/power/main.c` to notify the SAM driver before system shutdown
    - Implement a battery reporting system that interfaces with the power supply subsystem
3. **Display Control**:
    - Implement a framebuffer or DRM driver to manage display updates
    - Add ioctl commands to control display refresh modes
    - Connect to the Linux graphics subsystem for proper integration
4. **Extended Commands**:
    - Define specific extended commands as needed for advanced functionality

## Packet Reference

This section provides a comprehensive reference of all possible packet combinations for each message type in the SAM protocol.

### Button Packets (TYPE_BUTTON)

Button packets are sent from the RP2040 microcontroller to the Linux host when physical buttons are pressed or released. The 5 least significant bits indicate button state.

| Button Combination | Binary Type+Flags | Hex Type+Flags | Data Bytes | Example (Microcontroller to Host) |
| --- | --- | --- | --- | --- |
| No buttons pressed | `0b00000000` | `0x00` | `0x00 0x00` | `{0x00, 0x00, 0x00, 0x00}` |
| UP button pressed | `0b00000001` | `0x01` | `0x00 0x00` | `{0x01, 0x00, 0x00, 0x01}` |
| DOWN button pressed | `0b00000010` | `0x02` | `0x00 0x00` | `{0x02, 0x00, 0x00, 0x02}` |
| UP+DOWN pressed | `0b00000011` | `0x03` | `0x00 0x00` | `{0x03, 0x00, 0x00, 0x03}` |
| SELECT pressed | `0b00000100` | `0x04` | `0x00 0x00` | `{0x04, 0x00, 0x00, 0x04}` |
| UP+SELECT pressed | `0b00000101` | `0x05` | `0x00 0x00` | `{0x05, 0x00, 0x00, 0x05}` |
| DOWN+SELECT pressed | `0b00000110` | `0x06` | `0x00 0x00` | `{0x06, 0x00, 0x00, 0x06}` |
| UP+DOWN+SELECT pressed | `0b00000111` | `0x07` | `0x00 0x00` | `{0x07, 0x00, 0x00, 0x07}` |
| POWER pressed | `0b00001000` | `0x08` | `0x00 0x00` | `{0x08, 0x00, 0x00, 0x08}` |
| UP+POWER pressed | `0b00001001` | `0x09` | `0x00 0x00` | `{0x09, 0x00, 0x00, 0x09}` |
| DOWN+POWER pressed | `0b00001010` | `0x0A` | `0x00 0x00` | `{0x0A, 0x00, 0x00, 0x0A}` |
| UP+DOWN+POWER pressed | `0b00001011` | `0x0B` | `0x00 0x00` | `{0x0B, 0x00, 0x00, 0x0B}` |
| SELECT+POWER pressed | `0b00001100` | `0x0C` | `0x00 0x00` | `{0x0C, 0x00, 0x00, 0x0C}` |
| UP+SELECT+POWER pressed | `0b00001101` | `0x0D` | `0x00 0x00` | `{0x0D, 0x00, 0x00, 0x0D}` |
| DOWN+SELECT+POWER pressed | `0b00001110` | `0x0E` | `0x00 0x00` | `{0x0E, 0x00, 0x00, 0x0E}` |
| All buttons pressed | `0b00001111` | `0x0F` | `0x00 0x00` | `{0x0F, 0x00, 0x00, 0x0F}` |

### LED Packets (TYPE_LED)

LED packets are sent from the Linux host to the RP2040 microcontroller to control RGB LEDs.

| LED Command | Binary Type+Flags | Hex Type+Flags | Data Format | Example (Host to Microcontroller) |
| --- | --- | --- | --- | --- |
| Static, All LEDs | `0b00100000` | `0x20` | `[RRRRGGGG] [BBBBVVVV]` | `{0x20, 0xF0, 0x00, 0xD0}` (Red color) |
| Static, LED 1 | `0b00100001` | `0x21` | `[RRRRGGGG] [BBBBVVVV]` | `{0x21, 0x0F, 0x00, 0x2E}` (Green color) |
| Static, LED 2 | `0b00100010` | `0x22` | `[RRRRGGGG] [BBBBVVVV]` | `{0x22, 0x00, 0xF0, 0xD2}` (Blue color) |
| Static, LED 3 | `0b00100011` | `0x23` | `[RRRRGGGG] [BBBBVVVV]` | `{0x23, 0xFF, 0xF0, 0x2C}` (White color) |
| Blink, All LEDs | `0b00100100` | `0x24` | `[RRRRGGGG] [BBBBVVVV]` | `{0x24, 0xF0, 0x08, 0xDC}` (Red blink) |
| Blink, LED 1 | `0b00100101` | `0x25` | `[RRRRGGGG] [BBBBVVVV]` | `{0x25, 0x0F, 0x08, 0x28}` (Green blink) |
| Blink, LED 2 | `0b00100110` | `0x26` | `[RRRRGGGG] [BBBBVVVV]` | `{0x26, 0x00, 0xF8, 0xDE}` (Blue blink) |
| Blink, LED 3 | `0b00100111` | `0x27` | `[RRRRGGGG] [BBBBVVVV]` | `{0x27, 0xFF, 0xF8, 0x24}` (White blink) |
| Fade, All LEDs | `0b00101000` | `0x28` | `[RRRRGGGG] [BBBBVVVV]` | `{0x28, 0xF0, 0x08, 0xD0}` (Red fade) |
| Fade, LED 1 | `0b00101001` | `0x29` | `[RRRRGGGG] [BBBBVVVV]` | `{0x29, 0x0F, 0x08, 0x24}` (Green fade) |
| Fade, LED 2 | `0b00101010` | `0x2A` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2A, 0x00, 0xF8, 0xD2}` (Blue fade) |
| Fade, LED 3 | `0b00101011` | `0x2B` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2B, 0xFF, 0xF8, 0x28}` (White fade) |
| Rainbow, All LEDs | `0b00101100` | `0x2C` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2C, 0x00, 0x0F, 0x23}` (Rainbow, speed 15) |
| Rainbow, LED 1 | `0b00101101` | `0x2D` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2D, 0x00, 0x0F, 0x22}` (Rainbow, speed 15) |
| Rainbow, LED 2 | `0b00101110` | `0x2E` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2E, 0x00, 0x0F, 0x21}` (Rainbow, speed 15) |
| Rainbow, LED 3 | `0b00101111` | `0x2F` | `[RRRRGGGG] [BBBBVVVV]` | `{0x2F, 0x00, 0x0F, 0x20}` (Rainbow, speed 15) |
| Sequence, All LEDs | `0b00110000` | `0x30` | `[RRRRGGGG] [BBBBVVVV]` | `{0x30, 0x55, 0x05, 0x60}` (Sequence command) |

### Power Management Packets (TYPE_POWER)

Power packets manage power states between the Linux host and RP2040.

| Power Command | Binary Type+Flags | Hex Type+Flags | Data Format | Example Usage |
| --- | --- | --- | --- | --- |
| Query Status | `0b01000000` | `0x40` | `[State] [Reserved]` | **MC→Host**: `{0x40, 0x01, 0x00, 0x41}` (Running state) |
| Query Temperature | `0b01000010` | `0x42` | `[TempH] [TempL]` | **MC→Host**: `{0x42, 0x01, 0x18, 0x53}` (28°C) |
| Set Power State | `0b01010000` | `0x50` | `[State] [Flags]` | **Host→MC**: `{0x50, 0x01, 0x00, 0x51}` (Set running) |
| Set Parameters | `0b01010001` | `0x51` | `[Param] [Value]` | **Host→MC**: `{0x51, 0x01, 0x0A, 0x5A}` (Set parameter) |
| Enter Sleep | `0b01100000` | `0x60` | `[Delay] [Flags]` | **Host→MC**: `{0x60, 0x0A, 0x00, 0x6A}` (Sleep after 10s) |
| Deep Sleep | `0b01100001` | `0x61` | `[Delay] [Flags]` | **Host→MC**: `{0x61, 0x00, 0x00, 0x61}` (Immediate deep sleep) |
| System Shutdown | `0b01110000` | `0x70` | `[Mode] [Flags]` | **Host→MC**: `{0x70, 0x00, 0x00, 0x70}` (Normal shutdown) |
| Emergency Shutdown | `0b01110001` | `0x71` | `[Reason] [Flags]` | **Host→MC**: `{0x71, 0x01, 0x00, 0x70}` (Thermal shutdown) |
| Request Metrics | `0b10000000` | `0x80` | `[0x00] [0x00]` | **Host→MC**: `{0x80, 0x00, 0x00, 0x80}` (Request all metrics) |

### Display Control Packets (TYPE_DISPLAY)

Display packets control the E-ink display on the CM5 device.

| Display Command | Binary Type+Flags | Hex Type+Flags | Data Format | Example Usage |
| --- | --- | --- | --- | --- |
| Status Report | `0b01100000` | `0x60` | `[Status] [Flags]` | **MC→Host**: `{0x60, 0x01, 0x00, 0x61}` (Ready status) |
| Refresh Complete | `0b01100001` | `0x61` | `[Time] [Flags]` | **MC→Host**: `{0x61, 0xFF, 0x00, 0x9E}` (Refresh complete) |
| Full Refresh | `0b01100010` | `0x62` | `[Mode] [Flags]` | **Host→MC**: `{0x62, 0x01, 0x00, 0x63}` (Full refresh) |
| Partial Update | `0b01100011` | `0x63` | `[Region] [Mode]` | **Host→MC**: `{0x63, 0x01, 0x00, 0x62}` (Partial update region 1) |
| Display Sleep | `0b01100100` | `0x64` | `[Mode] [Timeout]` | **Host→MC**: `{0x64, 0x01, 0x3C, 0x59}` (Sleep after 60s) |
| Display Wake | `0b01100101` | `0x65` | `[Mode] [Flags]` | **Host→MC**: `{0x65, 0x00, 0x00, 0x65}` (Wake display) |
| Set Contrast | `0b01100110` | `0x66` | `[Level] [Mode]` | **Host→MC**: `{0x66, 0x80, 0x00, 0xE6}` (Set 50% contrast) |
| Set Orientation | `0b01100111` | `0x67` | `[Orient] [Flags]` | **Host→MC**: `{0x67, 0x01, 0x00, 0x66}` (Landscape orientation) |

### Debug Code Packets (TYPE_DEBUG_CODE)

Debug code packets provide diagnostic information from the RP2040 to the Linux host.

| Debug Code Type | Binary Type+Flags | Hex Type+Flags | Data Format | Example (Microcontroller to Host) |
| --- | --- | --- | --- | --- |
| System Info (Cat 0) | `0b10000000` | `0x80` | `[Code] [Param]` | `{0x80, 0x01, 0x00, 0x81}` (System startup) |
| Error (Cat 1) | `0b10000001` | `0x81` | `[Code] [Param]` | `{0x81, 0xFF, 0x02, 0x7C}` (Critical error) |
| Button (Cat 2) | `0b10000010` | `0x82` | `[Code] [Param]` | `{0x82, 0x01, 0x05, 0x86}` (Button error) |
| LED (Cat 3) | `0b10000011` | `0x83` | `[Code] [Param]` | `{0x83, 0x02, 0x00, 0x81}` (LED driver error) |
| Power (Cat 4) | `0b10000100` | `0x84` | `[Code] [Param]` | `{0x84, 0x03, 0x32, 0xB5}` (Battery at 50%) |
| Display (Cat 5) | `0b10000101` | `0x85` | `[Code] [Param]` | `{0x85, 0x01, 0x00, 0x84}` (Display error) |
| Communication (Cat 6) | `0b10000110` | `0x86` | `[Code] [Param]` | `{0x86, 0x01, 0x00, 0x87}` (Comm error) |
| Performance (Cat 7) | `0b10000111` | `0x87` | `[Code] [Param]` | `{0x87, 0x02, 0x19, 0xAE}` (High CPU usage) |

### Debug Text Packets (TYPE_DEBUG_TEXT)

Debug text packets allow sending text messages from the RP2040 to the Linux host, potentially split over multiple packets.

| Debug Text Type | Binary Type+Flags | Hex Type+Flags | Data Format | Example (Microcontroller to Host) |
| --- | --- | --- | --- | --- |
| First chunk | `0b10110000` | `0xB0` | `[Char1] [Char2]` | `{0xB0, 0x48, 0x65, 0xD5}` ("He") |
| Continuation, last chunk | `0b10100000` | `0xA0` | `[Char1] [Char2]` | `{0xA0, 0x6C, 0x6C, 0xC0}` ("ll") |
| First chunk + more follow | `0b10111000` | `0xB8` | `[Char1] [Char2]` | `{0xB8, 0x42, 0x6F, 0xC5}` ("Bo") |
| Continuation + more follow | `0b10101000` | `0xA8` | `[Char1] [Char2]` | `{0xA8, 0x6F, 0x74, 0xC1}` ("ot") |
| First chunk, index 1 | `0b10110001` | `0xB1` | `[Char1] [Char2]` | `{0xB1, 0x44, 0x65, 0xD0}` ("De") |
| Continuation, index 1 | `0b10100001` | `0xA1` | `[Char1] [Char2]` | `{0xA1, 0x62, 0x75, 0xC6}` ("bu") |
| First + more, index 2 | `0b10110010` | `0xB2` | `[Char1] [Char2]` | `{0xB2, 0x45, 0x72, 0xD5}` ("Er") |
| Continuation + more, index 2 | `0b10101010` | `0xAA` | `[Char1] [Char2]` | `{0xAA, 0x72, 0x6F, 0xC5}` ("ro") |

### System Command Packets (TYPE_SYSTEM)

System command packets provide core control and monitoring functions.

| System Command | Binary Type+Flags | Hex Type+Flags | Data Format | Example Usage |
| --- | --- | --- | --- | --- |
| Ping | `0b11000000` | `0xC0` | `[0x00] [0x00]` | **Host→MC**: `{0xC0, 0x00, 0x00, 0xC0}` (Ping request)<br>**MC→Host**: `{0xC0, 0x00, 0x00, 0xC0}` (Ping response) |
| Reset | `0b11000001` | `0xC1` | `[Mode] [Reason]` | **Host→MC**: `{0xC1, 0x00, 0x00, 0xC1}` (Normal reset)<br>**MC→Host**: `{0xC1, 0x01, 0x00, 0xC0}` (Reset notification) |
| Version Request | `0b11000010` | `0xC2` | `[0x00] [0x00]` | **Host→MC**: `{0xC2, 0x00, 0x00, 0xC2}` (Request version) |
| Version Response | `0b11000010` | `0xC2` | `[Major] [Minor]` | **MC→Host**: `{0xC2, 0x01, 0x05, 0xC6}` (v1.5) |
| Status Request | `0b11000011` | `0xC3` | `[Type] [0x00]` | **Host→MC**: `{0xC3, 0x01, 0x00, 0xC2}` (Request status) |
| Status Response | `0b11000011` | `0xC3` | `[Status] [Error]` | **MC→Host**: `{0xC3, 0x01, 0x00, 0xC2}` (Status OK) |
| Config Get | `0b11000100` | `0xC4` | `[0x00] [Param]` | **Host→MC**: `{0xC4, 0x00, 0x01, 0xC5}` (Get debug level) |
| Config Response | `0b11000100` | `0xC4` | `[Value] [Param]` | **MC→Host**: `{0xC4, 0x01, 0x01, 0xC4}` (Debug level 1) |
| Config Set | `0b11000100` | `0xC4` | `[0x01] [Param+Value]` | **Host→MC**: `{0xC4, 0x01, 0x21, 0xE4}` (Set debug level 2) |
| Sync Request | `0b11000101` | `0xC5` | `[Mode] [0x00]` | **Host→MC**: `{0xC5, 0x01, 0x00, 0xC4}` (Sync request) |
| Sync Response | `0b11000101` | `0xC5` | `[Status] [Value]` | **MC→Host**: `{0xC5, 0x01, 0x00, 0xC4}` (Sync OK) |

### Extended Command Packets (TYPE_EXTENDED)

Extended command packets provide a framework for future expansion.

| Extended Command | Binary Type+Flags | Hex Type+Flags | Data Format | Example Usage |
| --- | --- | --- | --- | --- |
| Reserved 0 | `0b11100000` | `0xE0` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE0, 0x00, 0x00, 0xE0}` (Reserved) |
| Extended Version | `0b11100001` | `0xE1` | `[Patch] [Rsv]` | **Host→MC**: `{0xE1, 0x01, 0x00, 0xE0}` (Patch version 1) |
| Reserved 2 | `0b11100010` | `0xE2` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE2, 0x00, 0x00, 0xE2}` (Reserved) |
| Reserved 3 | `0b11100011` | `0xE3` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE3, 0x00, 0x00, 0xE3}` (Reserved) |
| Reserved 4 | `0b11100100` | `0xE4` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE4, 0x00, 0x00, 0xE4}` (Reserved) |
| Reserved 5 | `0b11100101` | `0xE5` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE5, 0x00, 0x00, 0xE5}` (Reserved) |
| Reserved 6 | `0b11100110` | `0xE6` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE6, 0x00, 0x00, 0xE6}` (Reserved) |
| Reserved 7 | `0b11100111` | `0xE7` | `[Cmd] [Param]` | **Host→MC/MC→Host**: `{0xE7, 0x00, 0x00, 0xE7}` (Reserved) |

## Performance Considerations

The SAM protocol is designed for embedded systems with limited resources. To ensure optimal performance:

### Host-side Considerations

1. **Command Batching:**
    - Group related LED commands together to minimize UART overhead
    - Avoid sending individual commands in rapid succession
2. **Polling Frequency:**
    - Power metrics polling interval should not be less than 100ms
    - Button events are rate-limited to 20 per second per button
3. **Resource Utilization:**
    - The driver uses approximately 12KB of kernel memory
    - UART interrupt handling has minimal CPU impact (<0.1%)
    - Debug logging at level 3 can impact performance on heavily loaded systems

### Microcontroller Considerations

1. **Processing Capacity:**
    - RP2040 can handle up to 1000 packets per second
    - Display operations block one core during refresh
    - LED animations are processed in background tasks
2. **Memory Usage:**
    - Protocol buffer: 256 bytes (64 packets)
    - LED animation queue: 16 steps per LED
    - Debug text buffer: 256 bytes
3. **Power Implications:**
    - High packet rates increase power consumption
    - LED animations significantly impact battery life
    - Sleep modes reduce consumption by up to 95%

### Communication Efficiency

1. **Optimization Techniques:**
    - Use debug codes instead of debug text when possible
    - Employ the queue-based LED control to reduce packet count
    - Minimize polling frequency for non-critical metrics
2. **Throughput Limits:**
    - Theoretical maximum: ~36,000 packets/second at 115200 baud
    - Practical maximum: ~5,000 packets/second with processing overhead
    - Recommended sustained rate: <1,000 packets/second for reliable operation

## Error Handling and Recovery

The SAM protocol implements multiple levels of error detection and recovery mechanisms:

### Error Detection

1. **Packet Integrity:**
    - XOR checksum validates each packet
    - Detects any single-bit error and many multi-bit errors
2. **Protocol Violations:**
    - Invalid command codes are detected and logged
    - Out-of-sequence packets are identified
3. **Timing Violations:**
    - Timeouts for expected acknowledgments
    - Detection of stalled transactions

### Recovery Mechanisms

1. **Automatic Retransmission:**
    - Critical commands can be configured to require acknowledgment
    - Automatic retransmission up to 3 times for unacknowledged packets
2. **Protocol Resynchronization:**
    - Flush buffers after detecting checksum errors
    - Resynchronize on system command packets
3. **Watchdog Protection:**
    - Hardware watchdog on RP2040 (2-second timeout)
    - Software watchdog in Linux driver (configurable timeout)

### Error Reporting

1. **Debug Codes:**
    - Error category (0x01) for protocol errors
    - Specific error codes for different failure types
2. **Kernel Logging:**
    - Warning and error messages logged to kernel log
    - Error counts maintained in driver statistics
3. **Userspace Notification:**
    - Critical errors reported via sysfs attributes
    - Error state accessible through character device

### Recovery Procedure

When checksum errors are detected, the following recovery sequence is initiated:

1. Discard the current packet
2. Log the error (visible with debug level ≥ 1)
3. Flush receive buffer to clear potentially corrupted data
4. Send a SYSTEM_PING command to verify communication
5. If ping fails, escalate to full protocol reset
6. Reinitialize protocol state and notify upper layers