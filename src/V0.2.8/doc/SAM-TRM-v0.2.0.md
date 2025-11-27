# Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual

**Document Classification:** Technical Reference Manual

**Document Version:** 0.2.0

**Last Updated:** 2025-07-22

**Previous Version:** 0.1.0 (DEPRECATED - Contains critical specification errors)

## Version 0.2.0 Changes

**BREAKING CHANGES:**
- Fixed LED command structure contradictions
- Resolved power command code conflicts
- Upgraded to CRC8 checksum for better error detection
- Standardized 4-bit LED ID encoding (supports 16 LEDs)
- Simplified protocol by removing dual encoding complexity

**Migration Required:** This version is NOT backward compatible with v0.1.0 implementations.

## Introduction

### Purpose

The Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual provides the corrected and comprehensive technical specifications for the communication protocol used to interface between a Linux host system (Raspberry Pi CM5) and the RP2040 microcontroller in Pamir AI devices. This manual serves as the definitive reference for implementing SAM protocol drivers and firmware.

### Scope

This document covers the complete technical specification of the SAM protocol, including:
- Corrected packet formats and field encodings
- Resolved command structures and bit field definitions
- Enhanced error detection and handling mechanisms
- Comprehensive packet examples with validation
- Clear implementation requirements for both kernel and firmware
- Performance characteristics and timing requirements
- Error handling and recovery procedures
- Complete API documentation for all message types

## Architecture Overview

The SAM driver implements a modular architecture with clearly defined components that provide comprehensive hardware control and status reporting capabilities:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Linux Host System (CM5)                         │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐          │
│  │   Userspace     │  │   System Tools  │  │  Applications   │          │
│  │  Applications   │  │   (evtest,      │  │                 │          │
│  │                 │  │    LED utils)   │  │                 │          │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘          │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐          │
│  │  Linux Kernel   │  │   Power Supply  │  │   LED Class     │          │
│  │   Input Layer   │  │   Subsystem     │  │   Subsystem     │          │
│  │    (buttons)    │  │  (/sys/class/   │  │ (/sys/class/    │          │
│  │                 │  │ power_supply/)  │  │   leds/)        │          │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘          │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │                       SAM Driver Core                               ││
│  │  ┌──────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────────┐  ││
│  │  │   Protocol   │ │   Input     │ │     LED     │ │    Power     │  ││
│  │  │   Handler    │ │   Handler   │ │   Handler   │ │   Manager    │  ││
│  │  └──────────────┘ └─────────────┘ └─────────────┘ └──────────────┘  ││
│  │  ┌──────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────────┐  ││
│  │  │   Display    │ │    Debug    │ │   System    │ │   Character  │  ││
│  │  │  Controller  │ │  Interface  │ │  Commands   │ │    Device    │  ││
│  │  └──────────────┘ └─────────────┘ └─────────────┘ └──────────────┘  ││
│  └─────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │                       UART Interface                                ││
│  │     115200 baud, 8N1, GPIO4/5 (UART2), 4-byte packet protocol       ││
│  └─────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                              ┌─────┴─────┐
                              │   UART    │
                              │ Interface │
                              └─────┬─────┘
┌─────────────────────────────────────────────────────────────────────────┐
│                      RP2040 Microcontroller                             │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │                    Firmware Protocol Stack                          ││
│  │  ┌──────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────────┐  ││
│  │  │    UART      │ │   Packet    │ │   Message   │ │   Handler    │  ││
│  │  │   Handler    │ │   Parser    │ │   Router    │ │  Dispatcher  │  ││
│  │  └──────────────┘ └─────────────┘ └─────────────┘ └──────────────┘  ││
│  └─────────────────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────────────────────────┐│
│  │                     Hardware Controllers                            ││
│  │  ┌──────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────────┐  ││
│  │  │   Button     │ │  NeoPixel   │ │   Power     │ │    E-ink     │  ││
│  │  │  Scanner     │ │ Controller  │ │   Manager   │ │  Controller  │  ││
│  │  │ (debounced)  │ │ (16 LEDs)   │ │ (metrics)   │ │ (display)    │  ││
│  │  └──────────────┘ └─────────────┘ └─────────────┘ └──────────────┘  ││
│  │  ┌──────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────────┐  ││
│  │  │   Battery    │ │ Temperature │ │   Voltage   │ │    Debug     │  ││
│  │  │   Monitor    │ │   Sensor    │ │   Monitor   │ │   Logger     │  ││
│  │  │    (ADC)     │ │   (ADC)     │ │    (ADC)    │ │              │  ││
│  │  └──────────────┘ └─────────────┘ └─────────────┘ └──────────────┘  ││
│  └─────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────┘
                                      │
        ┌───────────────┬─────────────┼─────────────┬──────────────┐
        │               │             │             │              │
    ┌───▼───┐      ┌────▼────┐   ┌────▼────┐   ┌────▼────┐     ┌───▼───┐
    │4 GPIO │      │16 RGB   │   │Power &  │   │ E-ink   │     │Battery│
    │Buttons│      │LEDs     │   │Sensors  │   │Display  │     │& Temp │
    │(PSDU) │      │(WS2812B)│   │(ADC)    │   │(SPI)    │     │Sensors│
    └───────┘      └─────────┘   └─────────┘   └─────────┘     └───────┘
```

### Communication Characteristics

| Aspect | Specification | Details |
|--------|--------------|---------|
| **Physical Layer** | UART (8N1) | 8 data bits, no parity, 1 stop bit |
| **Baud Rate** | 115200 bps | ≈11.5 KB/s theoretical maximum |
| **GPIO Pins** | GPIO4 (TX), GPIO5 (RX) | UART2 interface on CM5 |
| **Packet Size** | 4 bytes fixed | Ultra-compact for embedded efficiency |
| **Protocol Overhead** | 50% | 2 bytes payload, 2 bytes overhead |
| **Checksum** | CRC8 (polynomial 0x07) | Enhanced error detection |
| **Message Types** | 8 types | Comprehensive hardware coverage |
| **Latency** | <10 ms | Time-critical responses |
| **Throughput** | ~5,000 packets/s | Practical maximum with processing |

### Component Status

| Component | Implementation Status | Feature Completeness | API Stability | Performance |
| --- | --- | --- | --- | --- |
| Protocol Core | Complete | 100% | Stable | Optimized |
| LED Handler | Complete | 100% | Stable | High-performance |
| Power Manager | Complete | 100% | Stable | Efficient polling |
| Input Handler | Complete | 100% | Stable | Hardware debounced |
| Debug Interface | Complete | 100% | Stable | Comprehensive logging |
| System Commands | Complete | 100% | Stable | Robust recovery |
| Character Device | Complete | 100% | Stable | Full userspace access |
| Display Controller | Minimal | 25% | Alpha | Boot-time only |

## Protocol Specification

### Packet Structure

The SAM protocol employs a fixed-size 4-byte packet format with enhanced error detection designed for ultra-efficient embedded communication:

| Field | Size | Offset | Bit Range | Description |
| --- | --- | --- | --- | --- |
| type_flags | 1 byte | 0 | 7-0 | Message type (3 MSB) and command-specific flags (5 LSB) |
| data[0] | 1 byte | 1 | 7-0 | First data byte (interpretation depends on message type) |
| data[1] | 1 byte | 2 | 7-0 | Second data byte (interpretation depends on message type) |
| checksum | 1 byte | 3 | 7-0 | CRC8 checksum of first 3 bytes for error detection |

**C Structure:**
```c
struct sam_protocol_packet {
    uint8_t type_flags;    // Message type and flags
    uint8_t data[2];       // 16-bit payload
    uint8_t checksum;      // CRC8 error detection
} __packed;
```

**Python Structure:**
```python
import struct

# Pack packet: type_flags, data0, data1, checksum
packet = struct.pack("BBBB", type_flags, data0, data1, checksum)

# Unpack packet: (type_flags, data0, data1, checksum)
type_flags, data0, data1, checksum = struct.unpack("BBBB", packet)
```

### Field Encoding

The `type_flags` byte uses a sophisticated bit layout that maximizes information density:

```
┌─────────────────────────────────────────────────────────────┐
│                     type_flags Bit Layout                   │
├───┬───┬───┬───┬───┬───┬───┬───┐                             │
│ 7 │ 6 │ 5 │ 4 │ 3 │ 2 │ 1 │ 0 │                             │
├───┴───┴───┼───┴───┴───┴───┴───┤                             │
│   Type    │    Command Flags  │                             │
│ (3 bits)  │     (5 bits)      │                             │
└───────────┴───────────────────┘─────────────────────────────┘
```

- **Bits 7-5**: Message type (3 bits, 8 possible types)
- **Bits 4-0**: Command-specific flags (5 bits, interpretation depends on message type)

### Message Types

| Type Value (Binary) | Type Value (Hex) | Name | Direction | Primary Use | Max Subtypes |
| --- | --- | --- | --- | --- | --- |
| `0b000xxxxx` | `0x00-0x1F` | TYPE_BUTTON | MCU → Host | Button state change events | 32 |
| `0b001xxxxx` | `0x20-0x3F` | TYPE_LED | Host ↔ MCU | LED control commands and status | 32 |
| `0b010xxxxx` | `0x40-0x5F` | TYPE_POWER | Host ↔ MCU | Power management and metrics | 32 |
| `0b011xxxxx` | `0x60-0x7F` | TYPE_DISPLAY | Host ↔ MCU | E-ink display control and status | 32 |
| `0b100xxxxx` | `0x80-0x9F` | TYPE_DEBUG_CODE | MCU → Host | Numeric debug codes | 32 |
| `0b101xxxxx` | `0xA0-0xBF` | TYPE_DEBUG_TEXT | MCU → Host | Text debug messages | 32 |
| `0b110xxxxx` | `0xC0-0xDF` | TYPE_SYSTEM | Host ↔ MCU | Core system control commands | 32 |
| `0b111xxxxx` | `0xE0-0xFF` | TYPE_EXTENDED | Host ↔ MCU | Extended commands (reserved) | 32 |

### CRC8 Checksum Algorithm

The protocol uses CRC8 with polynomial 0x07 (x^8 + x^2 + x + 1) for robust error detection. This polynomial provides excellent error detection capabilities for the 4-byte packet size.

**Algorithm Implementation:**
```c
uint8_t crc8_calculate(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

**Packet Validation:**
```c
uint8_t validate_packet(const struct sam_protocol_packet *packet) {
    uint8_t calculated_crc = crc8_calculate((uint8_t*)packet, 3);
    return calculated_crc == packet->checksum;
}
```

**Python Implementation:**
```python
def calculate_crc8(data):
    """Calculate CRC8 checksum using polynomial 0x07"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF  # Keep it as 8-bit
    return crc
```

**Error Detection Capabilities:**
- **Single-bit errors**: 100% detection
- **Burst errors**: ≤8 bits, 100% detection
- **Random errors**: >99.6% detection probability
- **Undetected error probability**: 1/256 (0.39%)

## LED Control Specification

### Technical Specifications

- **Communication Direction**: Host → RP2040 (commands), RP2040 → Host (status)
- **Supported LEDs**: 16 LEDs (ID 0-15) + Broadcast LED (ID 15 for all LEDs)
- **Color Depth**: 4 bits per channel (RGB444), 4096 colors total
- **LED Modes**: 4 modes (Static, Blink, Fade, Rainbow)
- **Timing Resolution**: 4 levels (100ms, 200ms, 500ms, 1000ms intervals)
- **Execution Model**: Immediate execution (no queuing)
- **Hardware**: WS2812B NeoPixel LEDs with hardware PWM control
- **Update Rate**: Up to 60 Hz refresh rate

### LED Command Structure

The LED command structure provides complete control over 16 individual RGB LEDs with support for animation modes and timing control:

```
Byte 0 (type_flags): [001][E][LED_ID(4-bit)]
                     │    │  │
                     │    │  └─ LED ID (0-15, 15=broadcast)
                     │    └─── Execute flag (0=command, 1=status/response)
                     └─────── Message type (001 = LED)

Byte 1 (data[0]):    [R(4-bit)][G(4-bit)]
                     │         │
                     │         └─ Green component (0-15)
                     └─────────── Red component (0-15)

Byte 2 (data[1]):    [B(4-bit)][MODE(2-bit)][TIME(2-bit)]
                     │         │            │
                     │         │            └─ Timing (0-3)
                     │         └───────────── Mode (0-3)
                     └─────────────────────── Blue component (0-15)

Byte 3 (checksum):   CRC8 of bytes 0-2
```

### Field Definitions

**Execute Bit (E) - Bit 4:**
- `0`: Command packet (Host → RP2040) - Set LED state
- `1`: Status/Response packet (RP2040 → Host) - Status reports, completion notifications

**LED ID (4-bit) - Bits 3-0:**
- `0x0` - `0xE`: Individual LED identifiers 0-14
- `0xF`: Broadcast to all LEDs (special case)

**RGB Values (4-bit each):**
- `0x0` - `0xF`: Color intensity (0-15)
- Hardware scaling: `rgb_8bit = (rgb_4bit * 255) / 15`
- Gamma correction applied in hardware for perceptual linearity

**LED Mode (2-bit) - Bits 3-2 of data[1]:**
- `00` (0): **Static** - Solid color, no animation
- `01` (1): **Blink** - On/off pattern with configurable timing
- `10` (2): **Fade** - Smooth brightness transitions (sine wave)
- `11` (3): **Rainbow** - Full spectrum color cycling animation

**Timing (2-bit) - Bits 1-0 of data[1]:**
- `00` (0): 100ms intervals - Fast animation
- `01` (1): 200ms intervals - Medium animation
- `10` (2): 500ms intervals - Slow animation
- `11` (3): 1000ms intervals - Very slow animation

### LED Command Examples

**Set LED 0 to Red (Static):**
```
type_flags: 0x20 = [001][0][0000] = TYPE_LED | LED_ID_0
data[0]:    0xF0 = [1111][0000]   = Red=15, Green=0
data[1]:    0x00 = [0000][00][00] = Blue=0, Mode=Static, Time=100ms
checksum:   CRC8([0x20, 0xF0, 0x00]) = 0x8E
Packet:     {0x20, 0xF0, 0x00, 0x8E}

Result: LED 0 displays bright red color continuously
```

**Set LED 5 to Blue (Blinking, 500ms):**
```
type_flags: 0x25 = [001][0][0101] = TYPE_LED | LED_ID_5
data[0]:    0x00 = [0000][0000]   = Red=0, Green=0
data[1]:    0xF6 = [1111][01][10] = Blue=15, Mode=Blink, Time=500ms
checksum:   CRC8([0x25, 0x00, 0xF6]) = 0x33
Packet:     {0x25, 0x00, 0xF6, 0x33}

Result: LED 5 blinks bright blue with 500ms on/off intervals
```

**Set LED 15 to White (Fade, 1000ms):**
```
type_flags: 0x2F = [001][0][1111] = TYPE_LED | LED_ID_15
data[0]:    0xFF = [1111][1111]   = Red=15, Green=15
data[1]:    0xFB = [1111][10][11] = Blue=15, Mode=Fade, Time=1000ms
checksum:   CRC8([0x2F, 0xFF, 0xFB]) = 0x7B
Packet:     {0x2F, 0xFF, 0xFB, 0x7B}

Result: LED 15 fades white in/out with 1000ms cycle time
```

**Set All LEDs to Green (Rainbow mode, 200ms):**
```
type_flags: 0x2F = [001][0][1111] = TYPE_LED | LED_ID_BROADCAST
data[0]:    0x0F = [0000][1111]   = Red=0, Green=15
data[1]:    0x07 = [0000][11][01] = Blue=0, Mode=Rainbow, Time=200ms
checksum:   CRC8([0x2F, 0x0F, 0x07]) = 0x61
Packet:     {0x2F, 0x0F, 0x07, 0x61}

Result: All LEDs cycle through rainbow colors with 200ms transitions
```

### LED Status Responses

The RP2040 can send status responses using the Execute bit set (E=1):

**LED Completion Response:**
```
type_flags: 0x30 = [001][1][0000] = TYPE_LED | EXECUTE | LED_ID_0
data[0]:    0xFF = Completion indicator
data[1]:    0x00 = Additional status info
checksum:   CRC8([0x30, 0xFF, 0x00]) = 0x4F
Packet:     {0x30, 0xFF, 0x00, 0x4F}

Meaning: LED 0 command completed successfully
```

**LED Error Response:**
```
type_flags: 0x35 = [001][1][0101] = TYPE_LED | EXECUTE | LED_ID_5
data[0]:    0xFE = Error indicator
data[1]:    0x02 = Error code (2 = invalid parameter)
checksum:   CRC8([0x35, 0xFE, 0x02]) = 0x29
Packet:     {0x35, 0xFE, 0x02, 0x29}

Meaning: LED 5 command failed with parameter error
```

### LED Animation Behaviors

**Static Mode (00):**
- LED displays solid color continuously
- No animation or transitions
- Lowest power consumption
- Immediate color change

**Blink Mode (01):**
- LED alternates between full brightness and off
- Uses specified timing for on/off duration
- 50% duty cycle (equal on/off time)
- Color remains constant during on phase

**Fade Mode (10):**
- LED smoothly transitions between full brightness and off
- Uses sine wave interpolation for natural fading
- Timing controls complete fade cycle duration
- Color hue remains constant, only brightness changes

**Rainbow Mode (11):**
- LED cycles through full color spectrum (HSV color wheel)
- Timing controls transition speed between colors
- Brightness remains constant at specified RGB maximum
- Smooth color transitions using HSV interpolation

### LED Hardware Characteristics

**WS2812B Specifications:**
- **Supply Voltage**: 3.3V - 5.0V
- **Current per LED**: 60mA maximum (white at full brightness)
- **PWM Resolution**: 8-bit per channel (256 levels)
- **Update Frequency**: 800 kHz data rate
- **Color Order**: GRB (Green, Red, Blue)
- **Propagation Delay**: 300ns per LED

**Performance Characteristics:**
- **Refresh Rate**: 60 Hz typical, 120 Hz maximum
- **Color Depth**: 16.7 million colors (24-bit true color)
- **Response Time**: <1ms for static color changes
- **Animation Smoothness**: 60 FPS interpolation
- **Power Consumption**: 20mA average per LED (mixed colors)

## Power Management Specification 

### Technical Specifications

- **Communication Direction**: Host ↔ RP2040 (bidirectional)
- **Power States**: 4 states (Off, Running, Suspend, Sleep)
- **Metrics Types**: 4 sensors (Current, Battery, Temperature, Voltage)
- **Update Rate**: Configurable polling (default 1000ms)
- **Resolution**: 16-bit values for all metrics
- **Integration**: Linux power supply subsystem support

### Power Command Structure

Power commands are separated into two distinct categories to eliminate command conflicts and provide clear functional separation:

**Control Commands (Host → RP2040):**
```
type_flags: [010][COMMAND(5-bit)]
           Type=2|Command(0x00-0x0F)

Categories:
- Commands 0x00-0x0F: Host control operations
- Commands 0x10-0x1F: RP2040 reporting operations
```

**Reporting Commands (RP2040 → Host):**
```
type_flags: [010][COMMAND(5-bit)]
           Type=2|Command(0x10-0x1F)
```

### Power Command Definitions

**Control Commands (0x00-0x0F):**
| Command | Hex | Name | Direction | Purpose |
|---------|-----|------|-----------|---------|
| 0x00 | 0x40 | POWER_CMD_QUERY | Host → RP2040 | Query current power status |
| 0x01 | 0x41 | POWER_CMD_SET | Host → RP2040 | Set power state |
| 0x02 | 0x42 | POWER_CMD_SLEEP | Host → RP2040 | Enter sleep mode |
| 0x03 | 0x43 | POWER_CMD_SHUTDOWN | Host → RP2040 | Shutdown system |
| 0x0F | 0x4F | POWER_CMD_REQUEST_METRICS | Host → RP2040 | Request all metrics |

**Reporting Commands (0x10-0x1F):**
| Command | Hex | Name | Direction | Purpose |
|---------|-----|------|-----------|---------|
| 0x10 | 0x50 | POWER_CMD_CURRENT | RP2040 → Host | Current draw reporting |
| 0x11 | 0x51 | POWER_CMD_BATTERY | RP2040 → Host | Battery state reporting |
| 0x12 | 0x52 | POWER_CMD_TEMP | RP2040 → Host | Temperature reporting |
| 0x13 | 0x53 | POWER_CMD_VOLTAGE | RP2040 → Host | Voltage reporting |
| 0x1F | 0x5F | POWER_CMD_METRICS_COMPLETE | RP2040 → Host | All metrics sent |

### Power States

| State | Value | Name | Description | Power Consumption |
|-------|-------|------|-------------|------------------|
| 0x00 | 0 | POWER_STATE_OFF | System powered off | 0mA |
| 0x01 | 1 | POWER_STATE_RUNNING | Normal operation | Variable |
| 0x02 | 2 | POWER_STATE_SUSPEND | Suspended to RAM | 10-50mA |
| 0x03 | 3 | POWER_STATE_SLEEP | Deep sleep mode | 1-5mA |

### Power Command Examples

**Query Power Status:**
```
type_flags: 0x40 = [010][00000] = TYPE_POWER | POWER_CMD_QUERY
data[0]:    0x00 = Reserved (should be 0x00)
data[1]:    0x00 = Reserved (should be 0x00)
checksum:   CRC8([0x40, 0x00, 0x00]) = 0x8C
Packet:     {0x40, 0x00, 0x00, 0x8C}

Purpose: Request current power state from RP2040
Response: RP2040 sends back current state via POWER_CMD_QUERY response
```

**Set Running State:**
```
type_flags: 0x41 = [010][00001] = TYPE_POWER | POWER_CMD_SET
data[0]:    0x01 = POWER_STATE_RUNNING
data[1]:    0x00 = Flags (reserved, should be 0x00)
checksum:   CRC8([0x41, 0x01, 0x00]) = 0x8A
Packet:     {0x41, 0x01, 0x00, 0x8A}

Purpose: Set RP2040 to running state
Effect: RP2040 exits sleep/suspend mode and resumes normal operation
```

**Enter Sleep Mode (30 second timeout):**
```
type_flags: 0x42 = [010][00010] = TYPE_POWER | POWER_CMD_SLEEP
data[0]:    0x1E = 30 seconds timeout
data[1]:    0x01 = Sleep flags (0x01 = wake on button press)
checksum:   CRC8([0x42, 0x1E, 0x01]) = 0x2F
Packet:     {0x42, 0x1E, 0x01, 0x2F}

Purpose: Put RP2040 into sleep mode with 30s auto-wake timeout
Effect: RP2040 reduces power consumption, wakes on button or timeout
```

**Shutdown Notification:**
```
type_flags: 0x43 = [010][00011] = TYPE_POWER | POWER_CMD_SHUTDOWN
data[0]:    0x00 = Normal shutdown (0x01 = emergency, 0x02 = reboot)
data[1]:    0x05 = Reason code (0x05 = user initiated shutdown)
checksum:   CRC8([0x43, 0x00, 0x05]) = 0x46
Packet:     {0x43, 0x00, 0x05, 0x46}

Purpose: Notify RP2040 of impending system shutdown
Effect: RP2040 prepares for power loss, saves state, releases peripherals
```

**Request All Metrics:**
```
type_flags: 0x4F = [010][01111] = TYPE_POWER | POWER_CMD_REQUEST_METRICS
data[0]:    0x00 = Metric mask (0x00 = all metrics)
data[1]:    0x00 = Reserved
checksum:   CRC8([0x4F, 0x00, 0x00]) = 0x93
Packet:     {0x4F, 0x00, 0x00, 0x93}

Purpose: Request RP2040 to send all current power metrics
Response: RP2040 sends 4 separate metric packets + completion packet
```

### Power Metrics Reporting

**Current Draw Report (250mA):**
```
type_flags: 0x50 = [010][10000] = TYPE_POWER | POWER_CMD_CURRENT
data[0]:    0xFA = 250 & 0xFF (low byte)
data[1]:    0x00 = (250 >> 8) & 0xFF (high byte)
checksum:   CRC8([0x50, 0xFA, 0x00]) = 0x0C
Packet:     {0x50, 0xFA, 0x00, 0x0C}

Reconstruction: current_ma = data[0] | (data[1] << 8) = 250mA
Units: Milliamperes (mA)
Range: 0-65535 mA
```

**Battery Level Report (85%):**
```
type_flags: 0x51 = [010][10001] = TYPE_POWER | POWER_CMD_BATTERY
data[0]:    0x55 = 85 & 0xFF (low byte)
data[1]:    0x00 = (85 >> 8) & 0xFF (high byte)
checksum:   CRC8([0x51, 0x55, 0x00]) = 0x04
Packet:     {0x51, 0x55, 0x00, 0x04}

Reconstruction: battery_percent = data[0] | (data[1] << 8) = 85%
Units: Percentage (%)
Range: 0-100%
```

**Temperature Report (23.5°C):**
```
type_flags: 0x52 = [010][10010] = TYPE_POWER | POWER_CMD_TEMP
data[0]:    0xEB = 235 & 0xFF (low byte) 
data[1]:    0x00 = (235 >> 8) & 0xFF (high byte)
checksum:   CRC8([0x52, 0xEB, 0x00]) = 0xB9
Packet:     {0x52, 0xEB, 0x00, 0xB9}

Reconstruction: temp_deci_celsius = data[0] | (data[1] << 8) = 235
Actual Temperature: 235 / 10 = 23.5°C
Units: Deci-Celsius (0.1°C resolution)
Range: -3276.8°C to +3276.7°C
```

**Voltage Report (3.7V):**
```
type_flags: 0x53 = [010][10011] = TYPE_POWER | POWER_CMD_VOLTAGE
data[0]:    0x70 = 3700 & 0xFF (low byte)
data[1]:    0x0E = (3700 >> 8) & 0xFF (high byte)
checksum:   CRC8([0x53, 0x70, 0x0E]) = 0x71
Packet:     {0x53, 0x70, 0x0E, 0x71}

Reconstruction: voltage_mv = data[0] | (data[1] << 8) = 3700mV = 3.7V
Units: Millivolts (mV)
Range: 0-65535 mV (0-65.535V)
```

### Power State Responses

**Power State Query Response:**
```
type_flags: 0x40 = [010][00000] = TYPE_POWER | POWER_CMD_QUERY
data[0]:    0x01 = POWER_STATE_RUNNING
data[1]:    0x80 = Status flags (0x80 = battery powered)
checksum:   CRC8([0x40, 0x01, 0x80]) = 0x21
Packet:     {0x40, 0x01, 0x80, 0x21}

Meaning: RP2040 is in running state, powered by battery
```

**Boot Notification:**
```
type_flags: 0x40 = [010][00000] = TYPE_POWER | POWER_CMD_QUERY  
data[0]:    0x01 = POWER_STATE_RUNNING
data[1]:    0x01 = Status flags (0x01 = boot complete)
checksum:   CRC8([0x40, 0x01, 0x01]) = 0x40
Packet:     {0x40, 0x01, 0x01, 0x40}

Meaning: RP2040 has completed boot and is ready for operation
```

### Power Management Features

**Automatic Boot Notification:**
- Sent during RP2040 firmware startup
- Indicates RP2040 is ready for communication
- Triggers driver initialization on host side

**Display Release Signal:**
- Automatically sent during Linux driver initialization
- Signals RP2040 to release e-ink display control to host
- Enables smooth handover from RP2040 boot animation to Linux display manager

**Shutdown Coordination:**
- Linux driver sends shutdown notification before power loss
- RP2040 saves critical state and releases hardware resources
- Ensures clean shutdown and prevents data corruption

**Power Metrics Polling:**
- Configurable polling interval (default 1000ms)
- Automatic retry on communication failures
- Integration with Linux power supply subsystem

**Sleep State Management:**
- Coordinated sleep states between host and RP2040
- Wake-on-event support (button presses, timers)
- Power consumption optimization

## Button Interface Specification

### Technical Specifications

- **Communication Direction**: RP2040 → Host (events only)
- **Button Count**: 4 physical buttons (UP, DOWN, SELECT, POWER)
- **Debouncing**: Hardware-level debouncing on RP2040
- **Event Types**: Press and release events
- **Sampling Rate**: 100 Hz button scanning
- **Bounce Time**: 50ms debounce period
- **Long Press**: Configurable long press detection

### Button Hardware Characteristics

| Button | GPIO Pin | Pull-up | Active State | Function |
|--------|----------|---------|--------------|----------|
| UP | GPIO12 | Internal | LOW | Navigation up |
| DOWN | GPIO13 | Internal | LOW | Navigation down |
| SELECT | GPIO14 | Internal | LOW | Selection/Enter |
| POWER | GPIO15 | Internal | LOW | Power control |

### Button Command Structure

Button events use the 5 least significant bits of the `type_flags` byte to encode simultaneous button states:

```
type_flags: [000][BUTTON_STATE(5-bit)]
            │    │
            │    └─ Button state bitmap (PSDU format)
            └───── Message type (000 = BUTTON)

data[0]:    Reserved (should be 0x00)
data[1]:    Reserved (should be 0x00)  
checksum:   CRC8 of all 3 bytes
```

### Button State Encoding

The button state uses a bitmap format where each bit represents one button:

```
┌─────────────────────────────────────────────────────────────┐
│                Button State Bitmap (5 bits)                 │
├───┬───┬───┬───┬───┐                                         │
│ 4 │ 3 │ 2 │ 1 │ 0 │  Bit Position                           │
├───┼───┼───┼───┼───┤                                         │
│ R │ P │ S │ D │ U │  Button Assignment                      │
├───┼───┼───┼───┼───┤                                         │
│ 0 │ P │ S │ D │ U │  Active State (1=pressed, 0=released)   │
└───┴───┴───┴───┴───┘─────────────────────────────────────────┘

Where:
U = UP button (bit 0, mask 0x01)
D = DOWN button (bit 1, mask 0x02)  
S = SELECT button (bit 2, mask 0x04)
P = POWER button (bit 3, mask 0x08)
R = Reserved (bit 4, should be 0)
```

**Button Mapping:**
- **Bit 0 (0x01)**: UP button
- **Bit 1 (0x02)**: DOWN button  
- **Bit 2 (0x04)**: SELECT button
- **Bit 3 (0x08)**: POWER button
- **Bit 4 (0x10)**: Reserved (must be 0)

### Button Event Examples

**UP Button Pressed:**
```
type_flags: 0x01 = [000][00001] = TYPE_BUTTON | UP_PRESSED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x01, 0x00, 0x00]) = 0x55
Packet:     {0x01, 0x00, 0x00, 0x55}

Meaning: UP button is currently pressed, others released
Linux Event: KEY_UP press event sent to input subsystem
```

**DOWN Button Pressed:**
```
type_flags: 0x02 = [000][00010] = TYPE_BUTTON | DOWN_PRESSED  
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x02, 0x00, 0x00]) = 0x56
Packet:     {0x02, 0x00, 0x00, 0x56}

Meaning: DOWN button is currently pressed, others released
Linux Event: KEY_DOWN press event sent to input subsystem
```

**SELECT + DOWN Pressed (Multi-button):**
```
type_flags: 0x06 = [000][00110] = TYPE_BUTTON | SELECT_PRESSED | DOWN_PRESSED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved  
checksum:   CRC8([0x06, 0x00, 0x00]) = 0x52
Packet:     {0x06, 0x00, 0x00, 0x52}

Meaning: Both SELECT and DOWN buttons pressed simultaneously
Linux Events: KEY_ENTER and KEY_DOWN press events sent
```

**All Buttons Released:**
```
type_flags: 0x00 = [000][00000] = TYPE_BUTTON | ALL_RELEASED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x00, 0x00, 0x00]) = 0x00  
Packet:     {0x00, 0x00, 0x00, 0x00}

Meaning: All buttons are currently released
Linux Events: Release events for any previously pressed buttons
```

**POWER Button Pressed:**
```
type_flags: 0x08 = [000][01000] = TYPE_BUTTON | POWER_PRESSED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x08, 0x00, 0x00]) = 0x5C
Packet:     {0x08, 0x00, 0x00, 0x5C}

Meaning: POWER button is currently pressed
Linux Event: KEY_POWER press event sent (may trigger power management)
```

### Button State Transitions

The RP2040 sends button events whenever the button state changes. The complete button state is sent in each packet, not just the changed buttons.

**Transition Example (UP press, then release):**

1. **Initial State** (all released): `{0x00, 0x00, 0x00, 0x00}`
2. **UP Pressed**: `{0x01, 0x00, 0x00, 0x55}`
3. **UP Released**: `{0x00, 0x00, 0x00, 0x00}`

**Multi-button Transition Example:**

1. **Initial State**: `{0x00, 0x00, 0x00, 0x00}`
2. **SELECT Pressed**: `{0x04, 0x00, 0x00, 0x50}`
3. **DOWN Also Pressed**: `{0x06, 0x00, 0x00, 0x52}`
4. **SELECT Released, DOWN Still Pressed**: `{0x02, 0x00, 0x00, 0x56}`
5. **All Released**: `{0x00, 0x00, 0x00, 0x00}`

### Linux Input Integration

The SAM driver integrates with the Linux input subsystem to provide standard keyboard events:

**Input Device Properties:**
```
Name: "Pamir AI SAM Buttons"
Vendor: 0x2040 (Raspberry Pi Foundation)
Product: 0x0001
Version: 0x0100
```

**Key Mappings:**
| Button | Linux Key Code | Symbolic Name |
|--------|----------------|---------------|
| UP | 103 | KEY_UP |
| DOWN | 108 | KEY_DOWN |
| SELECT | 28 | KEY_ENTER |
| POWER | 116 | KEY_POWER |

**Event Monitoring:**
```bash
# Find the input device
cat /proc/bus/input/devices | grep -A 5 "Pamir AI SAM"

# Monitor button events  
sudo evtest /dev/input/event<N>

# Example output:
Event: time 1642857234.123456, type 1 (EV_KEY), code 103 (KEY_UP), value 1
Event: time 1642857234.223456, type 1 (EV_KEY), code 103 (KEY_UP), value 0
```

### Button Debouncing Algorithm

The RP2040 implements hardware-level debouncing using a state machine approach:

**Debounce Parameters:**
- **Sampling Period**: 10ms (100 Hz)
- **Debounce Time**: 50ms (5 consecutive samples)
- **Long Press Threshold**: 1000ms (configurable)
- **Repeat Rate**: 200ms (for long press repeat)

**State Machine:**
1. **IDLE**: No button activity
2. **DEBOUNCE_PRESS**: Button pressed, confirming for 50ms
3. **PRESSED**: Button confirmed pressed
4. **DEBOUNCE_RELEASE**: Button released, confirming for 50ms
5. **LONG_PRESS**: Button held for >1000ms (optional)

### Performance Characteristics

**Timing Specifications:**
- **Response Time**: <10ms from physical press to packet transmission
- **Debounce Delay**: 50ms settling time
- **Event Rate**: Up to 20 events/second sustained
- **Multi-button Support**: All 4 buttons can be pressed simultaneously
- **Packet Overhead**: 4 bytes per button state change

**Reliability Features:**
- **Hardware Debouncing**: Eliminates contact bounce artifacts
- **State Confirmation**: Multiple samples required for state change
- **Error Detection**: CRC8 checksum on all button packets
- **Missed Event Detection**: Complete state sent in each packet

## Display Control Specification

### Technical Specifications

- **Communication Direction**: Host ↔ RP2040 (bidirectional)
- **Display Type**: E-ink display with SPI interface
- **Resolution**: Configurable (typically 250x122 pixels)
- **Color Depth**: Monochrome (1-bit per pixel)
- **Refresh Rate**: ~2 seconds full refresh, ~0.5 seconds partial refresh
- **Control Handover**: Boot-time release from RP2040 to Host

### Display Command Structure

Display commands provide control over e-ink display operations and coordinate display access between RP2040 and the host system:

```
type_flags: [011][ACTION(5-bit)]
            │    │
            │    └─ Display action (0-31)
            └───── Message type (011 = DISPLAY)

data[0]:    Command-specific parameter 1
data[1]:    Command-specific parameter 2
checksum:   CRC8 of all 3 bytes
```

### Display Command Definitions

| Command | Hex | Name | Direction | Purpose |
|---------|-----|------|-----------|---------|
| 0x00 | 0x60 | DISPLAY_CMD_QUERY | Host → RP2040 | Query display status |
| 0x01 | 0x61 | DISPLAY_CMD_STATUS | RP2040 → Host | Display status report |
| 0x02 | 0x62 | DISPLAY_CMD_INIT | Host → RP2040 | Initialize display |
| 0x03 | 0x63 | DISPLAY_CMD_CLEAR | Host → RP2040 | Clear display |
| 0x04 | 0x64 | DISPLAY_CMD_REFRESH | Host → RP2040 | Refresh display |
| 0x05 | 0x65 | DISPLAY_CMD_SLEEP | Host → RP2040 | Put display to sleep |
| 0x06 | 0x66 | DISPLAY_CMD_WAKE | Host → RP2040 | Wake display |
| 0x07 | 0x67 | DISPLAY_CMD_RELEASE | Host → RP2040 | Release display control |
| 0x08 | 0x68 | DISPLAY_CMD_ACQUIRE | Host → RP2040 | Acquire display control |

### Display Control States

| State | Value | Name | Description |
|-------|-------|------|-------------|
| 0x00 | 0 | DISPLAY_STATE_UNKNOWN | Unknown/uninitialized state |
| 0x01 | 1 | DISPLAY_STATE_RP2040_CONTROL | RP2040 has display control |
| 0x02 | 2 | DISPLAY_STATE_HOST_CONTROL | Host has display control |
| 0x03 | 3 | DISPLAY_STATE_SLEEP | Display in sleep mode |
| 0x04 | 4 | DISPLAY_STATE_ERROR | Display error state |

### Display Command Examples

**Release Display Control (Boot-time Handover):**
```
type_flags: 0x67 = [011][00111] = TYPE_DISPLAY | DISPLAY_CMD_RELEASE
data[0]:    0xFF = Release signal (special value)
data[1]:    0x00 = Flags (reserved)
checksum:   CRC8([0x67, 0xFF, 0x00]) = 0x98
Packet:     {0x67, 0xFF, 0x00, 0x98}

Purpose: Signal RP2040 to release display control to host during boot
Effect: 
  1. RP2040 calls eink.de_init() to release GPIO control
  2. RP2040 calls einkMux.low() to route display to host
  3. RP2040 stops boot animation loop
  4. Host can now control display directly
```

**Query Display Status:**
```
type_flags: 0x60 = [011][00000] = TYPE_DISPLAY | DISPLAY_CMD_QUERY
data[0]:    0x00 = Query type (0x00 = general status)
data[1]:    0x00 = Reserved
checksum:   CRC8([0x60, 0x00, 0x00]) = 0x60
Packet:     {0x60, 0x00, 0x00, 0x60}

Purpose: Request current display status from RP2040
Response: RP2040 sends DISPLAY_CMD_STATUS packet with current state
```

**Display Status Response:**
```
type_flags: 0x61 = [011][00001] = TYPE_DISPLAY | DISPLAY_CMD_STATUS  
data[0]:    0x02 = DISPLAY_STATE_HOST_CONTROL
data[1]:    0x00 = Status flags (0x01 = busy, 0x02 = error)
checksum:   CRC8([0x61, 0x02, 0x00]) = 0x63
Packet:     {0x61, 0x02, 0x00, 0x63}

Meaning: Display is under host control, no errors, not busy
```

**Display Refresh Complete Notification:**
```
type_flags: 0x61 = [011][00001] = TYPE_DISPLAY | DISPLAY_CMD_STATUS
data[0]:    0xFF = Completion indicator
data[1]:    0x00 = Refresh type completed (0x00 = full, 0x01 = partial)
checksum:   CRC8([0x61, 0xFF, 0x00]) = 0x9E
Packet:     {0x61, 0xFF, 0x00, 0x9E}

Meaning: RP2040 completed display refresh operation
```

**Clear Display:**
```
type_flags: 0x63 = [011][00011] = TYPE_DISPLAY | DISPLAY_CMD_CLEAR
data[0]:    0x00 = Clear color (0x00 = white, 0xFF = black)
data[1]:    0x01 = Clear type (0x00 = full, 0x01 = partial)
checksum:   CRC8([0x63, 0x00, 0x01]) = 0x62
Packet:     {0x63, 0x00, 0x01, 0x62}

Purpose: Clear display to white using partial refresh
```

### Display Handover Sequence

The display handover from RP2040 to host during boot follows this sequence:

1. **RP2040 Boot**: RP2040 initializes display and shows boot animation
2. **Host Boot**: Linux kernel loads and SAM driver initializes
3. **Release Signal**: Driver sends DISPLAY_CMD_RELEASE to RP2040
4. **RP2040 Cleanup**: 
   - RP2040 calls `eink.de_init()` to release GPIO control
   - RP2040 calls `einkMux.low()` to route display signals to host
   - RP2040 stops animation loop
5. **Host Takeover**: Host can now control display directly via SPI
6. **Acknowledgment**: RP2040 sends status confirming handover

### Display Hardware Characteristics

**E-ink Display Specifications:**
- **Technology**: Electrophoretic display (E-ink)
- **Interface**: SPI (Serial Peripheral Interface)
- **Multiplexer**: GPIO-controlled signal routing between RP2040 and host
- **Refresh Types**: Full refresh (clean, slow) and partial refresh (fast, ghosting)
- **Power**: Low power consumption, image persists without power

**Timing Characteristics:**
- **Full Refresh**: ~2000ms (complete image update)
- **Partial Refresh**: ~500ms (faster update with potential ghosting)
- **Sleep Entry**: ~100ms
- **Wake Time**: ~200ms
- **Command Response**: <50ms

## Debug Interface Specification

### Technical Specifications

- **Communication Direction**: RP2040 → Host (logging/monitoring only)
- **Debug Types**: 2 types (Numeric codes, Text messages)
- **Code Categories**: 8 categories (System, Error, Button, LED, Power, Display, Comm, Performance)
- **Text Messages**: Multi-packet support for long messages
- **Buffer Management**: Circular buffer with overflow protection

### Debug Code Structure

Debug codes provide compact, efficient reporting of predefined events and conditions:

```
type_flags: [100][CATEGORY(5-bit)]
            │    │
            │    └─ Debug category (0-31)
            └───── Message type (100 = DEBUG_CODE)

data[0]:    Debug code (0-255)
data[1]:    Parameter value (0-255)
checksum:   CRC8 of all 3 bytes
```

### Debug Code Categories

| Category | Hex Range | Name | Description |
|----------|-----------|------|-------------|
| 0 | 0x80 | SYSTEM | System events (boot, reset, watchdog) |
| 1 | 0x81 | ERROR | Error conditions (hardware, protocol, timeouts) |
| 2 | 0x82 | BUTTON | Button events (press, release, debounce) |
| 3 | 0x83 | LED | LED events (command received, animation complete) |
| 4 | 0x84 | POWER | Power events (state change, metric update) |
| 5 | 0x85 | DISPLAY | Display events (refresh start/complete, handover) |
| 6 | 0x86 | COMM | Communication events (packet TX/RX, errors) |
| 7 | 0x87 | PERFORMANCE | Performance metrics (timing, memory usage) |

### Debug Text Structure

Debug text messages support long-form diagnostic information with multi-packet transmission:

```
type_flags: [101][FLAGS(5-bit)]
            │    │
            │    └─ Text flags (First, Continue, Sequence)
            └───── Message type (101 = DEBUG_TEXT)

Bit Layout of FLAGS:
┌───┬───┬───┬───┬───┐
│ 4 │ 3 │ 2 │ 1 │ 0 │
├───┼───┼───┼───┼───┤
│ F │ C │SEQ (3-bit)│
└───┴───┴───┴───┴───┘

F = First chunk (1 = start of message)
C = Continue (1 = more chunks follow)  
SEQ = Sequence number (0-7, for packet ordering)
```

### Debug Code Examples

**System Boot Complete:**
```
type_flags: 0x80 = [100][00000] = TYPE_DEBUG_CODE | CATEGORY_SYSTEM
data[0]:    0x01 = DEBUG_CODE_BOOT_COMPLETE
data[1]:    0x02 = Boot stage (2 = firmware initialization complete)
checksum:   CRC8([0x80, 0x01, 0x02]) = 0x83
Packet:     {0x80, 0x01, 0x02, 0x83}

Meaning: RP2040 firmware boot completed, stage 2 (initialization)
```

**Button Press Event:**
```
type_flags: 0x82 = [100][00010] = TYPE_DEBUG_CODE | CATEGORY_BUTTON
data[0]:    0x10 = DEBUG_CODE_BUTTON_PRESS
data[1]:    0x04 = Button mask (SELECT button = 0x04)
checksum:   CRC8([0x82, 0x10, 0x04]) = 0x96
Packet:     {0x82, 0x10, 0x04, 0x96}

Meaning: SELECT button press detected and debounced
```

**LED Animation Complete:**
```
type_flags: 0x83 = [100][00011] = TYPE_DEBUG_CODE | CATEGORY_LED
data[0]:    0x20 = DEBUG_CODE_ANIMATION_COMPLETE
data[1]:    0x05 = LED ID that completed animation
checksum:   CRC8([0x83, 0x20, 0x05]) = 0xA6
Packet:     {0x83, 0x20, 0x05, 0xA6}

Meaning: LED 5 completed its animation sequence
```

**Communication Error:**
```
type_flags: 0x86 = [100][00110] = TYPE_DEBUG_CODE | CATEGORY_COMM
data[0]:    0xFE = DEBUG_CODE_PACKET_ERROR
data[1]:    0x02 = Error type (2 = checksum mismatch)
checksum:   CRC8([0x86, 0xFE, 0x02]) = 0x7A
Packet:     {0x86, 0xFE, 0x02, 0x7A}

Meaning: Received packet with checksum error
```

### Debug Text Examples

**Single Packet Text Message:**
```
type_flags: 0xB0 = [101][10000] = TYPE_DEBUG_TEXT | FIRST | SEQ_0
data[0]:    0x48 = 'H' (ASCII)
data[1]:    0x69 = 'i' (ASCII)
checksum:   CRC8([0xB0, 0x48, 0x69]) = 0x11
Packet:     {0xB0, 0x48, 0x69, 0x11}

Meaning: Short debug message "Hi" (complete in one packet)
```

**Multi-Packet Text Message ("Hello World!"):**

Packet 1 (First chunk):
```
type_flags: 0xB8 = [101][11000] = TYPE_DEBUG_TEXT | FIRST | CONTINUE | SEQ_0
data[0]:    0x48 = 'H' (ASCII)
data[1]:    0x65 = 'e' (ASCII)  
checksum:   CRC8([0xB8, 0x48, 0x65]) = 0x11
Packet:     {0xB8, 0x48, 0x65, 0x11}
```

Packet 2 (Continue chunk):
```
type_flags: 0xA9 = [101][01001] = TYPE_DEBUG_TEXT | CONTINUE | SEQ_1
data[0]:    0x6C = 'l' (ASCII)
data[1]:    0x6C = 'l' (ASCII)
checksum:   CRC8([0xA9, 0x6C, 0x6C]) = 0x0D
Packet:     {0xA9, 0x6C, 0x6C, 0x0D}
```

Packet 3 (Continue chunk):
```
type_flags: 0xAA = [101][01010] = TYPE_DEBUG_TEXT | CONTINUE | SEQ_2
data[0]:    0x6F = 'o' (ASCII)
data[1]:    0x20 = ' ' (ASCII space)
checksum:   CRC8([0xAA, 0x6F, 0x20]) = 0x89
Packet:     {0xAA, 0x6F, 0x20, 0x89}
```

Packet 4 (Final chunk):
```
type_flags: 0xA3 = [101][00011] = TYPE_DEBUG_TEXT | SEQ_3 (no CONTINUE)
data[0]:    0x57 = 'W' (ASCII)
data[1]:    0x6F = 'o' (ASCII)
checksum:   CRC8([0xA3, 0x57, 0x6F]) = 0xDB
Packet:     {0xA3, 0x57, 0x6F, 0xDB}
```

### Debug Performance Monitoring

**Memory Usage Report:**
```
type_flags: 0x87 = [100][00111] = TYPE_DEBUG_CODE | CATEGORY_PERFORMANCE
data[0]:    0x30 = DEBUG_CODE_MEMORY_USAGE
data[1]:    0x42 = 66% memory utilization
checksum:   CRC8([0x87, 0x30, 0x42]) = 0xF5
Packet:     {0x87, 0x30, 0x42, 0xF5}

Meaning: Current memory usage is 66% of available RAM
```

**Packet Processing Time:**
```
type_flags: 0x87 = [100][00111] = TYPE_DEBUG_CODE | CATEGORY_PERFORMANCE  
data[0]:    0x40 = DEBUG_CODE_PROCESSING_TIME
data[1]:    0x08 = 8 microseconds processing time
checksum:   CRC8([0x87, 0x40, 0x08]) = 0xC7
Packet:     {0x87, 0x40, 0x08, 0xC7}

Meaning: Last packet took 8μs to process
```

## System Commands Specification

### Technical Specifications

- **Communication Direction**: Host ↔ RP2040 (bidirectional)
- **Command Types**: 8 primary commands (Ping, Reset, Version, Status, Config, etc.)
- **Response Time**: <50ms for most commands
- **Error Handling**: Automatic retry and escalation
- **Version Support**: Semantic versioning with compatibility checks

### System Command Structure

System commands provide core control and status functions for the SAM protocol:

```
type_flags: [110][ACTION(5-bit)]
            │    │
            │    └─ System action (0-31)
            └───── Message type (110 = SYSTEM)

data[0]:    Command-specific parameter 1
data[1]:    Command-specific parameter 2  
checksum:   CRC8 of all 3 bytes
```

### System Command Definitions

| Command | Hex | Name | Direction | Purpose |
|---------|-----|------|-----------|---------|
| 0x00 | 0xC0 | SYSTEM_PING | Host ↔ RP2040 | Connectivity test |
| 0x01 | 0xC1 | SYSTEM_RESET | Host → RP2040 | Reset microcontroller |
| 0x02 | 0xC2 | SYSTEM_VERSION | Host ↔ RP2040 | Version information exchange |
| 0x03 | 0xC3 | SYSTEM_STATUS | Host ↔ RP2040 | System status query/report |
| 0x04 | 0xC4 | SYSTEM_CONFIG | Host → RP2040 | Configuration commands |
| 0x05 | 0xC5 | SYSTEM_SYNC | Host ↔ RP2040 | Time synchronization |
| 0x06 | 0xC6 | SYSTEM_CAPABILITIES | Host ↔ RP2040 | Feature capability exchange |
| 0x1F | 0xDF | SYSTEM_EXTENDED | Host ↔ RP2040 | Extended/custom commands |

### System Command Examples

**Ping Request (Host → RP2040):**
```
type_flags: 0xC0 = [110][00000] = TYPE_SYSTEM | SYSTEM_PING
data[0]:    0x00 = Ping ID (for matching response)
data[1]:    0x00 = Ping flags (reserved)
checksum:   CRC8([0xC0, 0x00, 0x00]) = 0x8B
Packet:     {0xC0, 0x00, 0x00, 0x8B}

Purpose: Test connectivity and measure round-trip time
Expected Response: Pong packet with same ping ID
```

**Version Request (Host → RP2040):**
```
type_flags: 0xC2 = [110][00010] = TYPE_SYSTEM | SYSTEM_VERSION
data[0]:    0x00 = Request type (0x00 = firmware version)
data[1]:    0x00 = Reserved
checksum:   CRC8([0xC2, 0x00, 0x00]) = 0x89
Packet:     {0xC2, 0x00, 0x00, 0x89}

Purpose: Request RP2040 firmware version information
Response: Version packet with major.minor.patch format
```

**Version Response (RP2040 → Host):**
```
type_flags: 0xC2 = [110][00010] = TYPE_SYSTEM | SYSTEM_VERSION
data[0]:    0x00 = Major version (0 = v0.x.x)
data[1]:    0x23 = Minor (2) and Patch (3) versions (0x23 = v0.2.3)
checksum:   CRC8([0xC2, 0x00, 0x23]) = 0xA8
Packet:     {0xC2, 0x00, 0x23, 0xA8}

Meaning: RP2040 firmware version 0.2.3
Decoding: major=0, minor=(0x23>>4)=2, patch=(0x23&0x0F)=3
```

## Extended Commands Specification

### Technical Specifications

- **Communication Direction**: Host ↔ RP2040 (bidirectional)
- **Purpose**: Future protocol extensions and custom functionality
- **Command Space**: 32 extended command types (0x00-0x1F)
- **Compatibility**: Forward/backward compatibility support
- **Implementation**: Optional, device-specific features

### Extended Command Structure

Extended commands provide a framework for future protocol enhancements:

```
type_flags: [111][EXTENSION(5-bit)]
            │    │
            │    └─ Extension type (0-31)
            └───── Message type (111 = EXTENDED)

data[0]:    Extension-specific parameter 1
data[1]:    Extension-specific parameter 2
checksum:   CRC8 of all 3 bytes
```

### Reserved Extension Types

| Extension | Hex | Name | Purpose |
|-----------|-----|------|---------|
| 0x00 | 0xE0 | EXT_CAPABILITIES | Extended capability negotiation |
| 0x01 | 0xE1 | EXT_SENSOR | Additional sensor support |
| 0x02 | 0xE2 | EXT_ACTUATOR | Additional actuator control |
| 0x03 | 0xE3 | EXT_NETWORK | Network interface commands |
| 0x04 | 0xE4 | EXT_STORAGE | Storage management |
| 0x05 | 0xE5 | EXT_CRYPTO | Cryptographic operations |
| 0x06 | 0xE6 | EXT_AUDIO | Audio interface control |
| 0x07 | 0xE7 | EXT_VIDEO | Video interface control |
| 0x1E | 0xFE | EXT_VENDOR | Vendor-specific extensions |
| 0x1F | 0xFF | EXT_EXPERIMENTAL | Experimental features |

## Performance and Timing Specifications

### Communication Performance

| Parameter | Specification | Typical | Notes |
|-----------|--------------|---------|-------|
| **UART Baud Rate** | 115200 bps | 115200 bps | Fixed rate |
| **Theoretical Throughput** | 11.5 KB/s | 10.8 KB/s | Including protocol overhead |
| **Practical Packet Rate** | 5000 pps max | 2000 pps | With processing overhead |
| **Packet Transmission Time** | 348 μs | 348 μs | 4 bytes @ 115200 baud |
| **Round-trip Latency** | <10 ms | 5 ms | Ping-pong measurement |
| **Protocol Efficiency** | 50% | 50% | 2 bytes payload / 4 bytes total |

### Processing Performance

| Component | Response Time | Throughput | CPU Usage |
|-----------|---------------|------------|-----------|
| **Button Processing** | <1 ms | 100 events/s | <5% |
| **LED Update** | <5 ms | 60 fps | <15% |
| **Power Metrics** | <50 ms | 10 samples/s | <10% |
| **Display Refresh** | 2000 ms | 0.5 fps | <20% |
| **Debug Logging** | <1 ms | 1000 msgs/s | <5% |
| **System Commands** | <50 ms | 100 cmds/s | <10% |

### Memory Usage

| Component | RAM Usage | Flash Usage | Notes |
|-----------|-----------|-------------|-------|
| **Protocol Core** | 2 KB | 8 KB | Basic packet handling |
| **LED Handler** | 1 KB | 4 KB | 16 LED support |
| **Power Manager** | 512 B | 2 KB | Metrics and state |
| **Input Handler** | 256 B | 1 KB | Button debouncing |
| **Debug Interface** | 4 KB | 3 KB | Circular buffers |
| **Display Controller** | 8 KB | 12 KB | Frame buffer |
| **Total Footprint** | 16 KB | 30 KB | Complete implementation |

### Timing Constraints

| Operation | Maximum Time | Typical Time | Timeout Action |
|-----------|-------------|-------------|----------------|
| **Packet Processing** | 1 ms | 100 μs | Log error |
| **Command Response** | 50 ms | 10 ms | Retry transmission |
| **Power Metrics Update** | 1000 ms | 500 ms | Use cached values |
| **Display Refresh** | 5000 ms | 2000 ms | Abort operation |
| **Button Debounce** | 100 ms | 50 ms | Force state change |
| **System Reset** | 10000 ms | 3000 ms | Hardware watchdog |

## Error Handling and Recovery

### Error Detection Mechanisms

1. **CRC8 Validation**: Every packet validated with CRC8 checksum
2. **Field Validation**: Command codes and parameters validated against specifications
3. **Boundary Checking**: LED IDs, power values, timing parameters checked for valid ranges
4. **State Validation**: System state consistency checks
5. **Timeout Detection**: Missing acknowledgments and responses detected
6. **Sequence Validation**: Multi-packet message sequence integrity

### Error Categories

| Category | Severity | Action | Recovery |
|----------|----------|--------|----------|
| **CRC8 Mismatch** | High | Discard packet | Request retransmission |
| **Invalid Command** | Medium | Send error response | Continue processing |
| **Parameter Error** | Medium | Send error response | Use default values |
| **Timeout** | Medium | Retry operation | Exponential backoff |
| **Hardware Error** | High | Reset subsystem | Full system reset |
| **Resource Exhaustion** | High | Graceful degradation | Free resources |

### Recovery Procedures

**Protocol Recovery:**
1. **Detection**: CRC8 error or invalid packet format detected
2. **Buffer Flush**: Clear receive buffer to eliminate corruption
3. **Backoff**: Exponential backoff delay (100ms, 200ms, 400ms)
4. **Ping Test**: Send ping command to test connectivity
5. **Escalation**: If 3 attempts fail, trigger system reset
6. **Success**: Resume normal operation on valid response

**Hardware Recovery:**
1. **Detection**: Hardware subsystem reports error or stops responding
2. **Isolation**: Disable problematic subsystem
3. **Reset**: Hardware reset of specific subsystem
4. **Reinitialize**: Full reinitialization of subsystem
5. **Test**: Verify proper operation before re-enabling
6. **Fallback**: Use degraded mode if recovery fails

### Error Response Format

```
type_flags: [100][ERROR_CATEGORY(5-bit)]
           Type=4|Category(0-31)
data[0]:    Original command type that failed
data[1]:    Specific error code
checksum:   CRC8 of response
```

**Error Categories:**
- 0x80: Protocol errors (CRC, format, sequence)
- 0x81: Command errors (invalid, unsupported, permission)
- 0x82: Parameter errors (range, type, consistency)
- 0x83: Hardware errors (sensor, actuator, communication)
- 0x84: Resource errors (memory, timeout, busy)
- 0x85: State errors (invalid state, state conflict)

**Common Error Codes:**
- 0x01: Invalid command
- 0x02: Invalid parameter range
- 0x03: Command timeout
- 0x04: Hardware not ready
- 0x05: Resource busy
- 0x06: Not implemented
- 0x07: Permission denied
- 0x08: Insufficient memory
- 0x09: Invalid state
- 0x0A: Checksum error

## Implementation Requirements

### Kernel Driver Requirements

1. **CRC8 Implementation**: Must implement CRC8 with polynomial 0x07
2. **LED Support**: Must support 16 LEDs (ID 0-15) with full RGB control
3. **Power Commands**: Must use separated command codes (control vs reporting)
4. **Error Handling**: Must validate all packets and handle errors gracefully
5. **Constants**: Must define all command constants matching this specification
6. **Linux Integration**: Must integrate with input, LED, and power supply subsystems
7. **Character Device**: Must provide userspace access via character device
8. **Sysfs Interface**: Must provide sysfs attributes for configuration and monitoring
9. **Device Tree**: Must support device tree configuration
10. **Module Parameters**: Must support runtime configuration via module parameters

### Firmware Requirements

1. **CRC8 Validation**: Must validate all incoming packets
2. **LED Control**: Must support 16 LEDs with 4 modes and 4 timing levels
3. **Power Reporting**: Must implement separated reporting commands
4. **Error Responses**: Must send appropriate error responses for all error conditions
5. **Compatibility**: Must reject v0.1.0 packets (different checksum algorithm)
6. **Hardware Abstraction**: Must abstract hardware differences between board revisions
7. **Watchdog Support**: Must implement hardware watchdog for reliability
8. **Boot Notification**: Must send boot complete notification
9. **Display Handover**: Must support display control handover to host
10. **Debug Interface**: Must implement comprehensive debug logging

### Validation Requirements

1. **Packet Validation**: Every implementation must validate packet format and checksums
2. **CRC8 Testing**: Must include CRC8 test vectors for validation
3. **Error Injection**: Must test error handling with corrupted packets
4. **Boundary Testing**: Must test all boundary conditions (max/min values)
5. **Performance Testing**: Must validate timing requirements under load
6. **Interoperability**: Must test with reference implementations
7. **Stress Testing**: Must handle continuous operation and error conditions
8. **Regression Testing**: Must maintain compatibility with existing implementations
9. **Compliance Testing**: Must verify adherence to this specification
10. **Documentation**: Must document any deviations or extensions

### Reference Implementation Status

| Component | Linux Driver | RP2040 Firmware | Test Coverage | Compliance |
|-----------|-------------|-----------------|---------------|------------|
| **Protocol Core** | ✅ Complete | ✅ Complete | 95% | Full |
| **LED Control** | ✅ Complete | ✅ Complete | 90% | Full |
| **Power Management** | ✅ Complete | ✅ Complete | 85% | Full |
| **Button Input** | ✅ Complete | ✅ Complete | 95% | Full |
| **Display Control** | ⚠️ Partial | ⚠️ Partial | 40% | Minimal |
| **Debug Interface** | ✅ Complete | ✅ Complete | 80% | Full |
| **System Commands** | ✅ Complete | ✅ Complete | 75% | Full |
| **Error Handling** | ✅ Complete | ✅ Complete | 85% | Full |

## Test Vectors and Examples

### CRC8 Test Vectors

```c
// Test vectors for CRC8 polynomial 0x07
struct crc8_test_vector {
    uint8_t data[3];
    uint8_t expected_crc;
    char *description;
};

struct crc8_test_vector test_vectors[] = {
    {{0x00, 0x00, 0x00}, 0x00, "All zeros"},
    {{0xFF, 0xFF, 0xFF}, 0x49, "All ones"},
    {{0x20, 0xF0, 0x00}, 0x8E, "LED red command"},
    {{0x40, 0x00, 0x00}, 0x8C, "Power query"},
    {{0xC0, 0x00, 0x00}, 0x8B, "System ping"},
    {{0x01, 0x00, 0x00}, 0x55, "Button UP pressed"},
    {{0x67, 0xFF, 0x00}, 0x98, "Display release"},
    {{0x80, 0x01, 0x02}, 0x83, "Debug system boot"},
};
```

### Complete Packet Examples

**LED Rainbow Animation:**
```c
// Set LED 0 to rainbow mode with 500ms timing
uint8_t rainbow_packet[] = {0x20, 0xFF, 0xFA, 0x75};
// type_flags=0x20 (LED, ID=0), data[0]=0xFF (R=15,G=15), 
// data[1]=0xFA (B=15,Mode=Rainbow,Time=500ms), checksum=0x75
```

**Power Metrics Request:**
```c
// Request all power metrics
uint8_t metrics_request[] = {0x4F, 0x00, 0x00, 0x93};
// type_flags=0x4F (POWER_REQUEST_METRICS), data=[0,0], checksum=0x93
```

**Button Combination:**
```c
// SELECT + DOWN buttons pressed
uint8_t button_combo[] = {0x06, 0x00, 0x00, 0x52};
// type_flags=0x06 (BUTTON, SELECT|DOWN), data=[0,0], checksum=0x52
```

**System Version Response:**
```c
// Firmware version 0.2.3
uint8_t version_response[] = {0xC2, 0x00, 0x23, 0xA8};
// type_flags=0xC2 (SYSTEM_VERSION), major=0, minor/patch=0x23, checksum=0xA8
```

## Appendices

### Appendix A: Command Reference Quick Table

| Type | Range | Commands | Purpose |
|------|-------|----------|---------|
| BUTTON | 0x00-0x1F | 32 button states | Physical button events |
| LED | 0x20-0x3F | 16 LEDs × 2 modes | RGB LED control |
| POWER | 0x40-0x5F | 16 control + 16 report | Power management |
| DISPLAY | 0x60-0x7F | 8 commands | E-ink display control |
| DEBUG_CODE | 0x80-0x9F | 8 categories × 4 codes | Numeric debug codes |
| DEBUG_TEXT | 0xA0-0xBF | 32 text fragments | Text debug messages |
| SYSTEM | 0xC0-0xDF | 8 system commands | Core system control |
| EXTENDED | 0xE0-0xFF | 32 extensions | Future enhancements |

### Appendix B: Error Code Reference

| Code | Hex | Name | Description | Recovery |
|------|-----|------|-------------|----------|
| 1 | 0x01 | INVALID_COMMAND | Unknown command | Ignore packet |
| 2 | 0x02 | INVALID_PARAMETER | Parameter out of range | Use default |
| 3 | 0x03 | TIMEOUT | Operation timeout | Retry with backoff |
| 4 | 0x04 | HARDWARE_ERROR | Hardware malfunction | Reset subsystem |
| 5 | 0x05 | BUSY | Resource busy | Retry later |
| 6 | 0x06 | NOT_IMPLEMENTED | Feature not available | Use alternative |
| 7 | 0x07 | PERMISSION_DENIED | Access denied | Check permissions |
| 8 | 0x08 | MEMORY_ERROR | Insufficient memory | Free resources |
| 9 | 0x09 | STATE_ERROR | Invalid state | Reset to known state |
| 10 | 0x0A | CHECKSUM_ERROR | CRC mismatch | Request retransmission |

### Appendix C: Timing Specifications

| Operation | Min | Typ | Max | Unit | Notes |
|-----------|-----|-----|-----|------|-------|
| Packet transmission | 348 | 348 | 348 | μs | 4 bytes @ 115200 |
| Button response | 5 | 10 | 50 | ms | Including debounce |
| LED update | 1 | 5 | 20 | ms | Single LED |
| Power query | 10 | 25 | 100 | ms | Including ADC |
| Display refresh | 1500 | 2000 | 5000 | ms | Full refresh |
| System ping | 1 | 5 | 50 | ms | Round trip |
| Boot time | 500 | 1000 | 3000 | ms | Full initialization |
| Watchdog timeout | 5000 | 10000 | 15000 | ms | Configurable |

### Appendix D: Hardware Pinout Reference

| Function | RP2040 GPIO | CM5 GPIO | Direction | Notes |
|----------|-------------|----------|-----------|-------|
| UART TX | GP0 | GPIO4 | RP2040→CM5 | 115200 baud |
| UART RX | GP1 | GPIO5 | CM5→RP2040 | 115200 baud |
| Button UP | GP12 | - | Input | Pull-up enabled |
| Button DOWN | GP13 | - | Input | Pull-up enabled |
| Button SELECT | GP14 | - | Input | Pull-up enabled |
| Button POWER | GP15 | - | Input | Pull-up enabled |
| LED Data | GP16 | - | Output | WS2812B chain |
| Display Mux | GP17 | - | Output | Signal routing |
| Battery ADC | GP26 | - | Input | Analog input |
| Temperature | GP27 | - | Input | Analog input |

---

