# Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual

**Document Classification:** Technical Reference Manual

**Document Version:** 0.2.0

**Last Updated:** 2025-01-18

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

The Pamir AI Signal Aggregation Module (SAM) Technical Reference Manual provides the corrected and comprehensive technical specifications for the communication protocol used to interface between a Linux host system and the RP2040 microcontroller in Pamir AI CM5 devices.
### Scope

This document covers the complete technical specification of the SAM protocol, including:
- Corrected packet formats and field encodings
- Resolved command structures and bit field definitions
- Enhanced error detection and handling mechanisms
- Comprehensive packet examples with validation
- Clear implementation requirements for both kernel and firmware

## Architecture Overview

The SAM driver implements a modular architecture with clearly defined components:

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
|Buttons|  |16 LEDs|  |Power |  |Display |
+-------+  +-------+  +------+  +--------+
```

### Component Status

| Component | Implementation Status | Feature Completeness | API Stability |
| --- | --- | --- | --- |
| Protocol Core | Complete | 100% | Stable |
| LED Handler | Complete | 100% | Stable |
| Power Manager | Complete | 100% | Stable |
| Input Handler | Complete | 100% | Stable |
| Debug Interface | Complete | 100% | Stable |
| System Commands | Complete | 100% | Stable |
| Character Device | Complete | 100% | Stable |
| Display Controller | Minimal | 25% | Alpha |

## Protocol Specification

### Packet Structure

The SAM protocol employs a fixed-size 4-byte packet format with enhanced error detection:

| Field | Size | Offset | Description |
| --- | --- | --- | --- |
| type_flags | 1 byte | 0 | Message type (3 MSB) and command-specific flags (5 LSB) |
| data[0] | 1 byte | 1 | First data byte (interpretation depends on message type) |
| data[1] | 1 byte | 2 | Second data byte (interpretation depends on message type) |
| checksum | 1 byte | 3 | CRC8 checksum of first 3 bytes for error detection |

**C Structure:**
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

### Message Types

| Type Value (Binary) | Type Value (Hex) | Name | Direction | Description |
| --- | --- | --- | --- | --- |
| `0b000xxxxx` | `0x00` | TYPE_BUTTON | MCU → Host | Button state change events |
| `0b001xxxxx` | `0x20` | TYPE_LED | Host ↔ MCU | LED control commands and status |
| `0b010xxxxx` | `0x40` | TYPE_POWER | Host ↔ MCU | Power management and metrics |
| `0b011xxxxx` | `0x60` | TYPE_DISPLAY | Host ↔ MCU | E-ink display control and status |
| `0b100xxxxx` | `0x80` | TYPE_DEBUG_CODE | MCU → Host | Numeric debug codes |
| `0b101xxxxx` | `0xA0` | TYPE_DEBUG_TEXT | MCU → Host | Text debug messages |
| `0b110xxxxx` | `0xC0` | TYPE_SYSTEM | Host ↔ MCU | Core system control commands |
| `0b111xxxxx` | `0xE0` | TYPE_EXTENDED | Host ↔ MCU | Extended commands |

### CRC8 Checksum Algorithm

The protocol uses CRC8 with polynomial 0x07 (x^8 + x^2 + x + 1):

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

**Validation:**
```c
uint8_t validate_packet(const struct sam_protocol_packet *packet) {
    uint8_t calculated_crc = crc8_calculate((uint8_t*)packet, 3);
    return calculated_crc == packet->checksum;
}
```

## LED Control Specification

### Technical Specifications

- **Communication Direction**: Host → RP2040 (commands), RP2040 → Host (status)
- **Supported LEDs**: 16 LEDs (ID 0-15)
- **Color Depth**: 4 bits per channel (RGB444), 4096 colors
- **LED Modes**: 4 modes (Static, Blink, Fade, Reserved)
- **Timing Resolution**: 4 levels (100ms, 200ms, 500ms, 1000ms)
- **Execution Model**: Immediate execution (no queuing)

### LED Command Structure

```
type_flags: [001][E][LED_ID(4-bit)]
           Type=1 |E| ID(0-15)

data[0]:    [R(4-bit)][G(4-bit)]
            Red(0-15) |Green(0-15)

data[1]:    [B(4-bit)][MODE(2-bit)][TIME(2-bit)]
            Blue(0-15)|Mode(0-3)   |Timing(0-3)

checksum:   CRC8 of previous 3 bytes
```

### Field Definitions

**Execute Bit (E):**
- `0`: Command packet (set LED state)
- `1`: Execute/Status packet (reserved for future use)

**LED ID (4-bit):**
- `0x0` - `0xF`: LED identifiers 0-15

**RGB Values (4-bit each):**
- `0x0` - `0xF`: Color intensity (0-15)
- Scaled to 0-255 range: `rgb_8bit = (rgb_4bit * 255) / 15`

**LED Mode (2-bit):**
- `00`: Static - solid color
- `01`: Blink - on/off pattern  
- `10`: Fade - smooth brightness transitions
- `11`: Rainbow - color cycling animation

**Timing (2-bit):**
- `00`: 100ms intervals
- `01`: 200ms intervals
- `10`: 500ms intervals  
- `11`: 1000ms intervals

### LED Command Examples

**Set LED 0 to Red (Static):**
```
type_flags: 0x20 = [001][0][0000] = TYPE_LED | LED_ID_0
data[0]:    0xF0 = [1111][0000]   = Red=15, Green=0
data[1]:    0x00 = [0000][00][00] = Blue=0, Mode=Static, Time=100ms
checksum:   CRC8([0x20, 0xF0, 0x00]) = 0x8E
Packet:     {0x20, 0xF0, 0x00, 0x8E}
```

**Set LED 5 to Blue (Blinking, 500ms):**
```
type_flags: 0x25 = [001][0][0101] = TYPE_LED | LED_ID_5
data[0]:    0x00 = [0000][0000]   = Red=0, Green=0
data[1]:    0xF6 = [1111][01][10] = Blue=15, Mode=Blink, Time=500ms
checksum:   CRC8([0x25, 0x00, 0xF6]) = 0x33
Packet:     {0x25, 0x00, 0xF6, 0x33}
```

**Set LED 15 to White (Fade, 1000ms):**
```
type_flags: 0x2F = [001][0][1111] = TYPE_LED | LED_ID_15
data[0]:    0xFF = [1111][1111]   = Red=15, Green=15
data[1]:    0xFB = [1111][10][11] = Blue=15, Mode=Fade, Time=1000ms
checksum:   CRC8([0x2F, 0xFF, 0xFB]) = 0x7B
Packet:     {0x2F, 0xFF, 0xFB, 0x7B}
```

## Power Management Specification 

### Power Command Structure

Power commands are separated into two categories to eliminate conflicts:

**Control Commands (0x00-0x0F):**
```
type_flags: [010][CMD(5-bit)]
           Type=2|Command(0-15)
```

**Reporting Commands (0x10-0x1F):**
```
type_flags: [010][CMD(5-bit)]
           Type=2|Command(16-31)
```

### Power Command Definitions

**Control Commands:**
- `0x40`: POWER_CMD_QUERY (0x00) - Query current power status
- `0x41`: POWER_CMD_SET (0x01) - Set power state
- `0x42`: POWER_CMD_SLEEP (0x02) - Enter sleep mode
- `0x43`: POWER_CMD_SHUTDOWN (0x03) - Shutdown system

**Reporting Commands:**
- `0x50`: POWER_CMD_CURRENT (0x10) - Current draw reporting
- `0x51`: POWER_CMD_BATTERY (0x11) - Battery state reporting
- `0x52`: POWER_CMD_TEMP (0x12) - Temperature reporting
- `0x53`: POWER_CMD_VOLTAGE (0x13) - Voltage reporting
- `0x5F`: POWER_CMD_REQUEST_METRICS (0x1F) - Request all metrics

### Power Command Examples

**Query Power Status:**
```
type_flags: 0x40 = [010][00000] = TYPE_POWER | POWER_CMD_QUERY
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x40, 0x00, 0x00]) = 0x8C
Packet:     {0x40, 0x00, 0x00, 0x8C}
```

**Set Running State:**
```
type_flags: 0x41 = [010][00001] = TYPE_POWER | POWER_CMD_SET
data[0]:    0x01 = Running state
data[1]:    0x00 = Flags (reserved)
checksum:   CRC8([0x41, 0x01, 0x00]) = 0x8A
Packet:     {0x41, 0x01, 0x00, 0x8A}
```

**Report Current Draw (250mA):**
```
type_flags: 0x50 = [010][10000] = TYPE_POWER | POWER_CMD_CURRENT
data[0]:    0xFA = 250 & 0xFF (low byte)
data[1]:    0x00 = (250 >> 8) & 0xFF (high byte)
checksum:   CRC8([0x50, 0xFA, 0x00]) = 0x0C
Packet:     {0x50, 0xFA, 0x00, 0x0C}
```

**Request All Metrics:**
```
type_flags: 0x5F = [010][11111] = TYPE_POWER | POWER_CMD_REQUEST_METRICS
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x5F, 0x00, 0x00]) = 0x93
Packet:     {0x5F, 0x00, 0x00, 0x93}
```

## Button Interface Specification

Button events use the 5 least significant bits of the `type_flags` byte:

```
type_flags: [000][BUTTON_STATE(5-bit)]
           Type=0|PSDU (P=Power, S=Select, D=Down, U=Up)
```

**Button Mapping:**
- Bit 0 (0x01): UP button
- Bit 1 (0x02): DOWN button
- Bit 2 (0x04): SELECT button
- Bit 3 (0x08): POWER button
- Bit 4 (0x10): Reserved

**Button Examples:**

**UP Button Pressed:**
```
type_flags: 0x01 = [000][00001] = TYPE_BUTTON | UP_PRESSED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x01, 0x00, 0x00]) = 0x55
Packet:     {0x01, 0x00, 0x00, 0x55}
```

**SELECT + DOWN Pressed:**
```
type_flags: 0x06 = [000][00110] = TYPE_BUTTON | SELECT_PRESSED | DOWN_PRESSED
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0x06, 0x00, 0x00]) = 0x52
Packet:     {0x06, 0x00, 0x00, 0x52}
```

## System Commands Specification

System commands provide core control functions:

```
type_flags: [110][ACTION(5-bit)]
           Type=6|Action(0-31)
```

**System Actions:**
- `0xC0`: SYSTEM_PING (0x00) - Connectivity test
- `0xC1`: SYSTEM_RESET (0x01) - Reset microcontroller
- `0xC2`: SYSTEM_VERSION (0x02) - Version information
- `0xC3`: SYSTEM_STATUS (0x03) - System status
- `0xC4`: SYSTEM_CONFIG (0x04) - Configuration

**System Command Examples:**

**Ping Request:**
```
type_flags: 0xC0 = [110][00000] = TYPE_SYSTEM | SYSTEM_PING
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0xC0, 0x00, 0x00]) = 0x8B
Packet:     {0xC0, 0x00, 0x00, 0x8B}
```

**Version Request:**
```
type_flags: 0xC2 = [110][00010] = TYPE_SYSTEM | SYSTEM_VERSION
data[0]:    0x00 = Reserved
data[1]:    0x00 = Reserved
checksum:   CRC8([0xC2, 0x00, 0x00]) = 0x89
Packet:     {0xC2, 0x00, 0x00, 0x89}
```

## Debug Interface Specification

**Debug Code Categories:**
- Category 0: System events
- Category 1: Error conditions
- Category 2: Button events
- Category 3: LED events
- Category 4: Power events
- Category 5: Display events
- Category 6: Communication events
- Category 7: Performance metrics

**Debug Code Structure:**
```
type_flags: [100][CATEGORY(5-bit)]
           Type=4|Category(0-31)
```

**Debug Text Structure:**
```
type_flags: [101][FLAGS(5-bit)]
           Type=5|F|C|SEQ (F=First, C=Continue, SEQ=Sequence)
```

## Error Handling and Recovery

### Error Detection

1. **CRC8 Validation**: Every packet validated with CRC8 checksum
2. **Field Validation**: Command codes and parameters validated
3. **Boundary Checking**: LED IDs, power values, etc. checked

### Recovery Procedures

1. **CRC8 Error**: Discard packet, log error, continue processing
2. **Invalid Command**: Send error response, log warning
3. **Parameter Error**: Send error response with details
4. **Timeout**: Retry up to 3 times, then escalate

### Error Response Format

```
type_flags: [100][ERROR_CODE(5-bit)]
           Type=4|Error Code(0-31)
data[0]:    Original command type
data[1]:    Error details
checksum:   CRC8 of response
```

## Implementation Requirements

### Kernel Driver Requirements

1. **CRC8 Implementation**: Must implement CRC8 with polynomial 0x07
2. **LED Support**: Must support 16 LEDs (ID 0-15)
3. **Power Commands**: Must use separated command codes
4. **Error Handling**: Must validate all packets and handle errors gracefully
5. **Constants**: Must define all command constants matching this specification

### Firmware Requirements

1. **CRC8 Validation**: Must validate all incoming packets
2. **LED Control**: Must support 16 LEDs with 4 modes and 4 timing levels
3. **Power Reporting**: Must implement separated reporting commands
4. **Error Responses**: Must send appropriate error responses
5. **Compatibility**: Must reject v0.1.0 packets (different checksum)

### Validation Requirements

1. **Packet Validation**: Every implementation must validate packet format
2. **CRC8 Testing**: Must include CRC8 test vectors
3. **Error Injection**: Must test error handling with corrupted packets
4. **Boundary Testing**: Must test all boundary conditions
5. **Performance Testing**: Must validate timing requirements
