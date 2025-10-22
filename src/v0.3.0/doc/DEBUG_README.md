# RP2040 SAM Firmware Debug Guide

## Enhanced UART Debugging

The firmware now includes comprehensive UART debugging controlled by the `DEBUG_UART` macro in `main.py`.

### Debug Features Added:

1. **Detailed UART packet logging** - All TX/RX packets are logged with hex representation
2. **Raw UART data logging** - Shows actual bytes received/transmitted
3. **Timestamped debug messages** - All debug messages include millisecond timestamps
4. **Heartbeat functionality** - Periodic ping packets to show the RP2040 is alive
5. **Boot notification tracking** - Logs when boot notification is sent
6. **Invalid packet logging** - Shows packets with bad checksums

### How to Enable/Disable Debug:

In `main.py`, line 19:
```python
DEBUG_UART = True  # Enable detailed UART debugging
```

Set to `False` to disable debug output.

### Debug Output Format:

```
[timestamp] UART: message
[timestamp] UART-RX: [hex bytes] len=4
[timestamp] UART-TX: [hex bytes] len=4
[timestamp] UART-RAW-RX: [hex bytes] len=X
[timestamp] UART-TX-HEARTBEAT: [hex bytes] len=4
```

### Monitoring Debug Output:

#### Option 1: Using the provided monitor script
```bash
python3 uart_monitor.py /dev/ttyACM0
```

#### Option 2: Using minicom
```bash
minicom -D /dev/ttyACM0 -b 115200
```

#### Option 3: Using screen
```bash
screen /dev/ttyACM0 115200
```

### Key Debug Messages to Look For:

1. **Firmware startup:**
   ```
   [xxx] UART: UART0 initialized - TX:GPIO0, RX:GPIO1, Baud:115200
   [xxx] UART: === RP2040 SAM Firmware Initialized ===
   [xxx] UART: Firmware ready for kernel driver communication
   ```

2. **Boot notification:**
   ```
   [xxx] UART-TX: [40 01 00 41] len=4
   [xxx] UART: [Boot] Boot notification sent to SoM
   ```

3. **Heartbeat pings (every 10 seconds):**
   ```
   [xxx] UART-TX-HEARTBEAT: [C0 00 00 C0] len=4
   [xxx] UART: Heartbeat ping sent to kernel driver
   ```

4. **Receiving data from kernel driver:**
   ```
   [xxx] UART-RAW-RX: [C0 00 00 C0] len=4
   [xxx] UART: Buffer now has 4 bytes
   [xxx] UART-RX: [C0 00 00 C0] len=4
   [xxx] UART: Processing packet type: 0xC0
   ```

5. **Button presses:**
   ```
   [xxx] UART-TX: [01 00 00 01] len=4
   [xxx] UART: Button states - UP: True, DOWN: False, SELECT: False
   ```

### Troubleshooting Guide:

#### If you see NO output at all:
1. Check if RP2040 is powered and running
2. Verify USB connection
3. Check if correct serial device is being used
4. Ensure PRODUCTION=False in main.py

#### If you see firmware startup but no kernel driver communication:
1. Check if kernel driver is loaded: `lsmod | grep pamir`
2. Verify UART device in device tree is correct
3. Check dmesg for kernel driver errors
4. Ensure correct GPIO pins are configured

#### If you see heartbeat pings but no responses:
1. Kernel driver is not receiving/processing packets
2. Check UART hardware connection
3. Verify baud rate matches (115200)
4. Check if serdev device is created properly

#### If you see invalid packets:
1. UART communication is happening but with errors
2. Check for electrical interference
3. Verify ground connections
4. Check for timing issues

### Protocol Packet Reference:

| Packet Type | Hex Pattern | Description |
|-------------|-------------|-------------|
| System Ping | `C0 00 00 C0` | Heartbeat/connectivity test |
| Button UP | `01 00 00 01` | UP button pressed |
| Button DOWN | `02 00 00 02` | DOWN button pressed |
| Button SELECT | `04 00 00 04` | SELECT button pressed |
| Boot Notification | `40 01 00 41` | RP2040 boot complete |
| Power Query | `40 XX XX XX` | Power status response |

### Common Issues and Solutions:

1. **No serial device found:**
   - Check USB connection
   - Try different USB ports
   - Verify RP2040 is in correct mode

2. **Permission denied accessing serial device:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```

3. **Garbled output:**
   - Wrong baud rate
   - Hardware connection issues
   - Try different serial terminal program

4. **Intermittent communication:**
   - Check power supply stability
   - Verify cable quality
   - Check for electromagnetic interference

### Next Steps for Debugging:

1. **If RP2040 firmware is working but no kernel communication:**
   - Check kernel driver loading
   - Verify device tree configuration
   - Check UART hardware setup

2. **If kernel driver loads but no device created:**
   - Check device tree overlay
   - Verify GPIO pin configuration
   - Check for serdev binding issues

3. **If device exists but no communication:**
   - Test basic UART with echo commands
   - Check baud rate configuration
   - Verify electrical connections