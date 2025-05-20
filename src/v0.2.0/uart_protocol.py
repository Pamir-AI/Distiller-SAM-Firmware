"""
Pamir AI UART Protocol Specification
===================================================

Overview:
---------
This protocol provides a compact, efficient binary communication interface between
a Linux host system and the RP2040 microcontroller. It replaces the previous
JSON-based approach with a fixed-size binary packet format to reduce CPU usage
and improve reliability.

Packet Structure:
----------------
Each packet is exactly 4 bytes:
  Byte 0: Type flags (3 bits type + 5 bits subtype/data)
  Byte 1: Data byte 1
  Byte 2: Data byte 2
  Byte 3: Checksum (XOR of bytes 0-2)

Message Types (3 most significant bits of Byte 0):
-------------------------------------------------
0b000 (0x00): Button events      - Reports button state changes
0b001 (0x20): LED control        - Controls RGB LED state and animations
0b010 (0x40): Power management   - Power state and battery reporting
0b011 (0x60): Display commands   - E-ink display control
0b100 (0x80): Debug codes        - System diagnostics and errors
0b101 (0xA0): Debug text         - Debug text messages (multi-packet)
0b110 (0xC0): System commands    - System control and status
0b111 (0xE0): Extended commands  - Reserved for future expansion

Button Events (TYPE_BUTTON = 0x00):
----------------------------------
The 5 least significant bits in Byte 0 indicate which buttons are pressed:
  Bit 0: Up button     (0x01)
  Bit 1: Down button   (0x02)
  Bit 2: Select button (0x04)
  Bit 3: Power button  (0x08)
  Bit 4: Reserved

Data bytes 1-2 are reserved for future use.

Example: 0x06 0x00 0x00 0x06 = Select+Down buttons pressed

LED Control (TYPE_LED = 0x20):
-----------------------------
Byte 0 (5 LSB bits):
  Bits 0-1: LED ID (0x00 = all LEDs)
  Bits 2-3: LED mode:
    00 (0x00): Static color
    01 (0x04): Blink
    10 (0x08): Fade
    11 (0x0C): Rainbow effect
  Bit 4: Command type:
    0 (0x00): Immediate command
    1 (0x10): Sequence command

Byte 1:
  Bits 4-7: Red value (0-15)
  Bits 0-3: Green value (0-15)

Byte 2:
  Bits 4-7: Blue value (0-15)
  Bits 0-3: Value parameter (brightness or timing)

Example: 0x24 0xF0 0x0F 0xDB = Static red color at full brightness

Power Management (TYPE_POWER = 0x40):
------------------------------------
Byte 0 (5 LSB bits):
  Bits 4-5: Command type:
    00 (0x00): Power status report
    01 (0x10): Battery level report
    10 (0x20): Power mode change
    11 (0x30): Shutdown command
  Bits 0-3: Subcommand or parameters

Bytes 1-2: Command-specific data

Example: 0x70 0x00 0x00 0x70 = Shutdown command

Debug Codes (TYPE_DEBUG_CODE = 0x80):
------------------------------------
Byte 0 (5 LSB bits):
  Bits 0-4: Debug category:
    0x00: System
    0x01: Input
    0x02: Display
    0x03: Memory
    0x04: Power

Byte 1: Debug code
Byte 2: Debug parameter

Example: 0x80 0x01 0x00 0x81 = System initialized

Debug Text (TYPE_DEBUG_TEXT = 0xA0):
----------------------------------
Used for sending multi-packet debug text messages:

Byte 0 (5 LSB bits):
  Bit 4: First chunk flag (0x10)
  Bit 3: Continue flag (0x08)
  Bits 0-2: Chunk number (0-7)

Bytes 1-2: Two UTF-8 bytes of the text message

System Commands (TYPE_SYSTEM = 0xC0):
-----------------------------------
Byte 0 (5 LSB bits):
  Bits 0-4: System action:
    0x00: Ping request/response
    0x01: System reset
    0x02: Version information
    0x03: Status request
    0x04: Configuration

Bytes 1-2: Command-specific data

Example: 0xC0 0x00 0x00 0xC0 = Ping request

Usage Examples:
--------------
1. Sending button state:
   [0x03, 0x00, 0x00, 0x03] = Up+Down buttons pressed

2. Setting LED to blink green:
   [0x24, 0x0F, 0x08, 0x2B] = Blink green LED with medium brightness

3. Request system version:
   [0xC2, 0x00, 0x00, 0xC2]

4. System will respond with:
   [0xC2, 0x01, 0x00, 0xC3] = Version 1.0

Implementation Notes:
-------------------
- All packets must include the correct checksum (XOR of first 3 bytes)
- The protocol is designed for efficiency and minimal overhead
- Commands requiring more than 2 bytes of data use multi-packet sequences
- Debug text messages split across multiple packets must be reassembled by the receiver
"""

import machine
import _thread

# Protocol definitions
# Message types (3 most significant bits)
TYPE_BUTTON = 0x00  # 0b000xxxxx
TYPE_LED = 0x20  # 0b001xxxxx
TYPE_POWER = 0x40  # 0b010xxxxx
TYPE_DISPLAY = 0x60  # 0b011xxxxx
TYPE_DEBUG_CODE = 0x80  # 0b100xxxxx
TYPE_DEBUG_TEXT = 0xA0  # 0b101xxxxx
TYPE_SYSTEM = 0xC0  # 0b110xxxxx
TYPE_EXTENDED = 0xE0  # 0b111xxxxx
TYPE_MASK = 0xE0  # 0b11100000

# Button event flags (5 least significant bits)
BTN_UP_MASK = 0x01
BTN_DOWN_MASK = 0x02
BTN_SELECT_MASK = 0x04
BTN_POWER_MASK = 0x08

# LED control flags
LED_CMD_IMMEDIATE = 0x00
LED_CMD_SEQUENCE = 0x10
LED_MODE_STATIC = 0x00
LED_MODE_BLINK = 0x04
LED_MODE_FADE = 0x08
LED_MODE_RAINBOW = 0x0C
LED_MODE_MASK = 0x0C
LED_ID_ALL = 0x00
LED_ID_MASK = 0x03

# Debug categories
DEBUG_CAT_SYSTEM = 0x00
DEBUG_CAT_INPUT = 0x01
DEBUG_CAT_DISPLAY = 0x02
DEBUG_CAT_MEMORY = 0x03
DEBUG_CAT_POWER = 0x04

# System actions
SYSTEM_PING = 0x00
SYSTEM_RESET = 0x01
SYSTEM_VERSION = 0x02
SYSTEM_STATUS = 0x03
SYSTEM_CONFIG = 0x04

# Constants
PACKET_SIZE = 4


class PamirProtocol:
    """Handles the ultra-optimized binary protocol for Pamir devices"""

    def __init__(self, uart, debug=False, wdt=None):
        """Initialize the protocol with a configured UART instance

        Args:
            uart: Configured machine.UART instance
            debug: Enable debug output
            wdt: Watchdog timer instance for feeding during long operations
        """
        self.uart = uart
        self.debug = debug
        self.wdt = wdt

        # Initialize receive buffer and state
        self.rx_buffer = bytearray(PACKET_SIZE)
        self.rx_pos = 0

        # Create lock for thread-safe UART access
        self.uart_lock = _thread.allocate_lock()

        # Callback handlers for different packet types
        self.handlers = {
            TYPE_BUTTON: None,
            TYPE_LED: None,
            TYPE_POWER: None,
            TYPE_DISPLAY: None,
            TYPE_DEBUG_CODE: None,
            TYPE_DEBUG_TEXT: None,
            TYPE_SYSTEM: None,
            TYPE_EXTENDED: None,
        }

    def register_handler(self, packet_type, handler_func):
        """Register a callback function for a specific packet type

        Args:
            packet_type: One of the TYPE_* constants
            handler_func: Function to call with the packet data
        """
        self.handlers[packet_type] = handler_func

    def calculate_checksum(self, type_flags, data1, data2):
        """Calculate XOR checksum for packet

        Args:
            type_flags: Byte 0 of packet
            data1: Byte 1 of packet
            data2: Byte 2 of packet

        Returns:
            Checksum byte
        """
        return type_flags ^ data1 ^ data2

    def send_packet(self, type_flags, data1, data2):
        """Send a packet over UART

        Args:
            type_flags: Byte 0 of packet (type and subtype/data)
            data1: Byte 1 of packet (data)
            data2: Byte 2 of packet (data)
        """
        checksum = self.calculate_checksum(type_flags, data1, data2)
        packet = bytes([type_flags, data1, data2, checksum])

        with self.uart_lock:
            self.uart.write(packet)

        if self.debug:
            print(f"TX: {packet.hex()}")

    def verify_checksum(self, packet):
        """Verify packet checksum

        Args:
            packet: 4-byte packet

        Returns:
            True if checksum is valid, False otherwise
        """
        return packet[3] == (packet[0] ^ packet[1] ^ packet[2])

    def send_button_state(self, btn_state):
        """Send button state packet

        Args:
            btn_state: Button state bitmask (combination of BTN_* constants)
        """
        self.send_packet(TYPE_BUTTON | (btn_state & 0x1F), 0, 0)

    def send_led_command(
        self,
        led_id=LED_ID_ALL,
        mode=LED_MODE_STATIC,
        is_sequence=False,
        r=0,
        g=0,
        b=0,
        value=0,
    ):
        """Send LED control packet

        Args:
            led_id: LED ID (0 = all LEDs)
            mode: LED mode (LED_MODE_*)
            is_sequence: True for sequence, False for immediate
            r: Red value (0-15)
            g: Green value (0-15)
            b: Blue value (0-15)
            value: Value parameter for brightness or timing (0-15)
        """
        flags = TYPE_LED | (led_id & LED_ID_MASK) | (mode & LED_MODE_MASK)
        if is_sequence:
            flags |= LED_CMD_SEQUENCE

        # Pack RGB and value into data bytes
        data1 = ((r & 0x0F) << 4) | (g & 0x0F)
        data2 = ((b & 0x0F) << 4) | (value & 0x0F)

        self.send_packet(flags, data1, data2)

    def send_power_command(self, cmd_type, param=0, data1=0, data2=0):
        """Send power management packet

        Args:
            cmd_type: Command type (0-3)
            param: Command parameter (0-15)
            data1: First data byte
            data2: Second data byte
        """
        flags = TYPE_POWER | ((cmd_type & 0x03) << 4) | (param & 0x0F)
        self.send_packet(flags, data1, data2)

    def send_debug_code(self, category, code, param):
        """Send debug code packet

        Args:
            category: Debug category (DEBUG_CAT_*)
            code: Debug code
            param: Debug parameter
        """
        self.send_packet(TYPE_DEBUG_CODE | (category & 0x1F), code, param)

    def _feed_wdt(self):
        """Feed the watchdog timer if one was provided"""
        if self.wdt is not None:
            self.wdt.feed()

    def send_debug_text(self, text):
        """Send debug text message (multi-packet if needed)

        Args:
            text: Text message to send
        """
        # Convert text to bytes
        text_bytes = text.encode("utf-8")

        # Split into chunks of 2 bytes
        chunks = []
        for i in range(0, len(text_bytes), 2):
            chunk = text_bytes[i : i + 2]
            if len(chunk) < 2:
                # Pad to 2 bytes
                chunk = chunk + b"\x00"
            chunks.append(chunk)

        # Send chunks
        for i, chunk in enumerate(chunks):
            first_chunk = 0x10 if i == 0 else 0x00
            continue_flag = 0x08 if i < len(chunks) - 1 else 0x00
            chunk_num = i % 8

            flags = TYPE_DEBUG_TEXT | first_chunk | continue_flag | chunk_num
            self.send_packet(flags, chunk[0], chunk[1])

            # Feed WDT after every few chunks to prevent timeout
            if i % 5 == 0:
                self._feed_wdt()

    def send_system_command(self, action, data1=0, data2=0):
        """Send system command packet

        Args:
            action: System action (SYSTEM_*)
            data1: First data byte
            data2: Second data byte
        """
        self.send_packet(TYPE_SYSTEM | (action & 0x1F), data1, data2)

    def send_display_command(self, cmd, param1=0, param2=0):
        """Send display command packet

        Args:
            cmd: Display command
            param1: First parameter
            param2: Second parameter
        """
        self.send_packet(TYPE_DISPLAY | (cmd & 0x1F), param1, param2)

    def send_ping(self):
        """Send ping request"""
        self.send_system_command(SYSTEM_PING)

    def send_version_request(self):
        """Send version information request"""
        self.send_system_command(SYSTEM_VERSION)

    def send_status_request(self):
        """Send status request"""
        self.send_system_command(SYSTEM_STATUS)

    def send_reset_command(self):
        """Send system reset command"""
        self.send_system_command(SYSTEM_RESET)

    def send_shutdown_command(self):
        """Send shutdown command"""
        self.send_power_command(3)  # 3 = shutdown (0x30)

    def process_packet(self, packet):
        """Process a received packet

        Args:
            packet: 4-byte packet

        Returns:
            True if packet was handled, False otherwise
        """
        # Check packet size
        if len(packet) != PACKET_SIZE:
            if self.debug:
                print(f"Invalid packet size: {len(packet)}")
            return False

        # Verify checksum
        if not self.verify_checksum(packet):
            if self.debug:
                print(f"Invalid checksum: {packet.hex()}")
            return False

        if self.debug:
            print(f"RX: {packet.hex()}")

        # Extract packet type and call appropriate handler
        packet_type = packet[0] & TYPE_MASK

        # Feed WDT before potentially time-consuming handler
        self._feed_wdt()

        if packet_type in self.handlers and self.handlers[packet_type] is not None:
            try:
                self.handlers[packet_type](packet)
                return True
            except Exception as e:
                if self.debug:
                    print(f"Handler exception: {e}")
                return False
        return False

    def check_uart(self):
        """Check for and process any available UART data

        Returns:
            Number of packets processed
        """
        packets_processed = 0

        if self.uart.any():
            # Read available data
            data = self.uart.read(self.uart.any())

            for byte in data:
                # Add byte to buffer
                self.rx_buffer[self.rx_pos] = byte
                self.rx_pos += 1

                # Check if we have a complete packet
                if self.rx_pos == PACKET_SIZE:
                    # Process packet
                    if self.process_packet(self.rx_buffer):
                        packets_processed += 1

                    # Reset buffer position
                    self.rx_pos = 0

                # Feed WDT during extended processing
                if packets_processed > 0 and packets_processed % 10 == 0:
                    self._feed_wdt()

        return packets_processed

    def parse_button_packet(self, packet):
        """Parse button state from packet

        Args:
            packet: 4-byte packet

        Returns:
            Dictionary with button states
        """
        btn_state = packet[0] & 0x1F
        return {
            "up": bool(btn_state & BTN_UP_MASK),
            "down": bool(btn_state & BTN_DOWN_MASK),
            "select": bool(btn_state & BTN_SELECT_MASK),
            "power": bool(btn_state & BTN_POWER_MASK),
        }

    def parse_led_packet(self, packet):
        """Parse LED control parameters from packet

        Args:
            packet: 4-byte packet

        Returns:
            Dictionary with LED parameters
        """
        led_id = packet[0] & LED_ID_MASK
        mode = packet[0] & LED_MODE_MASK
        is_sequence = bool(packet[0] & LED_CMD_SEQUENCE)

        r = (packet[1] >> 4) & 0x0F
        g = packet[1] & 0x0F
        b = (packet[2] >> 4) & 0x0F
        value = packet[2] & 0x0F

        return {
            "led_id": led_id,
            "mode": mode,
            "is_sequence": is_sequence,
            "r": r,
            "g": g,
            "b": b,
            "value": value,
        }

    def parse_power_packet(self, packet):
        """Parse power management parameters from packet

        Args:
            packet: 4-byte packet

        Returns:
            Dictionary with power management parameters
        """
        cmd_type = (packet[0] >> 4) & 0x03
        param = packet[0] & 0x0F

        return {
            "cmd_type": cmd_type,
            "param": param,
            "data1": packet[1],
            "data2": packet[2],
        }

    def get_packet_type_str(self, packet):
        """Get human-readable packet type

        Args:
            packet: 4-byte packet

        Returns:
            String describing the packet type
        """
        packet_type = packet[0] & TYPE_MASK

        types = {
            TYPE_BUTTON: "Button",
            TYPE_LED: "LED",
            TYPE_POWER: "Power",
            TYPE_DISPLAY: "Display",
            TYPE_DEBUG_CODE: "Debug Code",
            TYPE_DEBUG_TEXT: "Debug Text",
            TYPE_SYSTEM: "System",
            TYPE_EXTENDED: "Extended",
        }

        return types.get(packet_type, f"Unknown ({packet_type:#04x})")
