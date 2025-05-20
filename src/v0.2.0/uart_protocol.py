"""
Pamir AI Signal Aggregation Module (SAM) Specification
======================================================

Overview:
---------
The Pamir AI SAM is an binary communication protocol for communication
between a Linux host processor and an RP2040 microcontroller. It provides a
compact, reliable interface for buttons, LEDs, power management, and system
commands.

The protocol uses a fixed-size 4-byte packet format with checksum validation
to ensure reliable communication.

Protocol Features:
----------------
- Button input reporting with debounce
- LED control with animation and sequences
- Power management, including boot/shutdown notifications
- Poll-based power metrics reporting (current, voltage, temperature, battery)
- E-ink display status and control
- Version exchange for compatibility
- Debug logging via codes and text messages
- System commands for core functionality

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
0b111 (0xE0): Extended commands  - Version and future features

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
  Bits 0-3: LED ID (0x00-0x0F, supports up to 16 unique LEDs)
  Bit 4: Command type:
    0 (0x00): Queue color instruction
    1 (0x10): Execute queued sequence

Byte 1:
  Bits 4-7: Red value (0-15)
  Bits 0-3: Green value (0-15)

Byte 2:
  Bits 4-7: Blue value (0-15)
  Bits 0-3: Time value (delay between color changes, 0-15)

Queue-based LED control:
1. Send one or more packets with Command Type = 0 to queue colors
2. Send a final packet with Command Type = 1 to execute the entire sequence
3. The RP2040 will acknowledge when sequence completes

Completion Acknowledgment:
When an LED sequence completes, the RP2040 sends:
- TYPE_LED | LED_CMD_EXECUTE | LED_ID
- data[0] = 0xFF (completion indicator)
- data[1] = sequence length

Example: Queue red color for LED 1: 0x21 0xF0 0x05 0xD4
         Execute sequence for LED 1: 0x31 0x00 0x00 0x31

Power Management (TYPE_POWER = 0x40):
------------------------------------
Byte 0 (5 LSB bits):
  Bits 4-5: Command type:
    00 (0x00): Query current power status
    01 (0x10): Set power state (boot notification)
    10 (0x20): Enter sleep mode
    11 (0x30): System shutdown notification
  Bits 0-3: Subcommand or parameters

Bytes 1-2: Command-specific data

Boot/Shutdown Notifications:
- Boot: 0x50 0x01 0x00 0x51 (Linux has booted)
- Shutdown: 0x70 0x00 0x00 0x70 (Normal shutdown)

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

2. Queue red color for LED 0, then execute:
   [0x20, 0xF0, 0x00, 0xD0] = Queue red color
   [0x30, 0x00, 0x00, 0x30] = Execute sequence

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
- LED control uses a queue-based approach to support complex animations
- Boot and shutdown notifications help coordinate power states between Linux and RP2040

For complete documentation, see the README.md file in the Linux driver.
"""

import _thread

try:
    from version import VERSION_MAJOR, VERSION_MINOR
except ImportError:
    # Default values if version module is not available
    VERSION_MAJOR = 1
    VERSION_MINOR = 0
    VERSION_PATCH = 0
    VERSION_STRING = "1.0.0"

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
LED_CMD_QUEUE = 0x00  # Queue command for later execution
LED_CMD_EXECUTE = 0x10  # Execute all queued commands
LED_ID_MASK = 0x0F  # LED identifier mask (0-15)
LED_COMPLETION = 0xFF  # Value in data[0] indicating sequence completion

# Power commands
POWER_CMD_MASK = 0xF0  # Mask for power command bits
POWER_CMD_QUERY = 0x00  # Query current power status
POWER_CMD_SET = 0x10  # Set power state (boot notification)
POWER_CMD_SLEEP = 0x20  # Enter sleep mode
POWER_CMD_SHUTDOWN = 0x30  # System shutdown notification
POWER_CMD_CURRENT = 0x40  # Current draw reporting
POWER_CMD_BATTERY = 0x50  # Battery state reporting
POWER_CMD_TEMP = 0x60  # Temperature reporting
POWER_CMD_VOLTAGE = 0x70  # Voltage reporting
POWER_CMD_REQUEST_METRICS = 0x80  # Request all power metrics

# Power states
POWER_STATE_OFF = 0x00  # Powered off
POWER_STATE_RUNNING = 0x01  # System running
POWER_STATE_SUSPEND = 0x02  # System suspended/sleeping
POWER_STATE_LOW = 0x03  # Low power mode

# Shutdown modes
SHUTDOWN_MODE_NORMAL = 0x00  # Normal planned shutdown
SHUTDOWN_MODE_EMERGENCY = 0x01  # Emergency shutdown (thermal, etc)
SHUTDOWN_MODE_REBOOT = 0x02  # System is rebooting

# Debug categories
DEBUG_CAT_SYSTEM = 0x00
DEBUG_CAT_INPUT = 0x01
DEBUG_CAT_DISPLAY = 0x02
DEBUG_CAT_MEMORY = 0x03
DEBUG_CAT_POWER = 0x04

# Debug text flags
DEBUG_FIRST_CHUNK = 0x10
DEBUG_CONTINUE = 0x08
DEBUG_CHUNK_MASK = 0x07

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

        # LED sequence queues for each LED ID (up to 16 LEDs)
        self.led_queues = [[] for _ in range(16)]
        self.led_active_sequence = [False] * 16  # Track if a sequence is running

        # Button state tracking
        self.prev_btn_state = 0

        # Power state tracking
        self.power_state = POWER_STATE_OFF
        self.linux_booted = False

        # Host version tracking
        self.host_version = {"major": 0, "minor": 0, "patch": 0, "string": "0.0.0"}

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
            print(f"TX: {[hex(b) for b in packet]}")

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
        if btn_state != self.prev_btn_state:
            self.prev_btn_state = btn_state
            self.send_packet(TYPE_BUTTON | (btn_state & 0x1F), 0, 0)

    def send_led_command(self, led_id, execute, r, g, b, time_value):
        """Send LED control command

        Args:
            led_id: LED ID (0-15)
            execute: True for execute, False for queue
            r: Red value (0-15)
            g: Green value (0-15)
            b: Blue value (0-15)
            time_value: Time value for transitions (0-15)
        """
        if led_id > 15 or r > 15 or g > 15 or b > 15 or time_value > 15:
            if self.debug:
                print("Invalid LED parameters")
            return

        cmd_flags = LED_CMD_EXECUTE if execute else LED_CMD_QUEUE
        self.send_packet(
            TYPE_LED | cmd_flags | (led_id & LED_ID_MASK),
            ((r & 0x0F) << 4) | (g & 0x0F),
            ((b & 0x0F) << 4) | (time_value & 0x0F),
        )

    def send_led_completion(self, led_id, sequence_length):
        """Send LED sequence completion acknowledgment

        Args:
            led_id: LED ID that completed execution
            sequence_length: Length of the executed sequence
        """
        self.send_packet(
            TYPE_LED | LED_CMD_EXECUTE | (led_id & LED_ID_MASK),
            LED_COMPLETION,
            sequence_length,
        )

    def queue_led_color(self, led_id, r, g, b, time_value):
        """Queue a color instruction for an LED

        Args:
            led_id: LED ID (0-15)
            r: Red value (0-15)
            g: Green value (0-15)
            b: Blue value (0-15)
            time_value: Time value (0-15) for transitions
        """
        # Validate parameters
        if led_id > 15 or r > 15 or g > 15 or b > 15 or time_value > 15:
            if self.debug:
                print("Invalid LED parameters")
            return

        self.send_packet(
            TYPE_LED | (led_id & LED_ID_MASK),
            ((r & 0x0F) << 4) | (g & 0x0F),
            ((b & 0x0F) << 4) | (time_value & 0x0F),
        )

    def execute_led_sequence(self, led_id):
        """Execute the queued sequence for an LED

        Args:
            led_id: LED ID (0-15)
        """
        self.send_packet(
            TYPE_LED | LED_CMD_EXECUTE | (led_id & LED_ID_MASK), 0x00, 0x00
        )

    def send_power_query_response(self, state):
        """Send power query response

        Args:
            state: Current power state
        """
        self.send_packet(TYPE_POWER | POWER_CMD_QUERY, state, 0x00)

    def send_power_current(self, current_ma):
        """Send current power draw in mA

        Args:
            current_ma: Current in mA (16-bit value)
        """
        self.send_packet(
            TYPE_POWER | POWER_CMD_CURRENT,
            current_ma & 0xFF,  # Low byte
            (current_ma >> 8) & 0xFF,  # High byte
        )

    def send_power_battery(self, battery_pct):
        """Send battery state of charge

        Args:
            battery_pct: Battery percentage (0-100)
        """
        self.send_packet(
            TYPE_POWER | POWER_CMD_BATTERY,
            battery_pct & 0xFF,  # Low byte
            (battery_pct >> 8) & 0xFF,  # High byte
        )

    def send_power_temperature(self, temp_decidegc):
        """Send temperature in 0.1°C units

        Args:
            temp_decidegc: Temperature in 0.1°C units (16-bit value)
        """
        self.send_packet(
            TYPE_POWER | POWER_CMD_TEMP,
            temp_decidegc & 0xFF,  # Low byte
            (temp_decidegc >> 8) & 0xFF,  # High byte
        )

    def send_power_voltage(self, voltage_mv):
        """Send voltage in mV

        Args:
            voltage_mv: Voltage in mV (16-bit value)
        """
        self.send_packet(
            TYPE_POWER | POWER_CMD_VOLTAGE,
            voltage_mv & 0xFF,  # Low byte
            (voltage_mv >> 8) & 0xFF,  # High byte
        )

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
        chunks = [text_bytes[i : i + 2] for i in range(0, len(text_bytes), 2)]

        for i, chunk in enumerate(chunks):
            # First chunk flag
            first_chunk = DEBUG_FIRST_CHUNK if i == 0 else 0x00
            # Continue flag (more chunks follow)
            continue_flag = DEBUG_CONTINUE if i < len(chunks) - 1 else 0x00
            # Chunk number (0-7, wrapping around)
            chunk_num = i % 8

            type_flags = TYPE_DEBUG_TEXT | first_chunk | continue_flag | chunk_num

            # Pad chunk to 2 bytes if needed
            if len(chunk) == 1:
                chunk = chunk + b"\x00"

            self.send_packet(type_flags, chunk[0], chunk[1] if len(chunk) > 1 else 0)

            # Feed WDT during long messages
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

    def send_ping_response(self):
        """Send ping response"""
        self.send_system_command(SYSTEM_PING, 0, 0)

    def send_version_info(self, major, minor):
        """Send version information

        Args:
            major: Major version number
            minor: Minor version number
        """
        self.send_system_command(SYSTEM_VERSION, major, minor)

    def send_display_status(self, status, flags=0):
        """Send display status

        Args:
            status: Display status code
            flags: Status flags
        """
        self.send_packet(TYPE_DISPLAY | 0x00, status, flags)

    def send_display_refresh_complete(self):
        """Send display refresh complete notification"""
        self.send_packet(TYPE_DISPLAY | 0x01, 0xFF, 0x00)

    def send_power_shutdown(self, shutdown_mode=SHUTDOWN_MODE_NORMAL):
        """Send shutdown notification to host

        Args:
            shutdown_mode: Shutdown mode (0=normal, 1=emergency, 2=reboot)
        """
        self.send_packet(TYPE_POWER | POWER_CMD_SHUTDOWN, shutdown_mode, 0x00)

    def send_power_shutdown_ack(self, mode, flag=1):
        """Send acknowledgment for shutdown command

        Args:
            mode: Original shutdown mode
            flag: Flag value (1=ACK)
        """
        self.send_packet(TYPE_POWER | POWER_CMD_SHUTDOWN, mode, flag)

    def send_power_sleep(self, delay=0):
        """Send sleep mode notification to host

        Args:
            delay: Delay before sleep in seconds
        """
        self.send_packet(TYPE_POWER | POWER_CMD_SLEEP, delay, 0x00)

    def send_power_boot(self):
        """Notify host that system is booted and running"""
        self.send_packet(TYPE_POWER | POWER_CMD_SET, POWER_STATE_RUNNING, 0x00)

    def request_power_metrics(self):
        """Request power metrics from firmware

        Send a request to trigger the firmware to send current power metrics
        """
        self.send_packet(TYPE_POWER | POWER_CMD_REQUEST_METRICS, 0x00, 0x00)

    def process_packet(self, packet):
        """Process a received packet

        Args:
            packet: 4-byte packet
        """
        # Verify checksum
        if not self.verify_checksum(packet):
            if self.debug:
                print(f"Invalid checksum: {[hex(b) for b in packet]}")
            return False

        packet_type = packet[0] & TYPE_MASK

        if self.debug:
            print(f"Processing packet type {hex(packet_type)}")

        # Process based on packet type
        if packet_type == TYPE_BUTTON:
            if self.handlers[TYPE_BUTTON]:
                self.handlers[TYPE_BUTTON](packet)

        elif packet_type == TYPE_LED:
            self._process_led_packet(packet)
            if self.handlers[TYPE_LED]:
                self.handlers[TYPE_LED](packet)

        elif packet_type == TYPE_POWER:
            self._process_power_packet(packet)
            if self.handlers[TYPE_POWER]:
                self.handlers[TYPE_POWER](packet)

        elif packet_type == TYPE_DISPLAY:
            if self.handlers[TYPE_DISPLAY]:
                self.handlers[TYPE_DISPLAY](packet)

        elif packet_type == TYPE_DEBUG_CODE:
            if self.handlers[TYPE_DEBUG_CODE]:
                self.handlers[TYPE_DEBUG_CODE](packet)

        elif packet_type == TYPE_DEBUG_TEXT:
            if self.handlers[TYPE_DEBUG_TEXT]:
                self.handlers[TYPE_DEBUG_TEXT](packet)

        elif packet_type == TYPE_SYSTEM:
            self._process_system_packet(packet)
            if self.handlers[TYPE_SYSTEM]:
                self.handlers[TYPE_SYSTEM](packet)

        elif packet_type == TYPE_EXTENDED:
            self.process_extended_packet(packet)

        return True

    def _process_led_packet(self, packet):
        """Process LED control packet"""
        cmd_type = packet[0] & LED_CMD_EXECUTE  # 0 = Queue, 0x10 = Execute
        led_id = packet[0] & LED_ID_MASK  # LED ID (0-15)

        # Extract RGB and time values
        r = (packet[1] >> 4) & 0x0F
        g = packet[1] & 0x0F
        b = (packet[2] >> 4) & 0x0F
        time_value = packet[2] & 0x0F

        if self.debug:
            print(
                f"LED packet: cmd={hex(cmd_type)}, id={led_id}, RGB=({r},{g},{b}), time={time_value}"
            )

        if cmd_type == LED_CMD_QUEUE:
            # Queue this color in the sequence for this LED
            self.led_queues[led_id].append((r, g, b, time_value))
            if self.debug:
                print(
                    f"Queued color for LED {led_id}, queue length: {len(self.led_queues[led_id])}"
                )

        elif cmd_type == LED_CMD_EXECUTE:
            # Handle execute command
            sequence_length = len(self.led_queues[led_id])
            if sequence_length > 0:
                if self.debug:
                    print(
                        f"Executing sequence for LED {led_id} with {sequence_length} steps"
                    )
                # Process LED sequence (implementation depends on hardware)
                # ...
                # Clear the queue after execution
                self.led_queues[led_id] = []

            # Send completion notification
            self.send_led_completion(led_id, sequence_length)

    def _process_power_packet(self, packet):
        """Process power management packet"""
        command = packet[0] & 0x70  # Extract command bits
        param = packet[0] & 0x0F  # Extract parameter bits
        data1 = packet[1]  # Data value
        data2 = packet[2]  # Additional parameter

        if self.debug:
            print(
                f"Power packet: cmd={hex(command)}, param={hex(param)}, data=[{hex(data1)},{hex(data2)}]"
            )

        if command == POWER_CMD_QUERY:
            # Respond with current power state
            if self.debug:
                print(f"Responding to power query with state: {self.power_state}")
            self.send_power_query_response(self.power_state)

        elif command == POWER_CMD_SET:
            if data1 == POWER_STATE_RUNNING:
                if self.debug:
                    print("Received boot notification from Linux host")
                # Update state
                self.power_state = POWER_STATE_RUNNING
                self.linux_booted = True

                # Send acknowledgment
                self.send_packet(TYPE_POWER | POWER_CMD_SET, POWER_STATE_RUNNING, 0x00)

                # Debug code: Boot notification received
                self.send_debug_code(
                    DEBUG_CAT_POWER, POWER_CMD_SET, POWER_STATE_RUNNING
                )

        elif command == POWER_CMD_SLEEP:
            # Host is entering sleep mode
            if self.debug:
                print(f"Host entering sleep mode: delay={data1}")
            self.power_state = POWER_STATE_SUSPEND

            # Send acknowledgment
            self.send_packet(TYPE_POWER | POWER_CMD_SLEEP, data1, 0x01)  # ACK

        elif command == POWER_CMD_SHUTDOWN:
            # Shutdown sequence
            if self.debug:
                print(f"Processing shutdown command: mode={data1}")

            # Update state
            self.power_state = POWER_STATE_OFF
            self.linux_booted = False

            # Send acknowledgment
            self.send_packet(TYPE_POWER | POWER_CMD_SHUTDOWN, data1, 0x01)  # ACK

            # Debug code: Shutdown notification received
            self.send_debug_code(DEBUG_CAT_POWER, POWER_CMD_SHUTDOWN, data1)

        elif command == POWER_CMD_REQUEST_METRICS:
            # Linux is requesting power metrics - handler in main.py will respond
            if self.debug:
                print("Received request for power metrics")

    def _process_system_packet(self, packet):
        """Process system command packet"""
        action = packet[0] & 0x1F
        data1 = packet[1]
        data2 = packet[2]

        if self.debug:
            print(
                f"System packet: action={hex(action)}, data=[{hex(data1)},{hex(data2)}]"
            )

        if action == SYSTEM_PING:
            # Respond with a ping acknowledgment
            self.send_ping_response()

        elif action == SYSTEM_RESET:
            # Perform a system reset (implementation depends on hardware)
            self.send_debug_code(DEBUG_CAT_SYSTEM, 0x02, 0x00)  # System resetting
            # Reset logic would go here

        elif action == SYSTEM_VERSION:
            # Receive version information from host
            if not packet[1] == 0 or not packet[2] == 0:
                # This is incoming version information from host
                self.host_version["major"] = data1
                self.host_version["minor"] = data2
                self.host_version["string"] = (
                    f"{data1}.{data2}.{self.host_version['patch']}"
                )
                if self.debug:
                    print(f"Received host version: {self.host_version['string']}")

                # Acknowledge receipt
                self.send_system_command(SYSTEM_VERSION, data1, data2)
            else:
                # This is a version request from host, respond with our version
                self.send_version_info(VERSION_MAJOR, VERSION_MINOR)

        elif action == SYSTEM_STATUS:
            # Send system status response
            self.send_packet(TYPE_SYSTEM | SYSTEM_STATUS, 0x01, 0x00)  # Status OK

        elif action == SYSTEM_CONFIG:
            # Handle configuration request/command
            if data1 == 0x00:  # Get config
                self.send_packet(
                    TYPE_SYSTEM | SYSTEM_CONFIG, data2, 0x01
                )  # Example value
            elif data1 == 0x01:  # Set config
                self.send_packet(TYPE_SYSTEM | SYSTEM_CONFIG, 0x01, data2)  # Confirm

    def process_extended_packet(self, packet):
        """Process extended command packet (for future expansion)

        Args:
            packet: Received packet
        """
        extended_cmd = packet[0] & 0x1F
        data1 = packet[1]
        _ = packet[2]

        if self.debug:
            print(f"Extended packet received: {[hex(b) for b in packet]}")

        if extended_cmd == 0x01:  # Extended version info
            # Store patch version
            self.host_version["patch"] = data1
            # Update complete version string
            self.host_version["string"] = (
                f"{self.host_version['major']}.{self.host_version['minor']}.{data1}"
            )

            if self.debug:
                print(f"Updated host version: {self.host_version['string']}")

            # Send acknowledgment
            self.send_packet(
                TYPE_EXTENDED | extended_cmd, data1, 0x01
            )  # ACK with original data
        else:
            # Send a generic acknowledgment for now
            self.send_packet(TYPE_EXTENDED | (packet[0] & 0x1F), 0x00, 0x00)

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
                # Store byte in buffer
                if self.rx_pos < PACKET_SIZE:
                    self.rx_buffer[self.rx_pos] = byte
                    self.rx_pos += 1

                # Process packet when complete
                if self.rx_pos == PACKET_SIZE:
                    if self.process_packet(self.rx_buffer):
                        packets_processed += 1
                    self.rx_pos = 0

                # Feed WDT during extended processing
                if packets_processed > 0 and packets_processed % 10 == 0:
                    self._feed_wdt()

        return packets_processed
