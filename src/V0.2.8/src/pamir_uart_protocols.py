"""Pamir SAM UART Protocol implementation for RP2040"""

import struct


class PamirUartProtocols:
    """Pamir UART Protocols for RP2040"""

    # RP2040 Firmware version constants for protocol negotiation
    FIRMWARE_VERSION_MAJOR = 0
    FIRMWARE_VERSION_MINOR = 2
    FIRMWARE_VERSION_PATCH = 3

    # Message type constants (3 MSB of type_flags)
    TYPE_BUTTON = 0x00  # 0b000xxxxx - Button state change events
    TYPE_LED = 0x20  # 0b001xxxxx - LED control commands and status
    TYPE_POWER = 0x40  # 0b010xxxxx - Power management and metrics
    TYPE_DISPLAY = 0x60  # 0b011xxxxx - E-ink display control and status
    TYPE_DEBUG_CODE = 0x80  # 0b100xxxxx - Numeric debug codes
    TYPE_DEBUG_TEXT = 0xA0  # 0b101xxxxx - Text debug messages
    TYPE_SYSTEM = 0xC0  # 0b110xxxxx - Core system control commands
    TYPE_EXTENDED = 0xE0  # 0b111xxxxx - Extended commands

    # Button bit masks
    BTN_UP = 0x01  # Bit 0
    BTN_DOWN = 0x02  # Bit 1
    BTN_SELECT = 0x04  # Bit 2
    BTN_POWER = 0x08  # Bit 3

    # LED command flags (5 LSB of type_flags)
    LED_CMD_QUEUE = 0x00  # Queue instruction (E=0)
    LED_CMD_EXECUTE = 0x10  # Execute sequence (E=1)

    # LED modes/commands (2-bit values)
    LED_MODE_STATIC = 0x00  # Static color
    LED_MODE_BLINK = 0x01  # Blinking
    LED_MODE_FADE = 0x02  # Fade in/out
    LED_MODE_RAINBOW = 0x03  # Rainbow cycle

    # Special LED IDs
    LED_ALL = 0x0F  # Broadcast to all LEDs (ID 15)

    # Power command constants 
    # Control commands (0x00-0x0F)
    POWER_CMD_QUERY = 0x00  # Query current power status
    POWER_CMD_SET = 0x01  # Set power state
    POWER_CMD_SLEEP = 0x02  # Enter sleep mode
    POWER_CMD_SHUTDOWN = 0x03  # Shutdown system
    
    # Reporting commands (0x10-0x1F)
    POWER_CMD_CURRENT = 0x10  # Current draw reporting
    POWER_CMD_BATTERY = 0x11  # Battery state reporting
    POWER_CMD_TEMP = 0x12  # Temperature reporting
    POWER_CMD_VOLTAGE = 0x13  # Voltage reporting
    POWER_CMD_REQUEST_METRICS = 0x1F  # Request all metrics

    # Power states
    POWER_STATE_OFF = 0x00  # System off
    POWER_STATE_RUNNING = 0x01  # System running
    POWER_STATE_SUSPEND = 0x02  # System suspended
    POWER_STATE_SLEEP = 0x03  # Low power sleep

    def __init__(self):
        pass

    def calculate_crc8(self, data):
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
    
    def calculate_checksum(self, type_flags, data0, data1):
        """Calculate CRC8 checksum for packet"""
        return self.calculate_crc8([type_flags, data0, data1])

    def create_packet(self, type_flags, data0=0x00, data1=0x00):
        """Create a 4-byte protocol packet with CRC8 checksum"""
        checksum = self.calculate_checksum(type_flags, data0, data1)
        return struct.pack("BBBB", type_flags, data0, data1, checksum)

    def validate_packet(self, packet_bytes):
        """Validate packet checksum and return parsed data"""
        if len(packet_bytes) != 4:
            return False, None

        type_flags, data0, data1, checksum = struct.unpack("BBBB", packet_bytes)
        calculated_checksum = self.calculate_checksum(type_flags, data0, data1)

        if checksum != calculated_checksum:
            return False, None

        return True, (type_flags, data0, data1, checksum)

    def create_button_packet(
        self,
        up_pressed=False,
        down_pressed=False,
        select_pressed=False,
        power_pressed=False,
    ):
        """Create button event packet according to protocol specification

        Args:
            up_pressed: UP button state (True=pressed, False=released)
            down_pressed: DOWN button state (True=pressed, False=released)
            select_pressed: SELECT button state (True=pressed, False=released)
            power_pressed: POWER button state (True=pressed, False=released)

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        # Start with TYPE_BUTTON (0b000xxxxx)
        type_flags = self.TYPE_BUTTON

        # Set button state bits in the 5 LSB
        if up_pressed:
            type_flags |= self.BTN_UP
        if down_pressed:
            type_flags |= self.BTN_DOWN
        if select_pressed:
            type_flags |= self.BTN_SELECT
        if power_pressed:
            type_flags |= self.BTN_POWER

        # Data bytes are reserved (set to 0x00 as per spec)
        return self.create_packet(type_flags, 0x00, 0x00)

    def parse_button_packet(self, packet_bytes):
        """Parse button packet and return button states

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, button_states) where button_states is dict with button states
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

        # Check if this is a button packet
        if (type_flags & 0xE0) != self.TYPE_BUTTON:
            return False, None

        # Extract button states from the 5 LSB
        button_states = {
            "up": bool(type_flags & self.BTN_UP),
            "down": bool(type_flags & self.BTN_DOWN),
            "select": bool(type_flags & self.BTN_SELECT),
            "power": bool(type_flags & self.BTN_POWER),
        }

        return True, button_states

    def create_led_packet(
        self, led_id=0, execute=False, mode=0, r4=0, g4=0, b4=0, time_value=0
    ):
        """Create LED control packet according to protocol specification

        Args:
            led_id: LED identifier (0-15, 15 for all LEDs)
            execute: True to execute sequence, False to queue
            mode: LED mode (static, blink, fade, rainbow, sequence)
            r4: Red component (4-bit, 0-15)
            g4: Green component (4-bit, 0-15)
            b4: Blue component (4-bit, 0-15)
            time_value: Time/speed parameter (4-bit, 0-15)

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        # format: type_flags = [001][E][LED_ID]
        type_flags = self.TYPE_LED

        # Add execute flag (bit 4)
        if execute:
            type_flags |= self.LED_CMD_EXECUTE

        # Add LED ID (bits 3-0)
        type_flags |= led_id & 0x0F

        # Pack color data: data[0] = RRRRGGGG, data[1] = BBBBMMTT
        data0 = ((r4 & 0x0F) << 4) | (g4 & 0x0F)
        data1 = ((b4 & 0x0F) << 4) | ((mode & 0x03) << 2) | (time_value & 0x03)

        return self.create_packet(type_flags, data0, data1)

    def create_led_completion_packet(self, led_id, sequence_length):
        """Create LED completion acknowledgment packet according to protocol specification

        Args:
            led_id: LED identifier that completed (0-15)
            sequence_length: Number of commands executed (0-255)

        Returns:
            bytes: 4-byte acknowledgment packet ready for UART transmission
        """
        # Start with TYPE_LED | LED_CMD_EXECUTE | LED_ID
        type_flags = self.TYPE_LED | self.LED_CMD_EXECUTE | (led_id & 0x0F)

        # data[0] = 0xFF (completion indicator as per spec)
        # data[1] = sequence length that was executed
        data0 = 0xFF
        data1 = min(255, max(0, sequence_length))  # Clamp to byte range

        return self.create_packet(type_flags, data0, data1)

    def create_led_error_packet(self, led_id, error_code):
        """Create LED error report packet

        Args:
            led_id: LED identifier that had error (0-15)
            error_code: Error code (1-255)

        Returns:
            bytes: 4-byte error packet ready for UART transmission
        """
        # Start with TYPE_LED | LED_CMD_EXECUTE | LED_ID
        type_flags = self.TYPE_LED | self.LED_CMD_EXECUTE | (led_id & 0x0F)

        # data[0] = 0xFE (error indicator)
        # data[1] = error code
        data0 = 0xFE
        data1 = min(255, max(1, error_code))  # Error codes 1-255

        return self.create_packet(type_flags, data0, data1)

    def create_led_status_packet(self, led_id, status_code, status_value=0):
        """Create LED status report packet

        Args:
            led_id: LED identifier (0-15)
            status_code: Status type code
            status_value: Optional status value

        Returns:
            bytes: 4-byte status packet ready for UART transmission
        """
        # Start with TYPE_LED | LED_CMD_EXECUTE | LED_ID
        type_flags = self.TYPE_LED | self.LED_CMD_EXECUTE | (led_id & 0x0F)

        # data[0] = status code (0x00-0xFD, avoiding 0xFE=error, 0xFF=completion)
        # data[1] = status value
        data0 = min(0xFD, max(0, status_code))
        data1 = status_value & 0xFF

        return self.create_packet(type_flags, data0, data1)

    def parse_led_packet(self, packet_bytes):
        """Parse LED packet and return LED command data

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, led_data) where led_data is dict with LED command info
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

        # Check if this is an LED packet
        if (type_flags & 0xE0) != self.TYPE_LED:
            return False, None

        # Extract LED command fields
        execute = bool(type_flags & self.LED_CMD_EXECUTE)
        led_id = type_flags & 0x0F  # LED ID is in bits 3-0 (0-15)
        
        # Extract RGB values from data[0]
        r4 = (data0 >> 4) & 0x0F  # Red in bits 7-4
        g4 = data0 & 0x0F         # Green in bits 3-0
        
        # Extract blue, mode, and timing from data[1]
        b4 = (data1 >> 4) & 0x0F  # Blue in bits 7-4
        led_mode = (data1 >> 2) & 0x03  # Mode in bits 3-2
        time_value = data1 & 0x03  # Timing in bits 1-0

        # Convert timing value to delay_ms
        timing_map = {0: 100, 1: 200, 2: 500, 3: 1000}
        delay_ms = timing_map.get(time_value, 100)

        led_data = {
            "execute": execute,
            "led_id": led_id,
            "led_mode": led_mode,
            "color": (r4, g4, b4),
            "time_value": time_value,
            "delay_ms": delay_ms,
        }

        return True, led_data

    def parse_led_acknowledgment(self, packet_bytes):
        """Parse LED acknowledgment/status packet

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, ack_data) where ack_data contains acknowledgment info
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

        # Check if this is an LED packet with execute flag
        if (type_flags & 0xE0) != self.TYPE_LED or not (
            type_flags & self.LED_CMD_EXECUTE
        ):
            return False, None

        led_id = type_flags & 0x0F

        # Determine acknowledgment type based on data[0]
        if data0 == 0xFF:
            # Completion acknowledgment
            ack_type = "completion"
            sequence_length = data1
            ack_data = {
                "type": ack_type,
                "led_id": led_id,
                "sequence_length": sequence_length,
            }
        elif data0 == 0xFE:
            # Error report
            ack_type = "error"
            error_code = data1
            ack_data = {"type": ack_type, "led_id": led_id, "error_code": error_code}
        else:
            # Status report
            ack_type = "status"
            status_code = data0
            status_value = data1
            ack_data = {
                "type": ack_type,
                "led_id": led_id,
                "status_code": status_code,
                "status_value": status_value,
            }

        return True, ack_data

    # ==================== POWER MANAGEMENT PROTOCOL ====================

    def create_power_packet_som_to_rp2040(self, command, data0=0x00, data1=0x00):
        """Create power command packet FROM SoM (Raspberry Pi 5) TO RP2040

        Commands sent by SoM to control RP2040 power management:
        - POWER_CMD_QUERY (0x00): Query current power status
        - POWER_CMD_SET (0x10): Set power state (running/sleep/suspend)
        - POWER_CMD_SLEEP (0x20): Enter sleep mode with optional delay
        - POWER_CMD_SHUTDOWN (0x30): Prepare for system shutdown
        - POWER_CMD_REQUEST_METRICS (0x80): Request all sensor metrics

        Args:
            command: Power command type (see POWER_CMD_* constants)
            data0: First data byte (command-specific)
            data1: Second data byte (command-specific)

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        # Start with TYPE_POWER (0b010xxxxx) + command in 5 LSB
        type_flags = self.TYPE_POWER | (command & 0x1F)
        return self.create_packet(type_flags, data0, data1)

    def create_power_metrics_packet_rp2040_to_som(self, metric_type, value_16bit):
        """Create power metrics packet FROM RP2040 TO SoM (Raspberry Pi 5)

        Metrics sent by RP2040 to report sensor data to SoM:
        - POWER_CMD_CURRENT (0x40): Current draw in mA
        - POWER_CMD_BATTERY (0x50): Battery percentage (0-100%)
        - POWER_CMD_TEMP (0x60): Temperature in 0.1°C resolution
        - POWER_CMD_VOLTAGE (0x70): Voltage in mV

        Args:
            metric_type: Metric type (POWER_CMD_CURRENT/BATTERY/TEMP/VOLTAGE)
            value_16bit: 16-bit sensor value (little-endian format)

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        # Start with TYPE_POWER + metric type
        type_flags = self.TYPE_POWER | (metric_type & 0x1F)

        # Pack 16-bit value as little-endian (low byte first)
        data0 = value_16bit & 0xFF  # Low byte
        data1 = (value_16bit >> 8) & 0xFF  # High byte

        return self.create_packet(type_flags, data0, data1)

    def create_power_status_packet_rp2040_to_som(self, power_state, status_flags=0x00):
        """Create power status packet FROM RP2040 TO SoM (Raspberry Pi 5)

        Status responses sent by RP2040 in response to SoM queries:
        - Boot notification: RP2040 has started and is running
        - Shutdown acknowledgment: RP2040 is ready for power-off
        - Power state changes: Current operating mode

        Args:
            power_state: Current power state (POWER_STATE_* constants)
            status_flags: Optional status flags

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        type_flags = self.TYPE_POWER | self.POWER_CMD_QUERY
        return self.create_packet(type_flags, power_state, status_flags)

    def parse_power_packet(self, packet_bytes):
        """Parse power packet and return power command data

        Handles both SoM→RP2040 commands and RP2040→SoM responses

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, power_data) where power_data contains command info
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

       # Check if this is a power packet
        if (type_flags & 0xE0) == self.TYPE_POWER:
            # Extract power command from 5 LSB (supports both control and reporting commands)
            command = type_flags & 0x1F
        else:
            return False, None

        # Parse based on command type
        if command == self.POWER_CMD_QUERY:
            # Status query or response
            power_data = {
                "command": "query",
                "power_state": data0,
                "status_flags": data1,
            }
        elif command == self.POWER_CMD_SET:
            # Set power state command (SoM → RP2040)
            power_data = {"command": "set_state", "power_state": data0, "flags": data1}
        elif command == self.POWER_CMD_SLEEP:
            # Enter sleep mode command (SoM → RP2040)
            power_data = {
                "command": "sleep",
                "delay_seconds": data0,
                "sleep_flags": data1,
            }
        elif command == self.POWER_CMD_SHUTDOWN:
            # Shutdown command (SoM → RP2040)
            power_data = {
                "command": "shutdown",
                "shutdown_mode": data0,  # 0=normal, 1=emergency, 2=reboot
                "reason_code": data1,
            }
        elif command == self.POWER_CMD_REQUEST_METRICS:
            # Metrics request (SoM → RP2040)
            power_data = {
                "command": "request_metrics",
                "metric_mask": data0,  # Which metrics to send (0=all)
                "reserved": data1,
            }
        elif command in [
            self.POWER_CMD_CURRENT,
            self.POWER_CMD_BATTERY,
            self.POWER_CMD_TEMP,
            self.POWER_CMD_VOLTAGE,
        ]:
            # Metrics response (RP2040 → SoM)
            value_16bit = data0 | (data1 << 8)  # Little-endian reconstruction

            metric_names = {
                self.POWER_CMD_CURRENT: "current_ma",
                self.POWER_CMD_BATTERY: "battery_percent",
                self.POWER_CMD_TEMP: "temperature_0_1c",
                self.POWER_CMD_VOLTAGE: "voltage_mv",
            }

            power_data = {
                "command": "metrics_response",
                "metric_type": metric_names.get(command, "unknown"),
                "value": value_16bit,
            }
        else:
            # Unknown power command
            power_data = {
                "command": "unknown",
                "raw_command": command,
                "data0": data0,
                "data1": data1,
            }

        return True, power_data

    def get_packet_type(self, packet_bytes):
        """Get the message type from a packet

        Args:
            packet_bytes: 4-byte packet

        Returns:
            int: Message type (3 MSB of type_flags) or None if invalid
        """
        if len(packet_bytes) != 4:
            return None

        type_flags = packet_bytes[0]
        return type_flags & 0xE0

    # ==================== SYSTEM COMMANDS PROTOCOL ====================

    def create_system_ping_packet(self):
        """Create system ping packet FROM RP2040 TO SoM

        Returns:
            bytes: 4-byte ping packet ready for UART transmission
        """
        return self.create_packet(0xC0, 0x00, 0x00)

    def create_system_pong_packet(self):
        """Create system pong response packet FROM RP2040 TO SoM

        Returns:
            bytes: 4-byte pong packet ready for UART transmission
        """
        return self.create_packet(0xC0, 0x01, 0x00)

    def create_firmware_version_packet(self):
        """Create firmware version packet FROM RP2040 TO SoM

        Returns:
            bytes: 4-byte version packet ready for UART transmission
        """
        # Pack version as: MAJOR.MINOR.PATCH into 2 bytes
        # data[0] = MAJOR (8 bits), data[1] = MINOR (4 bits) | PATCH (4 bits)
        version_data0 = self.FIRMWARE_VERSION_MAJOR & 0xFF
        version_data1 = ((self.FIRMWARE_VERSION_MINOR & 0x0F) << 4) | (
            self.FIRMWARE_VERSION_PATCH & 0x0F
        )
        return self.create_packet(0xC2, version_data0, version_data1)

    def parse_system_packet(self, packet_bytes):
        """Parse system packet and return system command data

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, system_data) where system_data contains command info
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

        # Check if this is a system packet
        if (type_flags & 0xE0) != 0xC0:
            return False, None

        # Extract system command from full type_flags
        command = type_flags & 0x1F

        if command == 0x00:
            # Ping command
            system_data = {"command": "ping"}
        elif command == 0x01:
            # Pong response
            system_data = {"command": "pong"}
        elif command == 0x02:
            # Version request
            system_data = {"command": "version_request"}
        elif command == 0x03:
            # Reset command
            system_data = {"command": "reset", "reset_type": data0}
        else:
            # Unknown system command
            system_data = {
                "command": "unknown",
                "raw_command": command,
                "data0": data0,
                "data1": data1,
            }

        return True, system_data

    # ==================== DISPLAY CONTROL PROTOCOL ====================

    def create_display_status_packet(self, status_code, data_value=0x00):
        """Create display status packet FROM RP2040 TO SoM

        Args:
            status_code: Display status code
            data_value: Additional status data

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        type_flags = self.TYPE_DISPLAY | 0x01  # Display status report
        return self.create_packet(type_flags, status_code, data_value)

    def create_display_completion_packet(self):
        """Create display refresh completion packet FROM RP2040 TO SoM

        Returns:
            bytes: 4-byte packet ready for UART transmission
        """
        type_flags = self.TYPE_DISPLAY | 0x01  # Display refresh completion
        return self.create_packet(type_flags, 0xFF, 0x00)  # 0xFF = completion indicator

    def parse_display_packet(self, packet_bytes):
        """Parse display packet and return display command data

        Args:
            packet_bytes: 4-byte packet from UART

        Returns:
            tuple: (valid, display_data) where display_data contains command info
        """
        valid, parsed = self.validate_packet(packet_bytes)
        if not valid:
            return False, None

        type_flags, data0, data1, checksum = parsed

        # Check if this is a display packet
        if (type_flags & 0xE0) != self.TYPE_DISPLAY:
            return False, None

        # Extract display command from lower 5 bits
        command = type_flags & 0x1F

        if command == 0x07:
            # Display release command from SoM
            if data0 == 0xFF:
                display_data = {"command": "release", "signal": data0}
            else:
                display_data = {"command": "unknown_release", "data0": data0, "data1": data1}
        elif command == 0x01:
            # Display status or completion
            if data0 == 0xFF:
                display_data = {"command": "completion", "data1": data1}
            else:
                display_data = {"command": "status", "status_code": data0, "data1": data1}
        else:
            # Unknown display command
            display_data = {
                "command": "unknown",
                "raw_command": command,
                "data0": data0,
                "data1": data1,
            }

        return True, display_data
