"""UART packet handler with frame synchronization and robust error recovery"""

# pylint: disable=import-error,broad-exception-caught
import utime


class UartHandler:
    """UART packet handler with robust frame synchronization and error recovery"""

    # Buffer constants
    MAX_BUFFER_SIZE = 1024  # Maximum buffer size before forced flush
    PACKET_SIZE = 4
    SYNC_SEARCH_LIMIT = 64  # Maximum bytes to search for sync

    # Statistics tracking
    STATS_RESET_INTERVAL = 30000  # Reset stats every 30 seconds

    def __init__(self, uart, protocol, debug_handler=None):
        """Initialize enhanced UART handler

        Args:
            uart: UART interface object
            protocol: PamirUartProtocols instance
            debug_handler: Optional debug handler for logging
        """
        self.uart = uart
        self.protocol = protocol
        self.debug = debug_handler

        # Circular buffer for incoming data
        self.rx_buffer = bytearray(self.MAX_BUFFER_SIZE)
        self.buffer_head = 0  # Write position
        self.buffer_tail = 0  # Read position
        self.buffer_count = 0  # Current buffer size

        # Frame synchronization state
        self.sync_state = "SEARCHING"  # SEARCHING, SYNCED, RECOVERING
        self.consecutive_valid = 0
        self.consecutive_invalid = 0

        # Statistics
        self.stats = {
            "total_bytes_received": 0,
            "packets_processed": 0,
            "valid_packets": 0,
            "invalid_packets": 0,
            "sync_losses": 0,
            "buffer_overflows": 0,
            "last_reset": utime.ticks_ms(),
        }

        # Error recovery
        self.recovery_in_progress = False
        self.last_valid_packet_time = utime.ticks_ms()

    def _debug_log(self, level, message):
        """Log debug message if debug handler available"""
        if self.debug:
            self.debug.log(level, "UART", f"[UartHandler] {message}")

    def _add_to_buffer(self, data):
        """Add data to circular buffer with overflow protection

        Args:
            data: bytes to add to buffer

        Returns:
            int: Number of bytes actually added
        """
        if not data:
            return 0

        bytes_added = 0
        for byte in data:
            # Check for buffer overflow
            if self.buffer_count >= self.MAX_BUFFER_SIZE:
                self.stats["buffer_overflows"] += 1
                self._debug_log(1, "Buffer overflow! Forcing flush.")
                self._force_buffer_flush()

            # Add byte to circular buffer
            self.rx_buffer[self.buffer_head] = byte
            self.buffer_head = (self.buffer_head + 1) % self.MAX_BUFFER_SIZE
            self.buffer_count += 1
            bytes_added += 1

        self.stats["total_bytes_received"] += bytes_added
        return bytes_added

    def _get_from_buffer(self, num_bytes):
        """Get bytes from circular buffer without removing them

        Args:
            num_bytes: Number of bytes to peek

        Returns:
            bytearray: Requested bytes (may be shorter if not enough available)
        """
        num_bytes = min(num_bytes, self.buffer_count)

        result = bytearray(num_bytes)
        pos = self.buffer_tail

        for i in range(num_bytes):
            result[i] = self.rx_buffer[pos]
            pos = (pos + 1) % self.MAX_BUFFER_SIZE

        return result

    def _consume_from_buffer(self, num_bytes):
        """Remove bytes from front of circular buffer

        Args:
            num_bytes: Number of bytes to remove
        """
        num_bytes = min(num_bytes, self.buffer_count)

        self.buffer_tail = (self.buffer_tail + num_bytes) % self.MAX_BUFFER_SIZE
        self.buffer_count -= num_bytes

    def _force_buffer_flush(self):
        """Force flush the buffer in case of critical errors"""
        self.buffer_head = 0
        self.buffer_tail = 0
        self.buffer_count = 0
        self.sync_state = "SEARCHING"
        self.consecutive_valid = 0
        self.consecutive_invalid = 0
        self._debug_log(2, "Buffer forcibly flushed")

    def _find_packet_boundary(self):
        """Search for valid packet boundary using frame synchronization

        Returns:
            int: Offset to potential packet start, or -1 if not found
        """
        if self.buffer_count < self.PACKET_SIZE:
            return -1

        search_limit = min(
            self.SYNC_SEARCH_LIMIT, self.buffer_count - self.PACKET_SIZE + 1
        )

        for offset in range(search_limit):
            # Extract potential 4-byte packet
            packet_data = self._get_from_buffer_at_offset(offset, self.PACKET_SIZE)

            # Quick validation without expensive logging
            if self._is_valid_packet_fast(packet_data):
                if offset > 0:
                    self._debug_log(2, f"Found packet boundary at offset {offset}")
                return offset

        return -1

    def _get_from_buffer_at_offset(self, offset, num_bytes):
        """Get bytes from buffer at specific offset

        Args:
            offset: Offset from buffer tail
            num_bytes: Number of bytes to get

        Returns:
            bytearray: Requested bytes
        """
        if offset + num_bytes > self.buffer_count:
            return bytearray()

        result = bytearray(num_bytes)
        pos = (self.buffer_tail + offset) % self.MAX_BUFFER_SIZE

        for i in range(num_bytes):
            result[i] = self.rx_buffer[pos]
            pos = (pos + 1) % self.MAX_BUFFER_SIZE

        return result

    def _is_valid_packet_fast(self, packet_data):
        """Fast packet validation without full parsing

        Args:
            packet_data: 4-byte packet data

        Returns:
            bool: True if packet appears valid
        """
        if len(packet_data) != 4:
            return False

        # CRC8 checksum validation using protocol's CRC8 algorithm
        calculated_checksum = self.protocol.calculate_crc8(packet_data[:3])
        return calculated_checksum == packet_data[3]

    def _handle_sync_loss(self):
        """Handle loss of frame synchronization"""
        self.stats["sync_losses"] += 1
        self.sync_state = "RECOVERING"
        self.consecutive_valid = 0
        self.consecutive_invalid += 1

        self._debug_log(
            2, f"Sync loss detected. Invalid count: {self.consecutive_invalid}"
        )

        # If too many consecutive invalid packets, force resync
        if self.consecutive_invalid >= 3:
            self._debug_log(1, "Multiple invalid packets. Forcing resync.")
            self._force_resync()

    def _force_resync(self):
        """Force frame resynchronization by searching for valid packets"""
        # Try to find a valid packet boundary
        boundary_offset = self._find_packet_boundary()

        if boundary_offset >= 0:
            # Found boundary - discard bytes before it
            if boundary_offset > 0:
                self._consume_from_buffer(boundary_offset)
                self._debug_log(2, f"Resynced by discarding {boundary_offset} bytes")

            self.sync_state = "SYNCED"
            self.consecutive_invalid = 0
        else:
            # No valid boundary found - discard some bytes and keep searching
            discard_count = min(16, self.buffer_count // 2)
            if discard_count > 0:
                self._consume_from_buffer(discard_count)
                self._debug_log(2, f"Discarded {discard_count} bytes while resyncing")

            self.sync_state = "SEARCHING"

    def _update_sync_state(self, packet_valid):
        """Update frame synchronization state based on packet validity

        Args:
            packet_valid: Whether the last packet was valid
        """
        if packet_valid:
            self.consecutive_valid += 1
            self.consecutive_invalid = 0
            self.last_valid_packet_time = utime.ticks_ms()

            # If we were recovering and got a valid packet, consider synced
            if self.sync_state == "RECOVERING" and self.consecutive_valid >= 2:
                self.sync_state = "SYNCED"
                self._debug_log(2, "Frame sync recovered")

        else:
            self.consecutive_valid = 0
            self.consecutive_invalid += 1

            # Handle sync loss
            if self.sync_state == "SYNCED":
                self._handle_sync_loss()

    def receive_data(self):
        """Receive and buffer new UART data

        Returns:
            int: Number of bytes received
        """
        if not self.uart.any():
            return 0

        try:
            data = self.uart.read()
            if data:
                bytes_added = self._add_to_buffer(data)
                self._debug_log(
                    3, f"Received {len(data)} bytes, buffered {bytes_added}"
                )
                return bytes_added
        except Exception as e:
            self._debug_log(1, f"UART read error: {e}")

        return 0

    def process_packets(self):
        """Process any complete packets in the buffer

        Returns:
            list: List of (valid, packet_data) tuples for processed packets
        """
        processed_packets = []

        # Process all available complete packets
        while self.buffer_count >= self.PACKET_SIZE:
            # Handle different sync states
            if self.sync_state == "SEARCHING":
                # Look for packet boundary
                boundary_offset = self._find_packet_boundary()

                if boundary_offset >= 0:
                    # Found boundary - discard bytes before it
                    if boundary_offset > 0:
                        self._consume_from_buffer(boundary_offset)

                    self.sync_state = "SYNCED"
                    self._debug_log(2, "Frame sync established")
                    continue  # Process the packet in next iteration

                # No valid packet found - discard some bytes
                discard_count = min(
                    self.PACKET_SIZE, self.buffer_count - self.PACKET_SIZE + 1
                )
                self._consume_from_buffer(discard_count)
                continue

            # Extract packet (either SYNCED or RECOVERING state)
            packet_data = self._get_from_buffer(self.PACKET_SIZE)

            # Validate packet
            valid, _ = self.protocol.validate_packet(packet_data)

            # Update statistics
            self.stats["packets_processed"] += 1
            if valid:
                self.stats["valid_packets"] += 1
            else:
                self.stats["invalid_packets"] += 1

            # Update sync state
            self._update_sync_state(valid)

            # Add to results
            processed_packets.append((valid, packet_data if valid else None))

            # Consume the packet from buffer
            self._consume_from_buffer(self.PACKET_SIZE)

            # Log packet processing (minimal overhead)
            if valid:
                self._debug_log(3, "✓ Valid packet processed")
            else:
                self._debug_log(2, f"✗ Invalid packet: {packet_data.hex()}")

                # If in synced state and got invalid packet, may need recovery
                if self.sync_state == "SYNCED":
                    continue  # Let _update_sync_state handle the transition

        return processed_packets

    def get_statistics(self):
        """Get current handler statistics

        Returns:
            dict: Statistics information
        """
        current_time = utime.ticks_ms()

        # Calculate success rate
        total_packets = self.stats["valid_packets"] + self.stats["invalid_packets"]
        success_rate = (
            (self.stats["valid_packets"] * 100 // total_packets)
            if total_packets > 0
            else 0
        )

        # Time since last valid packet
        time_since_valid = utime.ticks_diff(current_time, self.last_valid_packet_time)

        stats = self.stats.copy()
        stats.update(
            {
                "success_rate_percent": success_rate,
                "buffer_usage_percent": (self.buffer_count * 100)
                // self.MAX_BUFFER_SIZE,
                "sync_state": self.sync_state,
                "consecutive_valid": self.consecutive_valid,
                "consecutive_invalid": self.consecutive_invalid,
                "time_since_valid_ms": time_since_valid,
                "current_buffer_size": self.buffer_count,
            }
        )

        return stats

    def reset_statistics(self):
        """Reset statistics counters"""
        self.stats = {
            "total_bytes_received": 0,
            "packets_processed": 0,
            "valid_packets": 0,
            "invalid_packets": 0,
            "sync_losses": 0,
            "buffer_overflows": 0,
            "last_reset": utime.ticks_ms(),
        }
        self._debug_log(2, "Statistics reset")

    def check_health(self):
        """Check handler health and perform maintenance

        Returns:
            dict: Health status information
        """
        current_time = utime.ticks_ms()
        time_since_valid = utime.ticks_diff(current_time, self.last_valid_packet_time)

        health_status = {"status": "HEALTHY", "issues": []}

        # Check for stalled communication
        if time_since_valid > 5000:  # 5 seconds without valid packet
            health_status["status"] = "DEGRADED"
            health_status["issues"].append("No valid packets received recently")

        # Check buffer health
        buffer_usage = (self.buffer_count * 100) // self.MAX_BUFFER_SIZE
        if buffer_usage > 80:
            health_status["status"] = "DEGRADED"
            health_status["issues"].append(f"High buffer usage: {buffer_usage}%")

        # Check sync state
        if self.sync_state != "SYNCED" and time_since_valid > 1000:
            health_status["status"] = "DEGRADED"
            health_status["issues"].append(f"Poor sync state: {self.sync_state}")

        # Check error rates
        total_packets = self.stats["valid_packets"] + self.stats["invalid_packets"]
        if total_packets > 10:
            error_rate = (self.stats["invalid_packets"] * 100) // total_packets
            if error_rate > 20:  # More than 20% error rate
                health_status["status"] = "DEGRADED"
                health_status["issues"].append(f"High error rate: {error_rate}%")

        # Auto-reset statistics periodically
        if (
            utime.ticks_diff(current_time, self.stats["last_reset"])
            > self.STATS_RESET_INTERVAL
        ):
            self.reset_statistics()

        return health_status
