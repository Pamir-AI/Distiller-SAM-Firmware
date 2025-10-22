"""Centralized debug system with runtime level control and minimal overhead"""

# pylint: disable=import-error,broad-exception-caught,global-statement,invalid-name
import _thread
from collections import deque
import utime


class DebugHandler:
    """Centralized debug handler with configurable levels and minimal overhead"""

    # Debug levels (aligned with kernel driver)
    LEVEL_OFF = 0  # No debug messages
    LEVEL_ERROR = 1  # Error messages only
    LEVEL_INFO = 2  # Basic informational messages + errors
    LEVEL_VERBOSE = 3  # Verbose debugging with detailed information

    # Message categories (aligned with kernel driver debug categories)
    CAT_SYSTEM = "SYS"  # General system events
    CAT_ERROR = "ERR"  # Error conditions
    CAT_BUTTON = "BTN"  # Button-related events
    CAT_LED = "LED"  # LED subsystem events
    CAT_POWER = "PWR"  # Power state changes
    CAT_DISPLAY = "DSP"  # Display-related events
    CAT_UART = "UART"  # UART communication events
    CAT_PERFORMANCE = "PERF"  # Performance metrics

    def __init__(
        self, initial_level, enable_uart_output, buffer_size, enable_statistics
    ):
        """Initialize debug handler

        Args:
            initial_level: Initial debug level
            enable_uart_output: Whether to output to UART/console
            buffer_size: Size of circular debug buffer
            enable_statistics: Whether to track debug statistics
        """
        self.debug_level = initial_level
        self.enable_uart_output = enable_uart_output
        self.enable_statistics = enable_statistics

        # Thread-safe level changes
        self.level_lock = _thread.allocate_lock()

        # Circular buffer for debug messages
        self.buffer_size = buffer_size
        self.debug_buffer = deque((), buffer_size)
        self.debug_buffer_maxlen = buffer_size
        self.buffer_lock = _thread.allocate_lock()

        # Statistics tracking
        self.stats = {
            "messages_by_level": {
                self.LEVEL_ERROR: 0,
                self.LEVEL_INFO: 0,
                self.LEVEL_VERBOSE: 0,
            },
            "messages_by_category": {},
            "total_messages": 0,
            "suppressed_messages": 0,
            "start_time": utime.ticks_ms(),
        }

        # Performance optimization - pre-allocate format strings
        self.level_prefixes = {
            self.LEVEL_ERROR: "E",
            self.LEVEL_INFO: "I",
            self.LEVEL_VERBOSE: "V",
        }

        # Category filters (can be used to enable/disable specific categories)
        self.category_filters = {
            self.CAT_SYSTEM: True,
            self.CAT_ERROR: True,
            self.CAT_BUTTON: True,
            self.CAT_LED: True,
            self.CAT_POWER: True,
            self.CAT_DISPLAY: True,
            self.CAT_UART: True,
            self.CAT_PERFORMANCE: True,
        }

        # Protocol instance for sending debug codes (if needed)
        # This should be set externally after initialization
        self.protocol = None  # Placeholder for protocol instance

        self._log_startup()

    def _log_startup(self):
        """Log debug handler startup"""
        self.log(
            self.LEVEL_INFO,
            self.CAT_SYSTEM,
            f"Debug handler initialized (level={self.debug_level})",
        )

    def set_level(self, new_level):
        """Set debug level with thread safety

        Args:
            new_level: New debug level (0-3)
        """
        if new_level < self.LEVEL_OFF or new_level > self.LEVEL_VERBOSE:
            return False

        with self.level_lock:
            old_level = self.debug_level
            self.debug_level = new_level

        if self.enable_uart_output:
            level_names = {0: "OFF", 1: "ERROR", 2: "INFO", 3: "VERBOSE"}
            old_name = level_names.get(old_level, str(old_level))
            new_name = level_names.get(new_level, str(new_level))
            print(f"[{utime.ticks_ms()}] DEBUG: Level changed {old_name} -> {new_name}")

        return True

    def get_level(self):
        """Get current debug level (thread-safe)

        Returns:
            int: Current debug level
        """
        with self.level_lock:
            return self.debug_level

    def set_category_filter(self, category, enabled):
        """Enable or disable a specific debug category

        Args:
            category: Category name
            enabled: Whether to enable the category
        """
        if category in self.category_filters:
            self.category_filters[category] = enabled
            if self.enable_uart_output and self.debug_level >= self.LEVEL_INFO:
                print(
                    f"[{utime.ticks_ms()}] DEBUG: {category} {'enabled' if enabled else 'disabled'}"
                )

    def log(self, level, category, message):
        """Log a debug message

        Args:
            level: Message debug level
            category: Message category
            message: Message text
        """
        # Quick early exit for performance
        current_level = self.debug_level  # Atomic read
        if level > current_level:
            if self.enable_statistics:
                self.stats["suppressed_messages"] += 1
            return

        # Check category filter
        if not self.category_filters.get(category, True):
            if self.enable_statistics:
                self.stats["suppressed_messages"] += 1
            return

        timestamp = utime.ticks_ms()

        # Update statistics (minimal overhead)
        if self.enable_statistics:
            self.stats["total_messages"] += 1
            if level in self.stats["messages_by_level"]:
                self.stats["messages_by_level"][level] += 1
            if category not in self.stats["messages_by_category"]:
                self.stats["messages_by_category"][category] = 0
            self.stats["messages_by_category"][category] += 1

        # Create log entry
        level_prefix = self.level_prefixes.get(level, "?")
        log_entry = {
            "timestamp": timestamp,
            "level": level,
            "category": category,
            "message": message,
            "formatted": f"[{timestamp}] {level_prefix}:{category} {message}",
        }

        # Add to circular buffer (thread-safe)
        with self.buffer_lock:
            self.debug_buffer.append(log_entry)
            # Manual maxlen enforcement for MicroPython compatibility
            while len(self.debug_buffer) > self.debug_buffer_maxlen:
                self.debug_buffer.popleft()

        # Output to UART/console if enabled
        if self.enable_uart_output:
            print(log_entry["formatted"])

    def log_error(self, category, message):
        """Log an error message

        Args:
            category: Message category
            message: Error message
        """
        self.log(self.LEVEL_ERROR, category, message)

    def log_info(self, category, message):
        """Log an info message

        Args:
            category: Message category
            message: Info message
        """
        self.log(self.LEVEL_INFO, category, message)

    def log_verbose(self, category, message):
        """Log a verbose message

        Args:
            category: Message category
            message: Verbose message
        """
        self.log(self.LEVEL_VERBOSE, category, message)

    # Convenience methods for common categories
    def log_uart(self, level, message):
        """Log UART-related message"""
        self.log(level, self.CAT_UART, message)

    def log_uart_packet(self, packet_bytes, direction="RX", valid=None):
        """Log UART packet with minimal overhead

        Args:
            packet_bytes: Packet data
            direction: "RX" or "TX"
            valid: Whether packet is valid (None for unknown)
        """
        if self.debug_level < self.LEVEL_VERBOSE:
            return

        # Efficient hex conversion
        hex_str = "".join(f"{b:02X}" for b in packet_bytes[:4])  # Limit to 4 bytes

        validity_str = ""
        if valid is not None:
            validity_str = " ✓" if valid else " ✗"

        packet_msg = f"{direction}: [{hex_str}] len={len(packet_bytes)}{validity_str}"
        self.log_uart(self.LEVEL_VERBOSE, packet_msg)

    def log_button(self, level, message):
        """Log button-related message"""
        self.log(level, self.CAT_BUTTON, message)

    def log_led(self, level, message):
        """Log LED-related message"""
        self.log(level, self.CAT_LED, message)

    def log_power(self, level, message):
        """Log power-related message"""
        self.log(level, self.CAT_POWER, message)

    def log_display(self, level, message):
        """Log display-related message"""
        self.log(level, self.CAT_DISPLAY, message)

    def log_system(self, level, message):
        """Log system-related message"""
        self.log(level, self.CAT_SYSTEM, message)

    def log_performance(self, level, message):
        """Log performance-related message"""
        self.log(level, self.CAT_PERFORMANCE, message)

    def get_recent_messages(self, count=None, level_filter=None, category_filter=None):
        """Get recent debug messages from buffer

        Args:
            count: Maximum number of messages to return
            level_filter: Only return messages at this level or higher
            category_filter: Only return messages from this category

        Returns:
            list: List of matching log entries
        """
        with self.buffer_lock:
            messages = list(self.debug_buffer)

        # Apply filters
        if level_filter is not None:
            messages = [msg for msg in messages if msg["level"] <= level_filter]

        if category_filter is not None:
            messages = [msg for msg in messages if msg["category"] == category_filter]

        # Apply count limit
        if count is not None and count > 0:
            messages = messages[-count:]

        return messages

    def get_statistics(self):
        """Get debug handler statistics

        Returns:
            dict: Statistics information
        """
        if not self.enable_statistics:
            return {"statistics_disabled": True}

        current_time = utime.ticks_ms()
        uptime = utime.ticks_diff(current_time, self.stats["start_time"])

        stats = self.stats.copy()
        stats.update(
            {
                "current_level": self.debug_level,
                "uptime_ms": uptime,
                "buffer_usage": len(self.debug_buffer),
                "buffer_size": self.buffer_size,
                "category_filters": self.category_filters.copy(),
            }
        )

        # Calculate message rate
        if uptime > 0:
            stats["messages_per_second"] = (
                self.stats["total_messages"] * 1000
            ) // uptime
        else:
            stats["messages_per_second"] = 0

        return stats

    def clear_buffer(self):
        """Clear the debug message buffer"""
        with self.buffer_lock:
            self.debug_buffer.clear()
        self.log_info(self.CAT_SYSTEM, "Debug buffer cleared")

    def dump_buffer_to_uart(self, max_lines=50):
        """Dump recent buffer contents to UART

        Args:
            max_lines: Maximum number of lines to dump
        """
        if not self.enable_uart_output:
            return

        print(f"=== DEBUG BUFFER DUMP (last {max_lines} messages) ===")

        recent = self.get_recent_messages(count=max_lines)
        for entry in recent:
            print(entry["formatted"])

        print("=== END DEBUG BUFFER DUMP ===")

    def reset_statistics(self):
        """Reset debug statistics"""
        if self.enable_statistics:
            self.stats = {
                "messages_by_level": {
                    self.LEVEL_ERROR: 0,
                    self.LEVEL_INFO: 0,
                    self.LEVEL_VERBOSE: 0,
                },
                "messages_by_category": {},
                "total_messages": 0,
                "suppressed_messages": 0,
                "start_time": utime.ticks_ms(),
            }
            self.log_info(self.CAT_SYSTEM, "Debug statistics reset")

    def create_debug_code_packet(self, category_code, debug_code, param=0):
        """Create a debug code packet for sending to kernel driver

        Args:
            category_code: Debug category code (0-7)
            debug_code: Specific debug code within category
            param: Optional parameter

        Returns:
            bytes: 4-byte debug packet or None if protocol not available
        """
        # This method requires protocol instance - should be set externally
        if self.protocol:
            # TYPE_DEBUG_CODE (0x80) + category (3 bits) + reserved (2 bits)
            type_flags = 0x80 | ((category_code & 0x07) << 2)
            return self.protocol.create_packet(type_flags, debug_code, param)
        return None

    def send_debug_code(self, uart, category_code, debug_code, param=0):
        """Send debug code to kernel driver via UART

        Args:
            uart: UART interface
            category_code: Debug category (0-7)
            debug_code: Debug code
            param: Optional parameter
        """
        packet = self.create_debug_code_packet(category_code, debug_code, param)
        if packet and uart:
            try:
                uart.write(packet)
                self.log_verbose(
                    self.CAT_UART,
                    f"Debug code sent: cat={category_code}, code={debug_code}",
                )
            except Exception as e:
                self.log_error(self.CAT_UART, f"Failed to send debug code: {e}")


# Global debug handler instance
_global_debug_handler = None


def get_debug_handler():
    """Get global debug handler instance"""
    global _global_debug_handler
    if _global_debug_handler is None:
        _global_debug_handler = DebugHandler(
            initial_level=DebugHandler.LEVEL_ERROR,
            enable_uart_output=True,
            buffer_size=100,
            enable_statistics=True,
        )
    return _global_debug_handler


def init_debug_handler(level, enable_uart_output, buffer_size, enable_statistics):
    """Initialize global debug handler

    Args:
        level: Initial debug level
        enable_uart_output: Whether to output to UART/console
        buffer_size: Size of circular debug buffer
        enable_statistics: Whether to track debug statistics

    Returns:
        DebugHandler: Initialized debug handler
    """
    global _global_debug_handler
    _global_debug_handler = DebugHandler(
        initial_level=level,
        enable_uart_output=enable_uart_output,
        buffer_size=buffer_size,
        enable_statistics=enable_statistics,
    )
    return _global_debug_handler
