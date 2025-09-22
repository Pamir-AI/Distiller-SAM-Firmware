"""
SAM Protocol Layer 3 - Application Layer
Message routing and subsystem handlers
"""

import struct
from protocol.sam_defs import *
from utils.priority_queue import MessagePriorityQueue
import config


class ApplicationLayer:
    """SAM Protocol Application Layer"""

    def __init__(self, transport=None):
        self.transport = transport
        self.handlers = {}
        self.tx_queue = MessagePriorityQueue(config.MAX_PENDING_MESSAGES)

        # Register default handlers
        self.register_handlers()

        # Subsystem interfaces
        self.button_driver = None
        self.battery_driver = None
        self.led_driver = None
        self.eink_driver = None
        self.power_monitor = None

        # Statistics
        self.stats = {
            "messages_sent": 0,
            "messages_received": 0,
            "invalid_messages": 0,
            "handler_errors": 0,
        }

    def register_handlers(self):
        """Register message handlers"""
        # System messages
        self.handlers[MSG_RESET_REQ] = self.handle_reset
        self.handlers[MSG_VERSION_REQ] = self.handle_get_version
        self.handlers[MSG_SET_CONFIG] = self.handle_set_config
        self.handlers[MSG_STATUS_REQ] = self.handle_get_status
        self.handlers[MSG_HEARTBEAT] = self.handle_heartbeat
        self.handlers[MSG_ERROR_REPORT] = self.handle_error_report

        # Input messages
        self.handlers[MSG_GET_INPUT_STATE] = self.handle_get_input_state

        # Output messages
        self.handlers[MSG_LED_SET] = self.handle_led_set
        self.handlers[MSG_LED_PATTERN] = self.handle_led_pattern
        self.handlers[MSG_GPIO_WRITE] = self.handle_gpio_set
        self.handlers[MSG_GET_OUTPUT_STATE] = self.handle_get_output_state

        # Power messages
        self.handlers[MSG_POWER_MODE] = self.handle_power_mode

        # Display messages
        self.handlers[MSG_DISPLAY_CLEAR] = self.handle_eink_clear
        self.handlers[MSG_DISPLAY_UPDATE] = self.handle_eink_update
        self.handlers[MSG_DISPLAY_PARTIAL] = self.handle_eink_partial
        self.handlers[MSG_DISPLAY_STATUS] = self.handle_eink_status

        # Debug messages
        self.handlers[MSG_DEBUG_STATS] = self.handle_debug_stats
        self.handlers[MSG_DEBUG_TEST] = self.handle_debug_test

    def set_drivers(self, button=None, battery=None, led=None, eink=None, power=None):
        """Set subsystem driver references"""
        self.button_driver = button
        self.battery_driver = battery
        self.led_driver = led
        self.eink_driver = eink
        self.power_monitor = power

    def process_message(self, data):
        """Process received application message"""
        if not data or len(data) == 0:
            self.stats["invalid_messages"] += 1
            return

        msg_type = data[0]
        msg_class = msg_type & 0xF0

        # Reject reserved message classes
        if msg_class >= 0x60:
            self.stats["invalid_messages"] += 1
            self.send_error_report(ERROR_INVALID_MSG, msg_type)
            return

        self.stats["messages_received"] += 1

        # Route to handler
        if msg_type in self.handlers:
            try:
                self.handlers[msg_type](data[1:] if len(data) > 1 else bytes())
            except Exception as e:
                self.stats["handler_errors"] += 1
                print(f"Handler error for {hex(msg_type)}: {e}")
                self.send_error_report(ERROR_PROTOCOL, msg_type)
        else:
            # Unknown message type
            self.stats["invalid_messages"] += 1
            self.send_error_report(ERROR_INVALID_MSG, msg_type)

    def send_message(self, msg_type, payload=None):
        """Send application message"""
        # Build message
        message = bytearray([msg_type])
        if payload:
            message.extend(payload)

        # Get priority based on message class
        msg_class = msg_type & 0xF0
        priority_map = {
            CLASS_POWER: config.PRIORITY_BATTERY,
            CLASS_DISPLAY: config.PRIORITY_EINK,
            CLASS_OUTPUT: config.PRIORITY_LED,
            CLASS_INPUT: config.PRIORITY_INPUT,
        }
        priority = priority_map.get(msg_class, 1)

        # Queue for transmission
        self.stats["messages_sent"] += 1
        if self.transport:
            return self.transport.send_message(bytes(message), priority)
        return False

    # System message handlers
    def handle_reset(self, data):
        """Handle system reset request"""
        import machine

        self.send_message(MSG_RESET_ACK, bytes([0x01]))  # Acknowledge
        machine.reset()

    def handle_get_version(self, data):
        """Handle version request"""
        version = struct.pack(
            ">HBB", config.PROTOCOL_VERSION, SYNC_VERSION_MAJOR, SYNC_VERSION_MINOR
        )
        self.send_message(MSG_VERSION_RSP, version)

    def handle_set_config(self, data):
        """Handle configuration update"""
        if len(data) >= 2:
            param_id = data[0]
            data[1]
            # TODO: Implement configuration parameters
            self.send_message(MSG_SET_CONFIG, bytes([param_id, 0x01]))  # ACK

    def handle_get_status(self, data):
        """Handle status request"""
        status = bytearray()
        # Connection state
        status.append(self.transport.state if self.transport else 0)
        # Battery level
        if self.battery_driver:
            status.append(self.battery_driver.get_soc())
        else:
            status.append(0xFF)
        # Error count
        status.append(self.stats["invalid_messages"] & 0xFF)
        self.send_message(MSG_STATUS_RSP, bytes(status))

    def handle_heartbeat(self, data):
        """Handle heartbeat (no response needed)"""
        pass

    def handle_error_report(self, data):
        """Handle error report from host"""
        if len(data) >= 2:
            error_code = data[0]
            error_data = data[1]
            print(f"Host error: code={error_code}, data={error_data}")

    # Input message handlers
    def handle_get_input_state(self, data):
        """Handle input state request"""
        state = bytearray()
        if self.button_driver:
            state.append(self.button_driver.get_button_states())
        else:
            state.append(0x00)
        self.send_message(MSG_GET_INPUT_STATE, bytes(state))

    def send_button_event(self, button_id, event_type):
        """Send button event to host"""
        payload = bytes([button_id, event_type])
        self.send_message(MSG_BUTTON_EVENT, payload)

    # Output message handlers
    def handle_led_set(self, data):
        """Handle LED control"""
        if len(data) >= 4 and self.led_driver:
            led_index = data[0]
            r = data[1]
            g = data[2]
            b = data[3]
            self.led_driver.set_pixel(led_index, r, g, b)
            self.led_driver.show()
            self.send_message(MSG_LED_SET, bytes([led_index, 0x01]))  # ACK

    def handle_led_pattern(self, data):
        """Handle LED pattern request"""
        if len(data) >= 1 and self.led_driver:
            pattern = data[0]
            self.led_driver.set_pattern(pattern)
            self.send_message(MSG_LED_PATTERN, bytes([pattern, 0x01]))  # ACK

    def handle_gpio_set(self, data):
        """Handle GPIO control"""
        if len(data) >= 2:
            gpio_num = data[0]
            data[1]
            # TODO: Implement GPIO control
            self.send_message(MSG_GPIO_WRITE, bytes([gpio_num, 0x01]))  # ACK

    def handle_get_output_state(self, data):
        """Handle output state request"""
        state = bytearray([0x00])  # TODO: Implement
        self.send_message(MSG_GET_OUTPUT_STATE, bytes(state))

    # Power message handlers
    def handle_power_mode(self, data):
        """Handle power mode change"""
        if len(data) >= 1:
            mode = data[0]
            if self.power_monitor:
                self.power_monitor.set_power_mode(mode)
            self.send_message(MSG_POWER_MODE, bytes([mode, 0x01]))  # ACK

    def send_battery_status(self):
        """Send battery status to host"""
        if self.battery_driver:
            soc = self.battery_driver.get_soc()
            voltage = int(
                self.battery_driver.get_voltage() * 100
            )  # Convert to centivolts
            current = struct.pack(">h", int(self.battery_driver.get_current()))
            charging = 0x01 if self.battery_driver.is_charging() else 0x00

            payload = bytearray()
            payload.append(soc)
            payload.extend(struct.pack(">H", voltage))
            payload.extend(current)
            payload.append(charging)

            self.send_message(MSG_BATTERY_STATUS, bytes(payload))

    # Display message handlers
    def handle_eink_clear(self, data):
        """Handle e-ink clear request"""
        if self.eink_driver:
            self.eink_driver.clear()
            self.send_message(MSG_DISPLAY_CLEAR, bytes([0x01]))  # ACK

    def handle_eink_update(self, data):
        """Handle e-ink update"""
        if self.eink_driver and len(data) > 0:
            # TODO: Implement full update protocol
            self.send_message(MSG_DISPLAY_UPDATE, bytes([0x01]))  # ACK

    def handle_eink_partial(self, data):
        """Handle e-ink partial update"""
        if self.eink_driver and len(data) >= 5:
            data[0]
            data[1]
            data[2]
            data[3]
            # TODO: Implement partial update
            self.send_message(MSG_DISPLAY_PARTIAL, bytes([0x01]))  # ACK

    def handle_eink_status(self, data):
        """Handle e-ink status request"""
        status = bytearray()
        if self.eink_driver:
            status.append(0x00 if self.eink_driver.is_busy() else 0x01)
        else:
            status.append(0xFF)
        self.send_message(MSG_DISPLAY_STATUS, bytes(status))

    # Debug message handlers
    def handle_debug_stats(self, data):
        """Handle debug statistics request"""
        stats = bytearray()
        # Add various statistics
        stats.extend(struct.pack(">I", self.stats["messages_sent"]))
        stats.extend(struct.pack(">I", self.stats["messages_received"]))
        stats.extend(struct.pack(">H", self.stats["invalid_messages"]))
        stats.extend(struct.pack(">H", self.stats["handler_errors"]))
        self.send_message(MSG_DEBUG_STATS, bytes(stats))

    def handle_debug_test(self, data):
        """Handle debug test message"""
        # Echo back test data
        self.send_message(MSG_DEBUG_TEST, data)

    def send_debug_log(self, text):
        """Send debug log message"""
        if config.DEBUG_ENABLED:
            payload = text.encode("utf-8")[:249]  # Limit to max payload
            self.send_message(MSG_DEBUG_LOG, payload)

    def send_error_report(self, error_code, error_data=0):
        """Send error report to host"""
        payload = bytes([error_code, error_data])
        self.send_message(MSG_ERROR_REPORT, payload)

    def process(self):
        """Process application layer tasks"""
        # Process any queued transmit messages
        while not self.tx_queue.is_empty():
            msg = self.tx_queue.get()
            if msg and self.transport:
                msg_class = msg[0] & 0xF0
                priority_map = {
                    CLASS_POWER: config.PRIORITY_BATTERY,
                    CLASS_DISPLAY: config.PRIORITY_EINK,
                    CLASS_OUTPUT: config.PRIORITY_LED,
                    CLASS_INPUT: config.PRIORITY_INPUT,
                }
                priority = priority_map.get(msg_class, 1)
                self.transport.send_message(msg, priority)

    def get_stats(self):
        """Get application layer statistics"""
        return self.stats.copy()
