"""
RP2040 SAM Protocol v2.0.0 Main Application
Complete firmware implementation for SAM protocol communication
"""

import gc
import utime
import machine
import uasyncio

# Configuration
import config

# Protocol layers
from protocol.sam_defs import *
from protocol.sam_transport import TransportLayer
from protocol.sam_app import ApplicationLayer

# Drivers
from drivers.uart_driver import UARTDriver, LoopbackUART
from drivers.button_driver import ButtonDriver, ButtonSimulator
from drivers.power_monitor import PowerMonitor, PowerSimulator
from drivers.neopixel_driver import NeoPixelDriver, NeoPixelSimulator
from drivers.eink_driver import EinkDriver, EinkSimulator
from drivers.gpio_control import GPIOControl, GPIOSimulator

# Utilities
from utils.debug_led import DebugLED


class SAMApplication:
    """Main SAM protocol application"""

    def __init__(self, use_simulator=False):
        self.use_simulator = use_simulator
        print(
            f"SAM Protocol v2.0.0 - {'Simulator' if use_simulator else 'Hardware'} Mode"
        )

        # Initialize drivers
        self.init_drivers()

        # Initialize protocol stack
        self.init_protocol()

        # Connect callbacks
        self.connect_callbacks()

        # Application state
        self.running = False
        self.connected = False
        self.main_task_handle = None

        # Statistics
        self.stats = {"uptime": 0, "messages_processed": 0, "errors": 0, "resets": 0}

        # Start time
        self.start_time = utime.ticks_ms()

    def init_drivers(self):
        """Initialize hardware drivers"""
        print("Initializing drivers...")

        if self.use_simulator:
            # Use simulators for testing
            self.uart = LoopbackUART()
            self.buttons = ButtonSimulator()
            self.power = PowerSimulator()
            self.leds = NeoPixelSimulator(config.NEOPIXEL_PIN, config.NEOPIXEL_COUNT)
            self.eink = EinkSimulator()
            self.gpio = GPIOSimulator()
        else:
            # Use real hardware drivers
            self.uart = UARTDriver(
                config.UART_ID,
                config.UART_BAUDRATE,
                config.UART_TX_PIN,
                config.UART_RX_PIN,
            )
            self.buttons = ButtonDriver()
            self.power = PowerMonitor(use_simulator=False)
            self.leds = NeoPixelDriver(config.NEOPIXEL_PIN, config.NEOPIXEL_COUNT)
            self.eink = EinkDriver()
            self.gpio = GPIOControl()

        # Debug LED
        self.debug_led = DebugLED(config.GPIO_DEBUG_LED)

    def init_protocol(self):
        """Initialize protocol stack"""
        print("Initializing protocol stack...")

        # Create transport layer
        self.transport = TransportLayer(self.uart)

        # Create application layer
        self.app = ApplicationLayer(self.transport)

        # Set driver references in app layer
        self.app.set_drivers(
            button=self.buttons,
            battery=self.power.battery,
            led=self.leds,
            eink=self.eink,
            power=self.power,
        )

        # Set transport callbacks
        self.transport.on_message_received = self.app.process_message

    def connect_callbacks(self):
        """Connect driver callbacks to protocol handlers"""
        # UART data callback
        self.uart.on_data_received = self.transport.process_rx_data

        # Button event callback
        self.buttons.on_button_event = self.handle_button_event

        # Power callbacks
        self.power.on_battery_update = self.handle_battery_update
        self.power.on_low_battery = self.handle_low_battery
        self.power.on_critical_battery = self.handle_critical_battery

        # Transport callbacks
        self.transport.on_connected = self.handle_connected
        self.transport.on_disconnected = self.handle_disconnected

    def handle_button_event(self, button_id, event_type):
        """Handle button press events"""
        print(f"Button: ID={button_id}, Event={event_type}")

        # Update power activity
        self.power.update_activity()

        # Send button event to host
        self.app.send_button_event(button_id, event_type)

        # Trigger host interrupt
        self.gpio.trigger_host_irq()

        # Visual feedback
        self.gpio.indicate_activity()

    def handle_battery_update(self, soc, voltage, charging):
        """Handle battery status update"""
        # Send battery status to host
        self.app.send_battery_status()

        # Update LED indication
        if not charging and soc < 20:
            self.leds.indicate_battery(soc)

        # Trigger host interrupt for critical levels
        if soc < 10 and not charging:
            self.gpio.trigger_host_irq()

    def handle_low_battery(self, soc):
        """Handle low battery warning"""
        print(f"Low battery: {soc}%")
        self.leds.flash_notification((255, 255, 0), 2)  # Yellow warning
        self.eink.show_message(f"Low Battery: {soc}%", 0)

    def handle_critical_battery(self, soc):
        """Handle critical battery warning"""
        print(f"Critical battery: {soc}%")
        self.leds.set_pattern(LED_PATTERN_ERROR)
        self.eink.show_message(f"CRITICAL: {soc}%", 0)
        self.gpio.indicate_error(ERROR_HARDWARE)

    def handle_connected(self):
        """Handle protocol connection established"""
        print("Protocol: Connected")
        self.connected = True
        self.leds.indicate_connection(True)
        self.debug_led.indicate_state(STATE_CONNECTED)
        self.eink.show_message("Connected", 1)

        # Send initial status
        self.app.send_battery_status()

    def handle_disconnected(self):
        """Handle protocol disconnection"""
        print("Protocol: Disconnected")
        self.connected = False
        self.leds.indicate_connection(False)
        self.debug_led.indicate_state(STATE_DISCONNECTED)
        self.eink.show_message("Disconnected", 1)

    async def start_async(self):
        """Start SAM application with async tasks"""
        if self.running:
            return

        print("Starting SAM application...")
        self.running = True

        # Set initial LED state
        self.debug_led.indicate_state(STATE_SYNCING)
        self.leds.set_pattern(LED_PATTERN_PULSE)

        # Display startup message
        self.eink.clear()
        self.eink.show_message("SAM v2.0.0", 0)
        self.eink.show_message("Initializing...", 1)

        # Start all async tasks
        tasks = [
            uasyncio.create_task(self.main_task()),
            uasyncio.create_task(self.transport.process_task()),
        ]

        # Start driver tasks
        self.uart.start()
        self.buttons.start()
        self.power.start()
        self.transport.start()

        print("SAM application started")

        # Run all tasks
        await uasyncio.gather(*tasks)

    def stop(self):
        """Stop SAM application"""
        if not self.running:
            return

        print("Stopping SAM application...")
        self.running = False

        # Stop protocol
        self.transport.stop()

        # Stop drivers
        self.uart.stop()
        self.buttons.stop()
        self.power.stop()

        # Clear displays
        self.leds.clear()
        self.debug_led.off()

        # Cancel main task
        if self.main_task_handle:
            self.main_task_handle.cancel()
            self.main_task_handle = None

        print("SAM application stopped")

    async def main_task(self):
        """Main processing task"""
        last_stats_time = utime.ticks_ms()
        last_gc_time = utime.ticks_ms()

        while self.running:
            try:
                now = utime.ticks_ms()

                # Process UART TX buffer
                await self.uart.process_tx_buffer()

                # Process application layer
                self.app.process()

                # Update statistics periodically (every 10 seconds)
                if utime.ticks_diff(now, last_stats_time) >= 10000:
                    self.update_statistics()
                    last_stats_time = now

                # Run garbage collection periodically (every 30 seconds)
                if utime.ticks_diff(now, last_gc_time) >= 30000:
                    gc.collect()
                    last_gc_time = now
                    if config.DEBUG_ENABLED:
                        print(f"GC: Free memory: {gc.mem_free()} bytes")

                # Cooperative yield
                await uasyncio.sleep_ms(1)

            except Exception as e:
                print(f"Main task error: {e}")
                self.stats["errors"] += 1

                # Auto-reset on too many errors
                if self.stats["errors"] > config.MAX_CONSECUTIVE_ERRORS:
                    self.handle_fatal_error()

    def update_statistics(self):
        """Update and optionally report statistics"""
        self.stats["uptime"] = utime.ticks_diff(utime.ticks_ms(), self.start_time)

        if config.DEBUG_ENABLED and config.DEBUG_LEVEL >= 2:
            # Send debug statistics
            transport_stats = self.transport.get_stats()
            self.app.get_stats()
            power_stats = self.power.get_power_stats()

            debug_msg = f"Up:{self.stats['uptime'] // 1000}s "
            debug_msg += f"TX:{transport_stats['frames_sent']} "
            debug_msg += f"RX:{transport_stats['frames_received']} "
            debug_msg += f"Bat:{power_stats['battery']['soc']}%"

            self.app.send_debug_log(debug_msg)

    def handle_fatal_error(self):
        """Handle fatal error condition"""
        print("FATAL: Too many errors, resetting...")
        self.stats["resets"] += 1

        # Try to notify host
        try:
            self.app.send_error_report(ERROR_PROTOCOL, 0xFF)
        except:
            pass

        # Visual indication
        self.leds.set_pattern(LED_PATTERN_ERROR)
        self.gpio.indicate_error(9)

        # Reset after delay
        utime.sleep(2)
        machine.reset()

    def get_stats(self):
        """Get application statistics"""
        stats = self.stats.copy()
        stats["transport"] = self.transport.get_stats()
        stats["app"] = self.app.get_stats()
        stats["uart"] = self.uart.get_stats()
        stats["buttons"] = self.buttons.get_stats()
        stats["power"] = self.power.get_power_stats()
        stats["gpio"] = self.gpio.get_stats()
        return stats


# Global application instance
app = None


async def main():
    """Main entry point"""
    global app

    # Check if running in simulator mode (for testing)
    use_simulator = False
    try:
        # Try to detect hardware
        import machine

        machine.Pin(config.BUTTON_SELECT, machine.Pin.IN)
    except:
        print("Hardware not detected, using simulator mode")
        use_simulator = True

    # Create application
    app = SAMApplication(use_simulator)

    # Start application with async
    await app.start_async()


if __name__ == "__main__":
    # Run the async main function
    uasyncio.run(main())
