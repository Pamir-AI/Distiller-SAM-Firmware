"""
Debug LED Status Patterns
Visual indication of system state and errors
"""

import machine
import utime
import uasyncio


class DebugLED:
    """Debug LED controller with pattern support"""

    def __init__(self, pin_num):
        self.pin = machine.Pin(pin_num, machine.Pin.OUT)
        self.pattern = None
        self.pattern_task_handle = None

    def on(self):
        """Turn LED on"""
        self.stop_current_pattern()
        self.pin.value(1)

    def off(self):
        """Turn LED off"""
        self.stop_current_pattern()
        self.pin.value(0)

    def toggle(self):
        """Toggle LED state"""
        self.pin.value(not self.pin.value())

    async def blink(self, count=1, on_ms=100, off_ms=100):
        """Blink LED specified number of times"""
        self.stop_current_pattern()
        for i in range(count):
            self.pin.value(1)
            await uasyncio.sleep_ms(on_ms)
            self.pin.value(0)
            if i < count - 1:
                await uasyncio.sleep_ms(off_ms)

    def stop_current_pattern(self):
        """Stop any running pattern"""
        if self.pattern_task_handle:
            self.pattern_task_handle.cancel()
            self.pattern_task_handle = None

    async def pattern_task(self, pattern_name):
        """Task for LED patterns"""
        try:
            while True:
                if pattern_name == "heartbeat":
                    await self.heartbeat_pattern()
                elif pattern_name == "error":
                    await self.error_pattern()
                elif pattern_name == "activity":
                    await self.activity_pattern()
                elif pattern_name == "sync":
                    await self.sync_pattern()
                elif pattern_name == "slow_blink":
                    await self.slow_blink_pattern()
                elif pattern_name == "fast_blink":
                    await self.fast_blink_pattern()
        except uasyncio.CancelledError:
            self.pin.value(0)
        except Exception as e:
            print(f"LED pattern error: {e}")
            self.pin.value(0)

    def start_pattern(self, pattern_name):
        """Start a named pattern"""
        self.stop_current_pattern()
        self.pattern = pattern_name
        if pattern_name in [
            "heartbeat",
            "error",
            "activity",
            "sync",
            "slow_blink",
            "fast_blink",
        ]:
            self.pattern_task_handle = uasyncio.create_task(
                self.pattern_task(pattern_name)
            )

    async def heartbeat_pattern(self):
        """Heartbeat pattern (double pulse)"""
        self.pin.value(1)
        await uasyncio.sleep_ms(100)
        self.pin.value(0)
        await uasyncio.sleep_ms(100)
        self.pin.value(1)
        await uasyncio.sleep_ms(100)
        self.pin.value(0)
        await uasyncio.sleep_ms(700)

    async def error_pattern(self):
        """Error pattern (rapid flashing)"""
        self.pin.value(1)
        await uasyncio.sleep_ms(50)
        self.pin.value(0)
        await uasyncio.sleep_ms(50)

    async def activity_pattern(self):
        """Activity pattern (brief flash)"""
        self.pin.value(1)
        await uasyncio.sleep_ms(20)
        self.pin.value(0)
        await uasyncio.sleep_ms(480)

    async def sync_pattern(self):
        """Sync pattern (alternating long/short)"""
        self.pin.value(1)
        await uasyncio.sleep_ms(300)
        self.pin.value(0)
        await uasyncio.sleep_ms(200)
        self.pin.value(1)
        await uasyncio.sleep_ms(100)
        self.pin.value(0)
        await uasyncio.sleep_ms(400)

    async def slow_blink_pattern(self):
        """Slow blink pattern"""
        self.pin.value(1)
        await uasyncio.sleep_ms(500)
        self.pin.value(0)
        await uasyncio.sleep_ms(500)

    async def fast_blink_pattern(self):
        """Fast blink pattern"""
        self.pin.value(1)
        await uasyncio.sleep_ms(100)
        self.pin.value(0)
        await uasyncio.sleep_ms(100)

    def indicate_state(self, state):
        """Indicate connection state with pattern"""
        patterns = {
            0: "off",  # Disconnected
            1: "sync",  # Syncing
            2: "heartbeat",  # Connected
            3: "error",  # Error state
        }
        pattern = patterns.get(state, "slow_blink")
        if pattern == "off":
            self.off()
        else:
            self.start_pattern(pattern)

    def flash_code(self, code):
        """Flash a numeric code (1-9)"""
        if not 1 <= code <= 9:
            return
        self.stop_current_pattern()
        # Long flash to indicate start
        self.pin.value(1)
        utime.sleep_ms(500)
        self.pin.value(0)
        utime.sleep_ms(300)
        # Flash code
        self.blink(code, 150, 150)


class StatusLEDs:
    """Multiple status LEDs for different subsystems"""

    def __init__(self):
        self.leds = {}

    def add_led(self, name, pin_num):
        """Add a status LED"""
        self.leds[name] = DebugLED(pin_num)

    def set_state(self, name, state):
        """Set LED state by name"""
        if name in self.leds:
            if state == "on":
                self.leds[name].on()
            elif state == "off":
                self.leds[name].off()
            else:
                self.leds[name].start_pattern(state)

    def all_off(self):
        """Turn all LEDs off"""
        for led in self.leds.values():
            led.off()

    def indicate_error(self, error_code):
        """Indicate error on all LEDs"""
        for led in self.leds.values():
            led.flash_code(error_code)
