"""
GPIO Control Driver
Manages host interrupt and debug GPIO
"""

import machine
import utime
import config


class GPIOControl:
    """GPIO control for host interrupts and debug"""

    def __init__(self):
        # Host interrupt pin (output to host)
        self.host_irq = machine.Pin(config.GPIO_HOST_IRQ, machine.Pin.OUT, value=0)

        # Debug LED pin
        self.debug_led = machine.Pin(config.GPIO_DEBUG_LED, machine.Pin.OUT, value=0)

        # Interrupt state
        self.irq_active = False
        self.irq_pulse_ms = 10  # IRQ pulse width

        # Statistics
        self.stats = {"irq_count": 0, "debug_toggles": 0}

    def trigger_host_irq(self):
        """Trigger interrupt to host processor"""
        if not self.irq_active:
            self.irq_active = True
            self.host_irq.value(1)
            self.stats["irq_count"] += 1
            # Pulse the interrupt line
            utime.sleep_ms(self.irq_pulse_ms)
            self.host_irq.value(0)
            self.irq_active = False

    def set_host_irq(self, state):
        """Set host interrupt line state"""
        self.host_irq.value(1 if state else 0)
        if state:
            self.stats["irq_count"] += 1

    def clear_host_irq(self):
        """Clear host interrupt"""
        self.host_irq.value(0)
        self.irq_active = False

    def set_debug_led(self, state):
        """Set debug LED state"""
        self.debug_led.value(1 if state else 0)
        self.stats["debug_toggles"] += 1

    def toggle_debug_led(self):
        """Toggle debug LED state"""
        self.debug_led.value(not self.debug_led.value())
        self.stats["debug_toggles"] += 1

    def blink_debug_led(self, count=1, on_ms=100, off_ms=100):
        """Blink debug LED"""
        for i in range(count):
            self.set_debug_led(True)
            utime.sleep_ms(on_ms)
            self.set_debug_led(False)
            if i < count - 1:
                utime.sleep_ms(off_ms)

    def indicate_error(self, error_code):
        """Indicate error with debug LED"""
        # Flash error code on debug LED
        self.blink_debug_led(error_code, 200, 200)

    def indicate_activity(self):
        """Brief activity flash on debug LED"""
        self.set_debug_led(True)
        utime.sleep_ms(5)
        self.set_debug_led(False)

    def get_stats(self):
        """Get GPIO statistics"""
        return self.stats.copy()

    def reset_stats(self):
        """Reset statistics"""
        self.stats = {"irq_count": 0, "debug_toggles": 0}


class GPIOSimulator:
    """GPIO simulator for testing"""

    def __init__(self):
        self.host_irq_state = False
        self.debug_led_state = False
        self.stats = {"irq_count": 0, "debug_toggles": 0}
        print("GPIOSimulator: Initialized")

    def trigger_host_irq(self):
        """Simulate IRQ trigger"""
        self.host_irq_state = True
        self.stats["irq_count"] += 1
        print("GPIO: Host IRQ triggered")
        utime.sleep_ms(10)
        self.host_irq_state = False

    def set_host_irq(self, state):
        """Simulate IRQ state set"""
        self.host_irq_state = state
        if state:
            self.stats["irq_count"] += 1
        print(f"GPIO: Host IRQ = {state}")

    def clear_host_irq(self):
        """Simulate IRQ clear"""
        self.host_irq_state = False

    def set_debug_led(self, state):
        """Simulate debug LED"""
        self.debug_led_state = state
        self.stats["debug_toggles"] += 1

    def toggle_debug_led(self):
        """Simulate debug LED toggle"""
        self.debug_led_state = not self.debug_led_state
        self.stats["debug_toggles"] += 1

    def blink_debug_led(self, count=1, on_ms=100, off_ms=100):
        """Simulate debug LED blink"""
        print(f"GPIO: Debug LED blink x{count}")

    def indicate_error(self, error_code):
        """Simulate error indication"""
        print(f"GPIO: Error code {error_code}")

    def indicate_activity(self):
        """Simulate activity indication"""
        pass

    def get_stats(self):
        """Get simulator statistics"""
        return self.stats.copy()

    def reset_stats(self):
        """Reset simulator statistics"""
        self.stats = {"irq_count": 0, "debug_toggles": 0}
