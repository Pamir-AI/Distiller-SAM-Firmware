"""
NeoPixel LED Driver
WS2812B RGB LED control with patterns and effects
"""

import machine
import array
import utime
import uasyncio
import math
from protocol.sam_defs import *


class NeoPixelDriver:
    """NeoPixel WS2812B LED driver"""

    def __init__(self, pin_num=6, num_pixels=8):
        self.num_pixels = num_pixels
        self.pin = machine.Pin(pin_num, machine.Pin.OUT)
        self.np = array.array("I", [0] * num_pixels)

        # Pattern control
        self.current_pattern = LED_PATTERN_OFF
        self.pattern_task_handle = None
        self.pattern_speed = 50  # ms between updates

        # Color values
        self.brightness = 32  # 0-255
        self.primary_color = (0, 0, 255)  # Blue
        self.secondary_color = (255, 0, 0)  # Red

        # Animation state
        self.animation_step = 0

    def set_pixel(self, index, r, g, b):
        """Set individual pixel color"""
        if 0 <= index < self.num_pixels:
            # Apply brightness
            r = (r * self.brightness) // 255
            g = (g * self.brightness) // 255
            b = (b * self.brightness) // 255
            # WS2812B expects GRB format
            self.np[index] = (g << 16) | (r << 8) | b

    def set_all(self, r, g, b):
        """Set all pixels to same color"""
        for i in range(self.num_pixels):
            self.set_pixel(i, r, g, b)

    def clear(self):
        """Turn off all pixels"""
        self.set_all(0, 0, 0)
        self.show()

    def show(self):
        """Update physical LEDs"""
        # Simplified bit-banging for WS2812B
        # In production, use PIO or SPI for better timing
        self._write_pixels()

    def _write_pixels(self):
        """Low-level pixel data transmission"""
        # TODO: Implement proper WS2812B timing
        # For now, this is a placeholder
        # Real implementation would use PIO on RP2040
        pass

    def set_brightness(self, brightness):
        """Set global brightness (0-255)"""
        self.brightness = min(255, max(0, brightness))

    def set_pattern(self, pattern):
        """Set LED pattern"""
        # Stop current pattern
        self.stop_current_pattern()

        self.current_pattern = pattern

        if pattern == LED_PATTERN_OFF:
            self.clear()
        elif pattern == LED_PATTERN_SOLID:
            self.pattern_solid()
        elif pattern != LED_PATTERN_OFF:
            # Start pattern task for animated patterns
            self.pattern_task_handle = uasyncio.create_task(self.pattern_task())

    def stop_current_pattern(self):
        """Stop running pattern"""
        if self.pattern_task_handle:
            self.pattern_task_handle.cancel()
            self.pattern_task_handle = None

    async def pattern_task(self):
        """Task for animated patterns"""
        try:
            while True:
                if self.current_pattern == LED_PATTERN_BLINK:
                    self.pattern_blink()
                elif self.current_pattern == LED_PATTERN_PULSE:
                    self.pattern_pulse()
                elif self.current_pattern == LED_PATTERN_RAINBOW:
                    self.pattern_rainbow()
                elif self.current_pattern == LED_PATTERN_ERROR:
                    self.pattern_error()

                await uasyncio.sleep_ms(self.pattern_speed)
                self.animation_step += 1
        except uasyncio.CancelledError:
            self.clear()
        except Exception as e:
            print(f"LED pattern error: {e}")
            self.clear()

    def pattern_solid(self):
        """Solid color pattern"""
        r, g, b = self.primary_color
        self.set_all(r, g, b)
        self.show()

    def pattern_blink(self):
        """Blinking pattern"""
        if self.animation_step % 10 < 5:
            r, g, b = self.primary_color
            self.set_all(r, g, b)
        else:
            self.clear()
        self.show()

    def pattern_pulse(self):
        """Pulsing/breathing pattern"""
        # Calculate brightness based on sine wave
        brightness = int(128 + 127 * math.sin(self.animation_step * 0.1))
        old_brightness = self.brightness
        self.brightness = brightness
        r, g, b = self.primary_color
        self.set_all(r, g, b)
        self.show()
        self.brightness = old_brightness

    def pattern_rainbow(self):
        """Rainbow cycle pattern"""
        for i in range(self.num_pixels):
            hue = ((self.animation_step * 5) + (i * 360 // self.num_pixels)) % 360
            r, g, b = self.hsv_to_rgb(hue, 100, 100)
            self.set_pixel(i, r, g, b)
        self.show()

    def pattern_error(self):
        """Error indication pattern (fast red flash)"""
        if self.animation_step % 4 < 2:
            self.set_all(255, 0, 0)  # Red
        else:
            self.clear()
        self.show()

    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB color space"""
        h = h / 60
        s = s / 100
        v = v / 100

        c = v * s
        x = c * (1 - abs(h % 2 - 1))
        m = v - c

        if h < 1:
            r, g, b = c, x, 0
        elif h < 2:
            r, g, b = x, c, 0
        elif h < 3:
            r, g, b = 0, c, x
        elif h < 4:
            r, g, b = 0, x, c
        elif h < 5:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x

        return (int((r + m) * 255), int((g + m) * 255), int((b + m) * 255))

    def indicate_battery(self, soc):
        """Show battery level on LEDs"""
        # Use pixels as battery meter
        lit_pixels = (soc * self.num_pixels) // 100

        for i in range(self.num_pixels):
            if i < lit_pixels:
                if soc > 50:
                    self.set_pixel(i, 0, 255, 0)  # Green
                elif soc > 20:
                    self.set_pixel(i, 255, 255, 0)  # Yellow
                else:
                    self.set_pixel(i, 255, 0, 0)  # Red
            else:
                self.set_pixel(i, 0, 0, 0)  # Off
        self.show()

    def indicate_connection(self, connected):
        """Show connection status"""
        if connected:
            self.set_pattern(LED_PATTERN_SOLID)
        else:
            self.set_pattern(LED_PATTERN_BLINK)

    def flash_notification(self, color=(255, 255, 255), count=3):
        """Flash notification pattern"""
        self.stop_current_pattern()
        for _ in range(count):
            self.set_all(*color)
            self.show()
            utime.sleep_ms(100)
            self.clear()
            utime.sleep_ms(100)


class NeoPixelSimulator:
    """NeoPixel simulator for testing"""

    def __init__(self, pin_num=6, num_pixels=8):
        self.num_pixels = num_pixels
        self.pixels = [(0, 0, 0)] * num_pixels
        self.brightness = 32
        self.current_pattern = LED_PATTERN_OFF
        print(f"NeoPixelSimulator: {num_pixels} pixels on pin {pin_num}")

    def set_pixel(self, index, r, g, b):
        """Simulate setting pixel"""
        if 0 <= index < self.num_pixels:
            r = (r * self.brightness) // 255
            g = (g * self.brightness) // 255
            b = (b * self.brightness) // 255
            self.pixels[index] = (r, g, b)

    def set_all(self, r, g, b):
        """Simulate setting all pixels"""
        for i in range(self.num_pixels):
            self.set_pixel(i, r, g, b)

    def clear(self):
        """Simulate clearing pixels"""
        self.pixels = [(0, 0, 0)] * self.num_pixels

    def show(self):
        """Simulate showing pixels"""
        # In simulation, just print state occasionally
        pass

    def set_brightness(self, brightness):
        """Simulate brightness setting"""
        self.brightness = min(255, max(0, brightness))

    def set_pattern(self, pattern):
        """Simulate pattern setting"""
        self.current_pattern = pattern
        print(f"LED pattern: {pattern}")

    def stop_current_pattern(self):
        """Stop pattern simulation"""
        pass

    def indicate_battery(self, soc):
        """Simulate battery indication"""
        print(f"LED battery: {soc}%")

    def indicate_connection(self, connected):
        """Simulate connection indication"""
        print(f"LED connection: {'connected' if connected else 'disconnected'}")

    def flash_notification(self, color=(255, 255, 255), count=3):
        """Simulate notification flash"""
        print(f"LED flash: {color} x{count}")
