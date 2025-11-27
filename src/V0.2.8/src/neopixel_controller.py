"""NeoPixel controller for Pamir SAM with animation support"""

# pylint: disable=import-error,broad-exception-caught
import _thread
import machine
import neopixel
import utime


class NeoPixelController:
    """Controller for NeoPixel LEDs with animation capabilities"""

    # Animation modes
    MODE_STATIC = 0
    MODE_BLINK = 1
    MODE_FADE = 2
    MODE_RAINBOW = 3
    MODE_SEQUENCE = 4

    def __init__(
        self, pin=7, num_leds=7, default_brightness=0.5, completion_callback=None
    ):
        self.pin = pin
        self.num_leds = num_leds
        self.np = neopixel.NeoPixel(machine.Pin(pin), num_leds)
        self.np.brightness = min(max(default_brightness, 0.0), 1.0)

        # Thread safety
        self.lock = _thread.allocate_lock()

        # Animation state
        self.animation_running = False
        self.animation_queue = []
        self.current_thread = None

        # Completion callback for LED acknowledgments
        self.completion_callback = completion_callback
        self.last_executed_led_id = 0
        self.last_sequence_length = 0

        # Clear all LEDs on init
        self.clear_all()

    def clear_all(self):
        """Turn off all LEDs"""
        with self.lock:
            for i in range(self.num_leds):
                self.np[i] = (0, 0, 0)
            self.np.write()

    def set_color(self, color, brightness=None, index=None):
        """Set color for specific LED or all LEDs

        Args:
            color: [r, g, b] values (0-255)
            brightness: Optional brightness (0.0-1.0)
            index: LED index (None for all LEDs)
        """
        with self.lock:
            if brightness is not None:
                self.np.brightness = min(max(brightness, 0.0), 1.0)

            r = int(color[0] * self.np.brightness)
            g = int(color[1] * self.np.brightness)
            b = int(color[2] * self.np.brightness)

            if index is None:
                for i, _ in enumerate(self.np):
                    self.np[i] = (r, g, b)
            else:
                if 0 <= index < len(self.np):
                    self.np[index] = (r, g, b)

            self.np.write()

    def rgb444_to_rgb888(self, rgb444_data):
        """Convert RGB444 (4 bits per channel) to RGB888 (8 bits per channel)

        Args:
            rgb444_data: (r4, g4, b4) tuple with 4-bit values (0-15)

        Returns:
            (r8, g8, b8) tuple with 8-bit values (0-255)
        """
        r4, g4, b4 = rgb444_data
        # Scale 4-bit to 8-bit: multiply by 17 (255/15)
        r8 = (r4 & 0x0F) * 17
        g8 = (g4 & 0x0F) * 17
        b8 = (b4 & 0x0F) * 17
        return (r8, g8, b8)

    def stop_animation(self):
        """Stop current animation"""
        self.animation_running = False

    def add_to_queue(self, led_id, mode, color_data, time_value):
        """Add LED command to animation queue

        Args:
            led_id: LED index (0-15, or 255 for all LEDs)
            mode: Animation mode (MODE_STATIC, MODE_BLINK, etc.)
            color_data: (r4, g4, b4) tuple (4-bit values)
            time_value: Timing parameter (0-15)
        """
        command = {
            "led_id": led_id,
            "mode": mode,
            "color": self.rgb444_to_rgb888(color_data),
            "time_value": time_value,
            "delay_ms": (time_value + 1) * 100,  # Convert to milliseconds
        }
        self.animation_queue.append(command)

    def execute_queue(self):
        """Execute all queued LED commands in sequence (non-blocking)"""
        if not self.animation_queue:
            return

        # Stop any current animation
        self.stop_animation()

        # Execute immediately without threading to avoid core1 conflict
        self._execute_animation_immediate()

    def _execute_animation_immediate(self):
        """Execute animation queue immediately without threading"""
        self.animation_running = True
        executed_commands = 0
        last_led_id = 0

        try:
            for command in self.animation_queue:
                if not self.animation_running:
                    break

                led_id = command["led_id"]
                mode = command["mode"]
                color = command["color"]
                delay_ms = command["delay_ms"]
                last_led_id = led_id

                if mode == self.MODE_STATIC:
                    self._animate_static(led_id, color)
                elif mode == self.MODE_BLINK:
                    self._animate_blink_quick(led_id, color, delay_ms)
                elif mode == self.MODE_FADE:
                    self._animate_fade_quick(led_id, color, delay_ms)
                elif mode == self.MODE_RAINBOW:
                    self._animate_rainbow_quick(led_id, delay_ms)
                elif mode == self.MODE_SEQUENCE:
                    self._animate_sequence(led_id, color, delay_ms)

                executed_commands += 1
                # Small delay between commands
                utime.sleep_ms(10)

        except Exception as e:
            print(f"Animation error: {e}")
        finally:
            self.animation_running = False

            # Store completion info for acknowledgment
            self.last_executed_led_id = last_led_id
            self.last_sequence_length = executed_commands

            # Send completion acknowledgment if callback is set
            if self.completion_callback and executed_commands > 0:
                try:
                    self.completion_callback(last_led_id, executed_commands)
                except Exception as e:
                    print(f"Completion callback error: {e}")

            self.animation_queue.clear()

    def _execute_animation_thread(self):
        """Thread function to execute animation queue"""
        self.animation_running = True
        executed_commands = 0
        last_led_id = 0

        try:
            for command in self.animation_queue:
                if not self.animation_running:
                    break

                led_id = command["led_id"]
                mode = command["mode"]
                color = command["color"]
                delay_ms = command["delay_ms"]
                last_led_id = led_id

                if mode == self.MODE_STATIC:
                    self._animate_static(led_id, color)
                elif mode == self.MODE_BLINK:
                    self._animate_blink(led_id, color, delay_ms)
                elif mode == self.MODE_FADE:
                    self._animate_fade(led_id, color, delay_ms)
                elif mode == self.MODE_RAINBOW:
                    self._animate_rainbow(led_id, delay_ms)
                elif mode == self.MODE_SEQUENCE:
                    self._animate_sequence(led_id, color, delay_ms)

                executed_commands += 1
                # Small delay between commands
                utime.sleep_ms(10)

        except Exception as e:
            print(f"Animation error: {e}")
        finally:
            self.animation_running = False

            # Store completion info for acknowledgment
            self.last_executed_led_id = last_led_id
            self.last_sequence_length = executed_commands

            # Send completion acknowledgment if callback is set
            if self.completion_callback and executed_commands > 0:
                try:
                    self.completion_callback(last_led_id, executed_commands)
                except Exception as e:
                    print(f"Completion callback error: {e}")

            self.animation_queue.clear()

    def _animate_static(self, led_id, color):
        """Set static color"""
        if led_id == 255:  # All LEDs
            self.set_color(color)
        else:
            self.set_color(color, index=led_id)

    def _animate_blink(self, led_id, color, delay_ms):
        """Blink animation"""
        for _ in range(5):  # Blink 5 times
            if not self.animation_running:
                break

            # Turn on
            if led_id == 255:
                self.set_color(color)
            else:
                self.set_color(color, index=led_id)
            utime.sleep_ms(delay_ms // 2)

            # Turn off
            if led_id == 255:
                self.set_color([0, 0, 0])
            else:
                self.set_color([0, 0, 0], index=led_id)
            utime.sleep_ms(delay_ms // 2)

    def _animate_fade(self, led_id, color, delay_ms):
        """Fade in/out animation"""
        steps = 20
        step_delay = delay_ms // (steps * 2)

        # Fade in
        for i in range(steps + 1):
            if not self.animation_running:
                break
            brightness = i / steps
            faded_color = [int(c * brightness) for c in color]

            if led_id == 255:
                self.set_color(faded_color)
            else:
                self.set_color(faded_color, index=led_id)
            utime.sleep_ms(step_delay)

        # Fade out
        for i in range(steps, -1, -1):
            if not self.animation_running:
                break
            brightness = i / steps
            faded_color = [int(c * brightness) for c in color]

            if led_id == 255:
                self.set_color(faded_color)
            else:
                self.set_color(faded_color, index=led_id)
            utime.sleep_ms(step_delay)

    def _animate_rainbow(self, led_id, delay_ms):
        """Rainbow color cycle animation"""
        steps = 360
        step_delay = max(1, delay_ms // steps)

        for hue in range(steps):
            if not self.animation_running:
                break

            # Convert HSV to RGB
            rgb = self._hsv_to_rgb(hue, 255, 255)

            if led_id == 255:
                self.set_color(rgb)
            else:
                self.set_color(rgb, index=led_id)
            utime.sleep_ms(step_delay)

    def _animate_sequence(self, led_id, color, delay_ms):
        """Custom sequence animation (placeholder)"""
        # This could be extended for custom patterns
        self._animate_static(led_id, color)
        utime.sleep_ms(delay_ms)

    def _hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB color space"""
        h = h % 360
        s = s / 255.0
        v = v / 255.0

        c = v * s
        x = c * (1 - abs((h / 60) % 2 - 1))
        m = v - c

        if 0 <= h < 60:
            r, g, b = c, x, 0
        elif 60 <= h < 120:
            r, g, b = x, c, 0
        elif 120 <= h < 180:
            r, g, b = 0, c, x
        elif 180 <= h < 240:
            r, g, b = 0, x, c
        elif 240 <= h < 300:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x

        r = int((r + m) * 255)
        g = int((g + m) * 255)
        b = int((b + m) * 255)

        return [r, g, b]

    def _animate_blink_quick(self, led_id, color, delay_ms):
        """Quick blink animation - blink a few times"""
        blink_count = 3  # Number of blinks
        on_time = min(delay_ms // 2, 200)  # Time LED is on
        off_time = min(delay_ms // 2, 200)  # Time LED is off
        
        for i in range(blink_count):
            if not self.animation_running:
                break
                
            # Turn on
            if led_id == 255:
                self.set_color(color)
            else:
                self.set_color(color, index=led_id)
            utime.sleep_ms(on_time)
            
            # Turn off
            if led_id == 255:
                self.set_color([0, 0, 0])
            else:
                self.set_color([0, 0, 0], index=led_id)
            utime.sleep_ms(off_time)
        
        # Leave LED on at the end
        if led_id == 255:
            self.set_color(color)
        else:
            self.set_color(color, index=led_id)

    def _animate_fade_quick(self, led_id, color, delay_ms):
        """Quick fade animation - fade in and out"""
        steps = 10  # Number of fade steps
        step_delay = min(delay_ms // (steps * 2), 50)  # Delay per step
        
        # Fade in
        for i in range(steps):
            if not self.animation_running:
                break
                
            brightness = (i + 1) / steps
            faded_color = [int(c * brightness) for c in color]
            
            if led_id == 255:
                self.set_color(faded_color)
            else:
                self.set_color(faded_color, index=led_id)
            utime.sleep_ms(step_delay)
        
        # Fade out
        for i in range(steps):
            if not self.animation_running:
                break
                
            brightness = (steps - i) / steps
            faded_color = [int(c * brightness) for c in color]
            
            if led_id == 255:
                self.set_color(faded_color)
            else:
                self.set_color(faded_color, index=led_id)
            utime.sleep_ms(step_delay)
        
        # Leave LED on at the end
        if led_id == 255:
            self.set_color(color)
        else:
            self.set_color(color, index=led_id)

    def _animate_rainbow_quick(self, led_id, delay_ms):
        """Quick rainbow animation - show a few colors"""
        colors = [
            [255, 0, 0],    # Red
            [255, 127, 0],  # Orange
            [255, 255, 0],  # Yellow
            [0, 255, 0],    # Green
            [0, 0, 255],    # Blue
            [75, 0, 130],   # Indigo
            [148, 0, 211]   # Violet
        ]
        
        step_delay = max(10, delay_ms // len(colors))
        
        for color in colors:
            if not self.animation_running:
                break
                
            if led_id == 255:
                self.set_color(color)
            else:
                self.set_color(color, index=led_id)
            utime.sleep_ms(step_delay)


    def get_status(self):
        """Get current LED controller status

        Returns:
            dict: Status information including running state, queue length, etc.
        """
        return {
            "animation_running": self.animation_running,
            "queue_length": len(self.animation_queue),
            "last_executed_led_id": self.last_executed_led_id,
            "last_sequence_length": self.last_sequence_length,
            "brightness": self.np.brightness,
            "num_leds": self.num_leds,
        }

    def send_error_report(self, led_id, error_code, error_msg=""):
        """Send error report via completion callback

        Args:
            led_id: LED that had the error
            error_code: Numeric error code
            error_msg: Optional error message
        """
        if self.completion_callback:
            try:
                # Use negative sequence length to indicate error
                self.completion_callback(led_id, -error_code)
                if error_msg:
                    print(f"LED Error {error_code} on LED {led_id}: {error_msg}")
            except Exception as e:
                print(f"Error callback failed: {e}")

    def set_completion_callback(self, callback):
        """Set or update the completion callback function

        Args:
            callback: Function to call when sequences complete (led_id, sequence_length)
        """
        self.completion_callback = callback
