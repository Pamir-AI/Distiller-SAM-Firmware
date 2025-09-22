"""
Button Driver with Debouncing and Event Detection
Handles button inputs with multiple event types
"""

import machine
import utime
import uasyncio
from protocol.sam_defs import *
import config


class Button:
    """Individual button with debouncing and event detection"""

    def __init__(self, pin_num, button_id, pull_up=True):
        self.pin = machine.Pin(
            pin_num, machine.Pin.IN, machine.Pin.PULL_UP if pull_up else None
        )
        self.button_id = button_id
        self.inverted = pull_up  # Pull-up means pressed = 0

        # State tracking
        self.state = self.read_raw()
        self.last_state = self.state
        self.last_change_time = utime.ticks_ms()
        self.press_start_time = 0
        self.press_count = 0
        self.double_click_time = 0

        # Thresholds
        self.debounce_ms = config.BUTTON_DEBOUNCE_MS
        self.long_press_ms = 1000
        self.double_click_ms = 300

    def read_raw(self):
        """Read raw button state"""
        value = self.pin.value()
        return not value if self.inverted else value

    def update(self):
        """Update button state and detect events"""
        current = self.read_raw()
        current_time = utime.ticks_ms()
        events = []

        # Check for state change
        if current != self.last_state:
            # Debounce
            if (
                utime.ticks_diff(current_time, self.last_change_time)
                >= self.debounce_ms
            ):
                self.last_change_time = current_time
                self.last_state = current

                if current and not self.state:
                    # Button pressed
                    self.state = True
                    self.press_start_time = current_time
                    events.append((self.button_id, BUTTON_PRESS))

                    # Check for double click
                    if self.double_click_time > 0:
                        if (
                            utime.ticks_diff(current_time, self.double_click_time)
                            < self.double_click_ms
                        ):
                            events.append((self.button_id, BUTTON_DOUBLE_CLICK))
                            self.double_click_time = 0
                        else:
                            self.double_click_time = current_time
                    else:
                        self.double_click_time = current_time

                elif not current and self.state:
                    # Button released
                    self.state = False
                    events.append((self.button_id, BUTTON_RELEASE))

                    # Check if it was a long press
                    press_duration = utime.ticks_diff(
                        current_time, self.press_start_time
                    )
                    if press_duration >= self.long_press_ms:
                        events.append((self.button_id, BUTTON_LONG_PRESS))

        # Check for long press while held
        elif self.state and current:
            press_duration = utime.ticks_diff(current_time, self.press_start_time)
            if press_duration >= self.long_press_ms:
                # Only send long press once
                if self.press_count == 0:
                    events.append((self.button_id, BUTTON_LONG_PRESS))
                    self.press_count = 1

        # Clear double click timeout
        if self.double_click_time > 0:
            if (
                utime.ticks_diff(current_time, self.double_click_time)
                > self.double_click_ms
            ):
                self.double_click_time = 0

        return events

    def is_pressed(self):
        """Check if button is currently pressed"""
        return self.state

    def reset(self):
        """Reset button state"""
        self.state = self.read_raw()
        self.last_state = self.state
        self.press_count = 0
        self.double_click_time = 0


class ButtonDriver:
    """Driver for multiple buttons with event handling"""

    def __init__(self):
        # Create button instances
        self.buttons = {
            BTN_ID_SELECT: Button(config.BUTTON_SELECT, BTN_ID_SELECT),
            BTN_ID_UP: Button(config.BUTTON_UP, BTN_ID_UP),
            BTN_ID_DOWN: Button(config.BUTTON_DOWN, BTN_ID_DOWN),
            BTN_ID_POWER: Button(config.BUTTON_POWER, BTN_ID_POWER),
        }

        # Event callback
        self.on_button_event = None

        # Task control
        self.running = False
        self.poll_task_handle = None

        # Statistics
        self.event_count = 0

    def start(self):
        """Start button driver"""
        if not self.running:
            self.running = True
            self.poll_task_handle = uasyncio.create_task(self.poll_task())
            print("ButtonDriver: Started")

    def stop(self):
        """Stop button driver"""
        self.running = False
        if self.poll_task_handle:
            self.poll_task_handle.cancel()
            self.poll_task_handle = None
        print("ButtonDriver: Stopped")

    async def poll_task(self):
        """Polling task"""
        while self.running:
            try:
                # Poll all buttons
                for button in self.buttons.values():
                    events = button.update()
                    for button_id, event_type in events:
                        self.event_count += 1
                        if self.on_button_event:
                            self.on_button_event(button_id, event_type)

                # Cooperative yield
                await uasyncio.sleep_ms(10)

            except Exception as e:
                print(f"Button poll error: {e}")

    def get_button_states(self):
        """Get current state of all buttons as bitmask"""
        states = 0
        if self.buttons[BTN_ID_SELECT].is_pressed():
            states |= 1 << 0
        if self.buttons[BTN_ID_UP].is_pressed():
            states |= 1 << 1
        if self.buttons[BTN_ID_DOWN].is_pressed():
            states |= 1 << 2
        if self.buttons[BTN_ID_POWER].is_pressed():
            states |= 1 << 3
        return states

    def is_pressed(self, button_id):
        """Check if specific button is pressed"""
        if button_id in self.buttons:
            return self.buttons[button_id].is_pressed()
        return False

    def reset_all(self):
        """Reset all button states"""
        for button in self.buttons.values():
            button.reset()

    def get_stats(self):
        """Get button statistics"""
        return {"event_count": self.event_count, "states": self.get_button_states()}


class ButtonSimulator:
    """Button simulator for testing"""

    def __init__(self):
        self.on_button_event = None
        self.states = 0
        self.event_count = 0

    def start(self):
        print("ButtonSimulator: Started")

    def stop(self):
        print("ButtonSimulator: Stopped")

    def simulate_press(self, button_id):
        """Simulate button press"""
        self.event_count += 1
        if button_id == BTN_ID_SELECT:
            self.states |= 1 << 0
        elif button_id == BTN_ID_UP:
            self.states |= 1 << 1
        elif button_id == BTN_ID_DOWN:
            self.states |= 1 << 2
        elif button_id == BTN_ID_POWER:
            self.states |= 1 << 3

        if self.on_button_event:
            self.on_button_event(button_id, BUTTON_PRESS)

    def simulate_release(self, button_id):
        """Simulate button release"""
        self.event_count += 1
        if button_id == BTN_ID_SELECT:
            self.states &= ~(1 << 0)
        elif button_id == BTN_ID_UP:
            self.states &= ~(1 << 1)
        elif button_id == BTN_ID_DOWN:
            self.states &= ~(1 << 2)
        elif button_id == BTN_ID_POWER:
            self.states &= ~(1 << 3)

        if self.on_button_event:
            self.on_button_event(button_id, BUTTON_RELEASE)

    def get_button_states(self):
        """Get simulated button states"""
        return self.states

    def is_pressed(self, button_id):
        """Check if button is pressed in simulation"""
        if button_id == BTN_ID_SELECT:
            return (self.states & (1 << 0)) != 0
        elif button_id == BTN_ID_UP:
            return (self.states & (1 << 1)) != 0
        elif button_id == BTN_ID_DOWN:
            return (self.states & (1 << 2)) != 0
        elif button_id == BTN_ID_POWER:
            return (self.states & (1 << 3)) != 0
        return False

    def reset_all(self):
        """Reset simulation"""
        self.states = 0

    def get_stats(self):
        """Get statistics"""
        return {"event_count": self.event_count, "states": self.states}
