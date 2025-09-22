"""
E-ink Display Driver
Support for e-ink display via SPI
"""

import machine
import utime
import framebuf
import config

# E-ink Commands
CMD_DRIVER_OUTPUT = 0x01
CMD_BOOSTER_SOFT_START = 0x0C
CMD_GATE_SCAN = 0x0F
CMD_DEEP_SLEEP = 0x10
CMD_DATA_ENTRY_MODE = 0x11
CMD_SW_RESET = 0x12
CMD_TEMPERATURE_SENSOR = 0x1A
CMD_MASTER_ACTIVATION = 0x20
CMD_DISPLAY_UPDATE = 0x22
CMD_WRITE_RAM = 0x24
CMD_WRITE_VCOM = 0x2C
CMD_WRITE_LUT_REGISTER = 0x32
CMD_SET_DUMMY_LINE = 0x3A
CMD_SET_GATE_TIME = 0x3B
CMD_BORDER_WAVEFORM = 0x3C
CMD_SET_RAM_X = 0x44
CMD_SET_RAM_Y = 0x45
CMD_SET_RAM_X_COUNTER = 0x4E
CMD_SET_RAM_Y_COUNTER = 0x4F
CMD_TERMINATE_WRITE = 0xFF


class EinkDriver:
    """E-ink display driver"""

    def __init__(self):
        # Initialize SPI
        self.spi = machine.SPI(
            config.EINK_SPI_ID,
            baudrate=2000000,
            polarity=0,
            phase=0,
            sck=machine.Pin(config.EINK_SCK_PIN),
            mosi=machine.Pin(config.EINK_MOSI_PIN),
        )

        # Initialize control pins
        self.cs = machine.Pin(config.EINK_CS_PIN, machine.Pin.OUT, value=1)
        self.dc = machine.Pin(config.EINK_DC_PIN, machine.Pin.OUT)
        self.rst = machine.Pin(config.EINK_RST_PIN, machine.Pin.OUT)
        self.busy = machine.Pin(config.EINK_BUSY_PIN, machine.Pin.IN)

        # Display parameters
        self.width = config.EINK_WIDTH
        self.height = config.EINK_HEIGHT

        # Frame buffer
        self.buffer = bytearray(self.width * self.height // 8)
        print(f"EinkDriver: {self.width}x{self.height} display")
        self.framebuf = framebuf.FrameBuffer(
            self.buffer, self.width, self.height, framebuf.MONO_HLSB
        )

        # State
        self.initialized = False
        self.partial_mode = False

        # Initialize display
        self.init_display()

    def init_display(self):
        """Initialize e-ink display"""
        try:
            # Hardware reset
            self.reset()

            # Driver output control
            self.send_command(CMD_DRIVER_OUTPUT)
            self.send_data((self.height - 1) & 0xFF)
            self.send_data(((self.height - 1) >> 8) & 0xFF)
            self.send_data(0x00)  # GD = 0; SM = 0; TB = 0;

            # Booster soft start control
            self.send_command(CMD_BOOSTER_SOFT_START)
            self.send_data(0xD7)
            self.send_data(0xD6)
            self.send_data(0x9D)

            # VCOM voltage
            self.send_command(CMD_WRITE_VCOM)
            self.send_data(0xA8)

            # Set dummy line period
            self.send_command(CMD_SET_DUMMY_LINE)
            self.send_data(0x1A)

            # Set gate line width
            self.send_command(CMD_SET_GATE_TIME)
            self.send_data(0x08)

            # Data entry mode
            self.send_command(CMD_DATA_ENTRY_MODE)
            self.send_data(0x03)  # X increment, Y increment

            # Set RAM area
            self.set_memory_area(0, 0, self.width - 1, self.height - 1)
            self.set_memory_pointer(0, 0)

            self.initialized = True
            print("EinkDriver: Initialized")
            return True

        except Exception as e:
            print(f"EinkDriver: Init failed - {e}")
            self.initialized = False
            return False

    def reset(self):
        """Hardware reset"""
        self.rst.value(0)
        utime.sleep_ms(10)
        self.rst.value(1)
        utime.sleep_ms(10)
        self.wait_until_idle()

    def send_command(self, command):
        """Send command to display"""
        self.dc.value(0)
        self.cs.value(0)
        self.spi.write(bytes([command]))
        self.cs.value(1)

    def send_data(self, data):
        """Send data to display"""
        self.dc.value(1)
        self.cs.value(0)
        if isinstance(data, int):
            self.spi.write(bytes([data]))
        else:
            self.spi.write(data)
        self.cs.value(1)

    def wait_until_idle(self):
        """Wait for display to be ready"""
        timeout = 5000  # 5 second timeout
        while self.busy.value() == 1 and timeout > 0:
            utime.sleep_ms(10)
            timeout -= 10
        if timeout <= 0:
            print("EinkDriver: Timeout waiting for idle")

    def set_memory_area(self, x_start, y_start, x_end, y_end):
        """Set display memory area"""
        self.send_command(CMD_SET_RAM_X)
        self.send_data((x_start >> 3) & 0xFF)
        self.send_data((x_end >> 3) & 0xFF)

        self.send_command(CMD_SET_RAM_Y)
        self.send_data(y_start & 0xFF)
        self.send_data((y_start >> 8) & 0xFF)
        self.send_data(y_end & 0xFF)
        self.send_data((y_end >> 8) & 0xFF)

    def set_memory_pointer(self, x, y):
        """Set memory pointer position"""
        self.send_command(CMD_SET_RAM_X_COUNTER)
        self.send_data((x >> 3) & 0xFF)

        self.send_command(CMD_SET_RAM_Y_COUNTER)
        self.send_data(y & 0xFF)
        self.send_data((y >> 8) & 0xFF)

    def turn_on_display(self):
        """Refresh display"""
        self.send_command(CMD_DISPLAY_UPDATE)
        self.send_data(0xC4)
        self.send_command(CMD_MASTER_ACTIVATION)
        self.send_command(CMD_TERMINATE_WRITE)
        self.wait_until_idle()

    def clear(self):
        """Clear display to white"""
        if not self.initialized:
            return

        self.framebuf.fill(1)  # White
        self.update()

    def update(self):
        """Update full display"""
        if not self.initialized:
            return

        self.set_memory_area(0, 0, self.width - 1, self.height - 1)
        self.set_memory_pointer(0, 0)
        self.send_command(CMD_WRITE_RAM)
        self.send_data(self.buffer)
        self.turn_on_display()

    def partial_update(self, x, y, w, h):
        """Update partial area of display"""
        if not self.initialized:
            return

        # Set update area
        self.set_memory_area(x, y, x + w - 1, y + h - 1)
        self.set_memory_pointer(x, y)

        # Send partial data
        self.send_command(CMD_WRITE_RAM)
        # Extract partial buffer data
        for row in range(y, y + h):
            row_start = (row * self.width + x) // 8
            row_end = (row * self.width + x + w) // 8 + 1
            self.send_data(self.buffer[row_start:row_end])

        self.turn_on_display()

    def sleep(self):
        """Put display into deep sleep mode"""
        self.send_command(CMD_DEEP_SLEEP)
        self.send_data(0x01)

    def wake(self):
        """Wake display from sleep"""
        self.reset()
        self.init_display()

    def is_busy(self):
        """Check if display is busy"""
        return self.busy.value() == 1

    # Drawing functions using framebuf
    def fill(self, color):
        """Fill display with color (0=black, 1=white)"""
        self.framebuf.fill(color)

    def pixel(self, x, y, color):
        """Set pixel color"""
        self.framebuf.pixel(x, y, color)

    def hline(self, x, y, w, color):
        """Draw horizontal line"""
        self.framebuf.hline(x, y, w, color)

    def vline(self, x, y, h, color):
        """Draw vertical line"""
        self.framebuf.vline(x, y, h, color)

    def line(self, x0, y0, x1, y1, color):
        """Draw line"""
        self.framebuf.line(x0, y0, x1, y1, color)

    def rect(self, x, y, w, h, color):
        """Draw rectangle"""
        self.framebuf.rect(x, y, w, h, color)

    def fill_rect(self, x, y, w, h, color):
        """Draw filled rectangle"""
        self.framebuf.fill_rect(x, y, w, h, color)

    def text(self, string, x, y, color=0):
        """Draw text"""
        self.framebuf.text(string, x, y, color)

    def show_message(self, message, line=0):
        """Show message on display"""
        self.fill(1)  # Clear to white
        y_pos = line * 10
        self.text(message, 0, y_pos, 0)  # Black text
        self.update()


class EinkSimulator:
    """E-ink display simulator for testing"""

    def __init__(self):
        self.width = config.EINK_WIDTH
        self.height = config.EINK_HEIGHT
        self.buffer = bytearray(self.width * self.height // 8)
        self.initialized = True
        self.busy_state = False
        print(f"EinkSimulator: {self.width}x{self.height} display")

    def clear(self):
        """Simulate clear"""
        self.buffer = bytearray(len(self.buffer))
        print("Eink: Cleared")

    def update(self):
        """Simulate update"""
        self.busy_state = True
        utime.sleep_ms(100)  # Simulate refresh time
        self.busy_state = False
        print("Eink: Updated")

    def partial_update(self, x, y, w, h):
        """Simulate partial update"""
        self.busy_state = True
        utime.sleep_ms(50)  # Simulate partial refresh time
        self.busy_state = False
        print(f"Eink: Partial update ({x},{y},{w},{h})")

    def is_busy(self):
        """Simulate busy state"""
        return self.busy_state

    def sleep(self):
        """Simulate sleep"""
        print("Eink: Sleep")

    def wake(self):
        """Simulate wake"""
        print("Eink: Wake")

    def show_message(self, message, line=0):
        """Simulate message display"""
        print(f"Eink: Line {line}: {message}")
