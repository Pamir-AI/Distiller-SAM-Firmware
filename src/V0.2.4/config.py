"""
RP2040 SAM Protocol v2.0.0 Configuration
Hardware pin definitions and system constants
"""

# UART Configuration
UART_ID = 0
UART_TX_PIN = 0
UART_RX_PIN = 1
UART_BAUDRATE = 115200

# Button Pins (with internal pull-ups)
BUTTON_SELECT = 16
BUTTON_UP = 17
BUTTON_DOWN = 18
BUTTON_POWER = 3  # Connected to PMIC

# I2C Configuration for Battery Monitor
I2C_ID = 0
I2C_SDA = 24
I2C_SCL = 25
I2C_FREQ = 100000
BQ27441_ADDR = 0x55

# NeoPixel LED Configuration
NEOPIXEL_PIN = 6
NEOPIXEL_COUNT = 8

# E-ink Display Configuration
EINK_SPI_ID = 1
EINK_SCK_PIN = 10
EINK_MOSI_PIN = 11
EINK_CS_PIN = 13
EINK_DC_PIN = 14
EINK_RST_PIN = 15
EINK_BUSY_PIN = 9
EINK_WIDTH = 256
EINK_HEIGHT = 128

# GPIO Control
GPIO_HOST_IRQ = 2  # Interrupt to host processor
GPIO_DEBUG_LED = 20  # Debug status LED

# Protocol Configuration
PROTOCOL_VERSION = 0x0200  # v2.0.0
MAX_FRAME_SIZE = 256
MAX_PAYLOAD_SIZE = 249
DEFAULT_WINDOW_SIZE = 4
MAX_WINDOW_SIZE = 8

# Protocol Timers (milliseconds)
T1_ACK_TIMEOUT = 1000  # Frame acknowledgment timeout
T2_IDLE_TIMEOUT = 5000  # Link idle timeout
T3_KEEPALIVE = 10000  # Keepalive interval
SYNC_RETRY_INTERVAL = 500  # SYNC negotiation retry

# Power Management
IDLE_SLEEP_TIMEOUT = 30000  # 30 seconds to sleep mode
BUTTON_DEBOUNCE_MS = 50
BATTERY_UPDATE_INTERVAL = 5000  # 5 seconds

# Debug Configuration
DEBUG_ENABLED = True
DEBUG_LEVEL = 2  # 0=off, 1=errors, 2=info, 3=verbose

# Message Priority Levels
PRIORITY_BATTERY = 4  # Highest
PRIORITY_EINK = 3
PRIORITY_LED = 2
PRIORITY_INPUT = 1  # Lowest

# Error Recovery
MAX_CONSECUTIVE_ERRORS = 10  # Auto-reset threshold
ERROR_LED_FLASH_MS = 100  # Error indication flash rate

# Buffer Sizes
RX_BUFFER_SIZE = 2048
TX_BUFFER_SIZE = 2048
UART_FIFO_SIZE = 64

# System Limits
MAX_PENDING_MESSAGES = 32
MAX_RETRANSMISSIONS = 3
MAX_SYNC_ATTEMPTS = 5
