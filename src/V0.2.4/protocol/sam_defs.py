"""
SAM Protocol v2.0.0 Definitions
Protocol constants and message structures
"""

# Frame Delimiters and Special Bytes
FLAG = 0x7E
ESC = 0x7D
FLAG_ESC = 0x5E  # XOR mask for escaped FLAG
ESC_ESC = 0x5D  # XOR mask for escaped ESC

# Frame Address Field
ADDR_HOST = 0x01
ADDR_SAM = 0x02

# Frame Types (lower 4 bits of CTRL field) - Per SAM Protocol v2.0.0
TYPE_INFO = 0x0  # Information frame (carries data)
TYPE_ACK = 0x1  # Positive acknowledgment
TYPE_NAK = 0x2  # Negative acknowledgment
TYPE_SYNC = 0x3  # Synchronization request
TYPE_PING = 0x4  # Keepalive probe
TYPE_PONG = 0x5  # Keepalive response
TYPE_RESET = 0x6  # Reset sequence numbers
TYPE_FLOW = 0x7  # Flow control
TYPE_MASK = 0x0F  # Mask for frame type

# Control Field Bits
CTRL_SEQ_MASK = 0xF0  # Sequence number in bits 7-4

# Message Classes (Application Layer)
CLASS_SYSTEM = 0x00
CLASS_INPUT = 0x10
CLASS_OUTPUT = 0x20
CLASS_POWER = 0x30
CLASS_DISPLAY = 0x40
CLASS_DEBUG = 0x50

# System Messages (0x00 - 0x0F) - Must match kernel driver!
MSG_VERSION_REQ = 0x00
MSG_VERSION_RSP = 0x01
MSG_RESET_REQ = 0x02
MSG_RESET_ACK = 0x03
MSG_STATUS_REQ = 0x04
MSG_STATUS_RSP = 0x05
MSG_ERROR_REPORT = 0x06
MSG_SET_CONFIG = 0x07
MSG_GET_CONFIG = 0x08
MSG_HEARTBEAT = 0x09

# Input Messages (0x10 - 0x1F) - Match kernel expectations
MSG_BUTTON_EVENT = 0x10
MSG_ADC_VALUE = 0x11
MSG_GPIO_STATE = 0x12
MSG_GET_INPUT_STATE = 0x13

# Output Messages (0x20 - 0x2F) - LED support maintained
MSG_LED_SET = 0x20
MSG_PWM_SET = 0x21
MSG_GPIO_WRITE = 0x22
MSG_LED_PATTERN = 0x23
MSG_GET_OUTPUT_STATE = 0x24

# Power Messages (0x30 - 0x3F) - Battery monitoring
MSG_BATTERY_STATUS = 0x30
MSG_CHARGING_STATE = 0x31
MSG_POWER_MODE = 0x32

# Display Messages (0x40 - 0x4F) - E-ink support
MSG_DISPLAY_CLEAR = 0x40
MSG_DISPLAY_UPDATE = 0x41
MSG_DISPLAY_PARTIAL = 0x42
MSG_DISPLAY_STATUS = 0x43

# Debug Messages (0x50 - 0x5F)
MSG_DEBUG_LOG = 0x50
MSG_DEBUG_STATS = 0x51
MSG_DEBUG_DUMP = 0x52
MSG_DEBUG_TEST = 0x53

# Button Event Types
BUTTON_PRESS = 0x01
BUTTON_RELEASE = 0x02
BUTTON_LONG_PRESS = 0x03
BUTTON_DOUBLE_CLICK = 0x04

# Button IDs
BTN_ID_SELECT = 0x01
BTN_ID_UP = 0x02
BTN_ID_DOWN = 0x03
BTN_ID_POWER = 0x04

# LED Patterns
LED_PATTERN_OFF = 0x00
LED_PATTERN_SOLID = 0x01
LED_PATTERN_BLINK = 0x02
LED_PATTERN_PULSE = 0x03
LED_PATTERN_RAINBOW = 0x04
LED_PATTERN_ERROR = 0x05

# Power Modes
POWER_MODE_NORMAL = 0x00
POWER_MODE_LOW = 0x01
POWER_MODE_SLEEP = 0x02
POWER_MODE_SHUTDOWN = 0x03

# Error Codes
ERROR_NONE = 0x00
ERROR_CRC = 0x01
ERROR_TIMEOUT = 0x02
ERROR_OVERFLOW = 0x03
ERROR_UNDERFLOW = 0x04
ERROR_INVALID_MSG = 0x05
ERROR_PROTOCOL = 0x06
ERROR_HARDWARE = 0x07

# Connection States
STATE_DISCONNECTED = 0
STATE_SYNCING = 1
STATE_CONNECTED = 2
STATE_ERROR = 3

# Frame Processing States
FRAME_STATE_IDLE = 0
FRAME_STATE_ADDR = 1
FRAME_STATE_CTRL = 2
FRAME_STATE_LEN = 3
FRAME_STATE_DATA = 4
FRAME_STATE_CRC_H = 5
FRAME_STATE_CRC_L = 6
FRAME_STATE_END = 7
FRAME_STATE_ESCAPE = 8

# SYNC Parameters
SYNC_VERSION_MAJOR = 2
SYNC_VERSION_MINOR = 0
SYNC_VERSION_PATCH = 0
SYNC_WINDOW_SIZE = 4  # Default window size
