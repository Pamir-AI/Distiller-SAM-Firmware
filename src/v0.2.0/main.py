#!/usr/bin/env micropython
"""
Pamir AI Signal Aggregation Module (SAM) Firmware
=================================================

This firmware runs on the RP2040 microcontroller and provides communication
between the main Linux system and hardware components on Pamir AI devices.

Features:
--------
- Button input handling and debounce
- LED control with animations
- Power state management (boot/shutdown coordination)
- Poll-based power metrics reporting
- E-Ink display controller interface
- Version information exchange
- Debug logging via UART

Version Information:
------------------
The firmware exchanges version information with the Linux driver during boot:
1. Linux driver sends its version (major, minor, patch) to the firmware
2. Firmware stores this for potential compatibility checks
3. Firmware responds with its own version information

Power Metrics:
------------
The firmware supports poll-based power metrics reporting:
1. Linux driver requests metrics via POWER_CMD_REQUEST_METRICS command
2. Firmware responds with current, voltage, temperature, and battery info
3. Linux driver exposes these values through sysfs

Author: PamirAI
Date: 2025-05-19
Version: 0.1.0
"""

# TODO EINK BLOCK CORE 1 from complete if eink broken
import machine
import utime
from eink_driver_sam import einkDSP_SAM
import _thread
import neopixel
from uart_protocol import *
from version import *

# # Reset PMIC, DO NO REMOVE THIS BLOCK, Covers Non Battery Non Boost Version Board
# pmic_enable.value(0) # Pull down pin
# utime.sleep(0.01) # Keep low for 0.01 second
# pmic_enable.init(mode=machine.Pin.IN)
# # END OF PMIC RESET BLOCK

PRODUCTION = True  # for production flash, set to true for usb debug
PRINT_DEBUG_UART = True  # for UART debug, set to true for UART debug
LUT_MODE = True  # for LUT mode, set to true for LUT mode
debounce_time = 50  # Debounce time in milliseconds


wdt = machine.WDT(timeout=2000)
# Set up GPIO pins
selectBTN = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_DOWN)
upBTN = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_DOWN)
downBTN = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_DOWN)
einkStatus = machine.Pin(9, machine.Pin.OUT, value=1)
einkMux = machine.Pin(22, machine.Pin.OUT, value=1)
nukeUSB = machine.Pin(19, machine.Pin.OUT, value=0)
pmic_enable = machine.Pin(3, machine.Pin.OUT)
i2c = machine.I2C(0, sda=machine.Pin(24), scl=machine.Pin(25))

# Check if battery gauge is present at address 85 (0x55)
devices = i2c.scan()
BATTERY_MODE = 85 in devices

if BATTERY_MODE:
    from battery import BQ27441

    battery = BQ27441(i2c)
    battery.initialise(
        design_capacity_mAh=3000, terminate_voltage_mV=3200, CALIBRATION=True
    )  # learn on this board


if PRODUCTION:
    nukeUSB.high()  # Disable SAM USB

# Setup UART0 on GPIO0 (TX) and GPIO1 (RX)
uart0 = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))
einkMux.low()  # EINK OFF
einkStatus.low()  # SOM CONTROL E-INK
eink = einkDSP_SAM()  # Initialize eink

# Add lock for shared variable
power_status = False
core1_task_interrupt_lock = _thread.allocate_lock()
core1_task_interrupt = False
eink_lock = _thread.allocate_lock()
einkRunning = False
neopixel_lock = _thread.allocate_lock()  # Add lock for neopixel operations
uart_lock = _thread.allocate_lock()  # Add lock for UART handling
current_neopixel_thread = None  # Track current neopixel thread
neopixel_running = False  # Flag to control current neopixel sequence

# Initialize protocol with our UART
pamir_protocol = PamirProtocol(uart0, debug=PRINT_DEBUG_UART, wdt=wdt)

# Last power report time for periodic reporting
last_power_report = utime.ticks_ms()
power_report_interval = 30000  # 30 seconds


def send_button_state():
    state_byte = 0
    state_byte |= get_debounced_state(selectBTN) * BTN_SELECT_MASK
    state_byte |= get_debounced_state(upBTN) * BTN_UP_MASK
    state_byte |= get_debounced_state(downBTN) * BTN_DOWN_MASK

    debug_print(f"BUTTON STATE: {state_byte}")
    pamir_protocol.send_button_state(state_byte)


# Interrupt handler for down button
def button_handler(pin):
    if debounce(pin):
        send_button_state()


# Set up interrupt handlers
selectBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler
)
upBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler
)
downBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler
)


print("Starting...")


def power_on_som():
    global power_status
    pmic_enable.init(mode=machine.Pin.IN)
    power_status = True


def power_off_som():
    global power_status
    pmic_enable.init(mode=machine.Pin.OPEN_DRAIN)
    pmic_enable.low()  # Pull down pin
    power_status = False


# Function to handle UART debug messages
def debug_print(message):
    if PRINT_DEBUG_UART:
        pamir_protocol.send_debug_text(message)
    print(message)


# Begin of neopixel
def init_neopixel(pin=20, num_leds=1, brightness=1.0):
    np = neopixel.NeoPixel(machine.Pin(pin), num_leds)
    np.brightness = min(max(brightness, 0.0), 1.0)  # 限制亮度范围
    return np


# Neopixel set color
def set_color(np, color, brightness=None, index=None):
    if brightness is not None:
        np.brightness = min(max(brightness, 0.0), 1.0)
    r = int(color[0] * np.brightness)
    g = int(color[1] * np.brightness)
    b = int(color[2] * np.brightness)
    if index is None:
        for i in range(len(np)):
            np[i] = (r, g, b)
    else:
        np[index] = (r, g, b)
    np.write()


# LED handler for new protocol
def handle_led_packet(packet):
    cmd_type = packet[0] & LED_CMD_EXECUTE  # Execute or Queue
    led_id = packet[0] & LED_ID_MASK
    r = (packet[1] >> 4) & 0x0F
    g = packet[1] & 0x0F
    b = (packet[2] >> 4) & 0x0F
    time_value = packet[2] & 0x0F

    # Scale RGB values to 0-255 range
    r_scaled = (r * 255) // 15
    g_scaled = (g * 255) // 15
    b_scaled = (b * 255) // 15

    debug_print(
        f"LED packet: cmd={hex(cmd_type)}, id={led_id}, RGB=({r_scaled},{g_scaled},{b_scaled})"
    )

    # Only handle LED 0 for compatibility with existing hardware
    if led_id == 0:
        with neopixel_lock:
            set_color(np, [r_scaled, g_scaled, b_scaled], time_value / 15, 0)

    # For sequence commands, respond with completion
    if cmd_type == LED_CMD_EXECUTE:  # Execute
        sequence_length = 1  # Default to 1 for backwards compatibility
        pamir_protocol.send_led_completion(led_id, sequence_length)


# Power packet handler
def handle_power_packet(packet):
    command = packet[0] & POWER_CMD_MASK  # Extract command bits
    param = packet[0] & 0x0F
    data1 = packet[1]
    data2 = packet[2]

    debug_print(
        f"Power packet: cmd={hex(command)}, param={hex(param)}, data=[{hex(data1)},{hex(data2)}]"
    )

    if command == POWER_CMD_SHUTDOWN:  # POWER_CMD_SHUTDOWN
        debug_print("Received shutdown command")
        # Start shutdown sequence
        einkMux.high()  # SAM CONTROL E-INK
        einkStatus.low()  # disable power to eink

        # Send acknowledgment
        pamir_protocol.send_power_shutdown_ack(data1)

        if PRODUCTION:
            nukeUSB.low()  # Rejoin SAM to USB

        # Signal core1 task to stop
        with core1_task_interrupt_lock:
            core1_task_interrupt = True

    elif command == POWER_CMD_REQUEST_METRICS:
        debug_print("Received request for power metrics")
        # Send power metrics on demand
        send_power_metrics()


# System packet handler
def handle_system_packet(packet):
    action = packet[0] & 0x1F

    if action == SYSTEM_PING:  # SYSTEM_PING
        pamir_protocol.send_ping_response()
    elif action == SYSTEM_VERSION:  # SYSTEM_VERSION
        # If this is a version packet with data (not a request)
        if packet[1] != 0 or packet[2] != 0:
            # Log version information
            debug_print(f"Host driver version: {pamir_protocol.host_version['string']}")

            # Check compatibility
            is_compatible = check_compatibility(
                (
                    pamir_protocol.host_version["major"],
                    pamir_protocol.host_version["minor"],
                    pamir_protocol.host_version["patch"],
                )
            )
            debug_print(
                f"Host driver compatibility: {'OK' if is_compatible else 'WARNING'}"
            )
        else:
            # This is a version request - send our firmware version
            pamir_protocol.send_version_info(VERSION_MAJOR, VERSION_MINOR)


def handle_neopixel_sequence(np, data):
    global neopixel_running

    if not isinstance(data, dict) or "colors" not in data:
        debug_print("[RP2040 DEBUG] Invalid data format or missing 'colors' key\n")
        return

    neopixel_running = True
    colors = data.get("colors", {})
    debug_print(f"[RP2040 DEBUG] Processing {len(colors)} color sequences\n")

    # Sort the sequence numbers to process them in order
    sequence_numbers = sorted([int(k) for k in colors.keys()])

    for seq_num in sequence_numbers:
        if not neopixel_running:  # Check if we should terminate
            break
        try:
            color_data = colors[str(seq_num)]
            if len(color_data) >= 5:
                r, g, b, brightness, delay = color_data
                debug_print(
                    f"[RP2040 DEBUG] Sequence {seq_num}: Setting LED to RGB({r},{g},{b}) with brightness {brightness}\n"
                )
                with neopixel_lock:
                    # Always set the first LED (index 0)
                    set_color(np, [r, g, b], brightness, 0)
                debug_print(
                    f"[RP2040 DEBUG] Color set: {r}, {g}, {b}, brightness: {brightness}, delay: {delay}\n"
                )
                utime.sleep(delay)
        except (ValueError, IndexError) as e:
            error_msg = f"Error processing sequence {seq_num}: {e}"
            print(error_msg)
            debug_print(f"[RP2040 DEBUG] {error_msg}\n")

    neopixel_running = False


# Function to debounce button press
def debounce(pin):
    state = pin.value()
    utime.sleep_ms(debounce_time)
    if pin.value() != state:
        return False
    return True


def get_debounced_state(pin):
    return pin.value() and debounce(pin)


# Initialize neopixel with just 1 LED
np = init_neopixel(pin=20, num_leds=1, brightness=0.5)
debug_print(f"[RP2040 DEBUG] Initialized {len(np)} NeoPixel\n")
neopixel_running = True


# Thread to handle both eink and UART tasks
def core1_task():
    global einkRunning, neopixel_running, core1_task_interrupt

    # First, run the eink task
    try:
        einkRunning = True
        if eink.init == False:
            eink.re_init()

        if LUT_MODE:
            eink.epd_init_lut()
        else:
            eink.epd_init_fast()

        try:
            eink.PIC_display(None, "./loading1.bin")
        except OSError:
            print("Loading files not found")
            einkRunning = False

        if LUT_MODE:
            utime.sleep_ms(1300)  # give time for first refresh, no lower than 1300

        repeat = 0
        while True:
            with eink_lock:
                if not einkRunning or repeat >= 3:
                    break
            eink.epd_init_part()
            eink.PIC_display("./loading1.bin", "./loading2.bin")
            eink.epd_init_part()
            eink.PIC_display("./loading2.bin", "./loading1.bin")
            wdt.feed()
            repeat += 1

        eink.de_init()
        with eink_lock:
            einkRunning = False
        einkMux.low()
        print("Eink Task Completed")
    except Exception as e:
        print(f"Exception in eink task: {e}")
        eink.de_init()
        with eink_lock:
            einkRunning = False
        einkMux.low()

    # Now handle UART and protocol in a continuous loop
    debug_print("[RP2040 DEBUG] Starting protocol handling on core1\n")

    # Log version information
    debug_print(f"Pamir AI SAM Firmware v{VERSION_STRING}")
    log_version_info(debug_print)

    # Register handlers for packet types
    pamir_protocol.register_handler(TYPE_LED, handle_led_packet)  # LED handler
    pamir_protocol.register_handler(TYPE_POWER, handle_power_packet)  # Power handler
    pamir_protocol.register_handler(TYPE_SYSTEM, handle_system_packet)  # System handler
    pamir_protocol.register_handler(
        TYPE_EXTENDED, handle_extended_packet
    )  # Extended handler

    # UART handling loop
    while True:
        with core1_task_interrupt_lock:
            if core1_task_interrupt:
                break

        # Check for and process packets
        pamir_protocol.check_uart()

        wdt.feed()
        utime.sleep_ms(1)

    debug_print("Core1 task completed\n")


def check_for_power_on():
    global einkRunning, core1_task_interrupt
    if (
        debounce(selectBTN)
        and selectBTN.value() == 1
        and upBTN.value() == 0
        and downBTN.value() == 0
    ):
        start_time = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_time) < 2000:
            if selectBTN.value() == 0:
                break
            wdt.feed()
            utime.sleep_ms(10)
        if (
            utime.ticks_diff(utime.ticks_ms(), start_time) >= 2000
            and power_status == False
        ):
            power_on_som()
            einkMux.high()  # SAM CONTROL E-INK
            einkStatus.high()  # provide power to eink
            if PRODUCTION:
                nukeUSB.high()  # Disable SAM USB

            # Turn on the power on loading screen
            print("einkRunning: ", einkRunning)
            if einkRunning == False:
                try:
                    # Reset interrupt flag before starting new thread
                    with core1_task_interrupt_lock:
                        core1_task_interrupt = False

                    # Ensure eink is properly initialized
                    einkRunning = True
                    _thread.start_new_thread(core1_task, ())
                    print("Started core1 task")
                except Exception as e:
                    print(f"Exception {e}")
                    eink.de_init()
                    einkRunning = False
                    # Reset interrupt flag in case of error
                    with core1_task_interrupt_lock:
                        core1_task_interrupt = False


def check_for_power_off():
    global core1_task_interrupt
    if debounce(upBTN) and selectBTN.value() == 1:
        start_time = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_time) < 8000:
            if upBTN.value() == 0 or selectBTN.value() == 0:
                break
            if utime.ticks_diff(utime.ticks_ms(), start_time) >= 2000:
                # Send shutdown command using the new protocol
                pamir_protocol.send_power_shutdown(SHUTDOWN_MODE_NORMAL)
                debug_print("Sent shutdown command")
            wdt.feed()
            utime.sleep_ms(10)
        if utime.ticks_diff(utime.ticks_ms(), start_time) >= 8000:
            # After 8 seconds, turn off the eink and mux. This will be a forced shutdown.
            einkMux.high()  # SAM CONTROL E-INK
            einkStatus.low()  # disable power to eink
            power_off_som()

            if PRODUCTION:
                nukeUSB.low()  # Rejoin SAM to USB
            with core1_task_interrupt_lock:
                core1_task_interrupt = True


def report_power_metrics():
    """Report power metrics to the host"""
    global last_power_report

    current_time = utime.ticks_ms()

    # Only send if enough time has passed and Linux has booted
    if (
        pamir_protocol.linux_booted
        and utime.ticks_diff(current_time, last_power_report) > power_report_interval
    ):
        if BATTERY_MODE:
            try:
                # Get real battery data
                soc = battery.remain_capacity()
                voltage_v = battery.voltage_V()
                temp_c = battery.temp_C()
                current_ma = abs(battery.avg_current_mA())

                # Convert to correct formats
                voltage_mv = int(voltage_v * 1000)
                temp_decidegc = int(temp_c * 10)

                # Send metrics using new protocol
                pamir_protocol.send_power_current(current_ma)
                pamir_protocol.send_power_battery(soc)
                pamir_protocol.send_power_temperature(temp_decidegc)
                pamir_protocol.send_power_voltage(voltage_mv)

                debug_print(
                    f"SOC: {soc}%  V: {voltage_v}V  T: {temp_c}°C  A: {current_ma}mA"
                )
            except Exception as e:
                debug_print(f"Battery read error: {e}")
                print(f"Battery read error: {e}")
        else:
            # Send simulated values if no battery
            pamir_protocol.send_power_current(250)
            pamir_protocol.send_power_battery(75)
            pamir_protocol.send_power_temperature(255)  # 25.5°C
            pamir_protocol.send_power_voltage(3800)

        last_power_report = current_time


def send_power_metrics():
    """Send current power metrics to host on demand"""
    if BATTERY_MODE:
        try:
            # Get real battery data
            soc = battery.remain_capacity()
            voltage_v = battery.voltage_V()
            temp_c = battery.temp_C()
            current_ma = abs(battery.avg_current_mA())

            # Convert to correct formats
            voltage_mv = int(voltage_v * 1000)
            temp_decidegc = int(temp_c * 10)

            # Send metrics using new protocol
            pamir_protocol.send_power_current(current_ma)
            pamir_protocol.send_power_battery(soc)
            pamir_protocol.send_power_temperature(temp_decidegc)
            pamir_protocol.send_power_voltage(voltage_mv)

            debug_print(
                f"SOC: {soc}%  V: {voltage_v}V  T: {temp_c}°C  A: {current_ma}mA"
            )
        except Exception as e:
            debug_print(f"Battery read error: {e}")
            print(f"Battery read error: {e}")
    else:
        # Send simulated values if no battery
        pamir_protocol.send_power_current(250)
        pamir_protocol.send_power_battery(75)
        pamir_protocol.send_power_temperature(255)  # 25.5°C
        pamir_protocol.send_power_voltage(3800)


# Extended packet handler
def handle_extended_packet(packet):
    ext_type = packet[0] & 0x1F

    if ext_type == 0x01:  # Extended version info
        debug_print(f"Received extended version info: patch={packet[1]}")
        debug_print(f"Complete host version: {pamir_protocol.host_version['string']}")

        # Log firmware version information
        log_version_info(debug_print)

        # Here you could perform version-specific initialization if needed
        if check_compatibility(pamir_protocol.host_version["string"]):
            # Enable advanced features for compatible host versions
            debug_print("Host driver supports all firmware features")
        else:
            # Fall back to basic functionality for older host versions
            debug_print("Host driver requires compatibility mode")


# Send system initialized debug code
pamir_protocol.send_debug_code(DEBUG_CAT_SYSTEM, 0x01, 0x00)  # System initialized

# Clean main loop
while True:
    wdt.feed()

    check_for_power_on()
    check_for_power_off()

    # Check for and process packets in main loop too
    pamir_protocol.check_uart()

    utime.sleep_ms(1)
