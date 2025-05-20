# Author: PamirAI
# Date: 2025-05-05
# Version: 0.1.0
# Description: This is the main program for the RP2040 SAM

# TODO EINK BLOCK CORE 1 from complete if eink broken
import machine
import utime
from eink_driver_sam import einkDSP_SAM
import _thread
from machine import WDT
import math
import neopixel
import json
from uart_protocol import PamirProtocol

# # Reset PMIC, DO NO REMOVE THIS BLOCK, Covers Non Battery Non Boost Version Board
# pmic_enable.value(0) # Pull down pin
# utime.sleep(0.01) # Keep low for 0.01 second
# pmic_enable.init(mode=machine.Pin.IN)
# # END OF PMIC RESET BLOCK

PRODUCTION = True  # for production flash, set to true for usb debug
PRINT_DEBUG_UART = True  # for UART debug, set to true for UART debug
LUT_MODE = True  # for LUT mode, set to true for LUT mode
debounce_time = 50  # Debounce time in milliseconds


wdt = WDT(timeout=2000)
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

BTN_UP_MASK = 0x01
BTN_DOWN_MASK = 0x02
BTN_SELECT_MASK = 0x04
BTN_POWER_MASK = 0x08

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

pamir_protocol = PamirProtocol(uart0, debug=False, wdt=wdt)


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

    # Now run the UART task
    debug_print("[RP2040 DEBUG] Starting UART handling on core1\n")
    uart_buffer = ""

    # UART handling loop
    while False:
        with core1_task_interrupt_lock:
            if core1_task_interrupt:
                break
        try:
            if uart0.any():
                raw_data = uart0.read(1)
                if raw_data:
                    uart_buffer += raw_data.decode("utf-8")
                    if uart_buffer.endswith("\n"):
                        debug_print(
                            f"[RP2040 DEBUG] Complete data received: {uart_buffer}\n"
                        )
                        try:
                            data = json.loads(uart_buffer.strip())
                            debug_print(f"[RP2040 DEBUG] Parsed JSON: {data}\n")
                            if isinstance(data, dict):
                                function_type = data.get("Function")
                                debug_print(
                                    f"[RP2040 DEBUG] Function type: {function_type}\n"
                                )
                                if function_type == "NeoPixel":
                                    # Execute NeoPixel sequence directly in this thread
                                    neopixel_running = (
                                        False  # Stop any ongoing sequence
                                    )
                                    utime.sleep_ms(10)  # Brief pause for cleanup
                                    handle_neopixel_sequence(
                                        np, data
                                    )  # Direct execution, no new thread
                                    debug_print("[Task] Neopixel Completed\n")
                                else:
                                    debug_print("Invalid function type\n")
                        except Exception as e:
                            debug_print(f"[RP2040 DEBUG] JSON decode error: {str(e)}\n")
                            # Handle non-JSON data as before
                            try:
                                int_data = uart_buffer.strip()
                                debug_print(
                                    f"[RP2040 DEBUG] Non-JSON data: {int_data}\n"
                                )
                            except ValueError:
                                debug_print("Invalid data received\n")
                                print(f"Invalid data received: {uart_buffer}")
                        uart_buffer = ""
        except Exception as e:
            debug_print(f"[RP2040 DEBUG] Error in UART handling: {str(e)}\n")
            print(f"Error in UART handling: {e}")
            uart_buffer = ""

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
                # This only send soft shutdown command to the Linux.
                # The actual power off is handled by the Linux.
                pamir_protocol.send_shutdown_command()

                # When the uart recieve confirmation from Linux that Shutdown is executed from its end
                # The RP2040 will turn off the eink and mux. This will be a forced shutdown.
                # TODO: from the core1 task
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


# Clean main loop
while True:
    wdt.feed()

    check_for_power_on()
    check_for_power_off()

    if BATTERY_MODE:
        try:
            soc = battery.remain_capacity()
            voltage = battery.voltage_V()
            temp = battery.temp_C()
            current = battery.avg_current_mA()
            print(f"SOC: {soc}%  V: {voltage}  T: {temp} A: {current}")
            pamir_protocol.send_debug_text(
                f"SOC: {soc}%  V: {voltage}  T: {temp} A: {current}"
            )
        except Exception as e:
            print(f"Battery read error: {e}")
            pamir_protocol.send_debug_text(f"Battery read error: {e}")
        utime.sleep_ms(1000)

    utime.sleep_ms(1)
