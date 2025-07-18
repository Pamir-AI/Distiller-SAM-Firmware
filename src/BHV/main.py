"""RP2040 SAM Firmware v2.3.0"""

# pylint: disable=import-error,broad-exception-caught,gl`obal-statement,consider-using-f-string,import-outside-toplevel
import machine
import utime
import neopixel
from debug_handler import DebugHandler, init_debug_handler
from uart_handler import UartHandler
from threaded_task_manager import ThreadedTaskManager
from pamir_uart_protocols import PamirUartProtocols
from neopixel_controller import NeoPixelController
from power_manager import PowerManager

# KEEP THIS LINE FIRST LINE OF THE FILE
pmic_enable = machine.Pin(3, machine.Pin.IN, pull=None)

# Configuration
PRODUCTION = False  # Set to True for production builds
LUT_MODE = True  # Set to True for LUT mode, False for fast mode
EINK_ANIMATION_TIMEOUT = 2  # Maximum number of animation cycles before timeout
INITIAL_DEBUG_LEVEL = (
    DebugHandler.LEVEL_VERBOSE if not PRODUCTION else DebugHandler.LEVEL_ERROR
)

# Hardware configuration
UART_BAUDRATE = 115200
UART_TX_PIN = 0
UART_RX_PIN = 1
DEBUG_LED_PIN = 20
LED_BAR_PIN = 6
BUTTON_DEBOUNCE_MS = 50

# Initialize global components
debug = init_debug_handler(
    INITIAL_DEBUG_LEVEL,  # level
    not PRODUCTION,  # enable_uart_output
    100,  # buffer_size
    True,  # enable_statistics
)

# Initialize watchdog
wdt = machine.WDT(timeout=2000)

# GPIO setup
selectBTN = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_DOWN)
upBTN = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_DOWN)
downBTN = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_DOWN)
einkStatus = machine.Pin(9, machine.Pin.OUT)
einkMux = machine.Pin(22, machine.Pin.OUT)
sam_interrupt = machine.Pin(2, machine.Pin.OUT)

# USB switch configuration
usb_switch_s = machine.Pin(23, machine.Pin.OUT, value=0)

# Debug RGB LED
debug_rgb = neopixel.NeoPixel(machine.Pin(DEBUG_LED_PIN), 1)

DEBUG_COLORS = {
    "OFF": (0, 0, 0),
    "INIT": (255, 0, 0),  # Red - Initialization
    "EINK_RUNNING": (255, 255, 255),  # White - EINK_RUNNING
    "UART_READY": (0, 255, 0),  # Green - UART ready
    "MAIN_LOOP": (0, 0, 255),  # Blue - Main loop
    "ERROR": (255, 0, 255),  # Magenta - Error
    "PACKET_RX": (0, 255, 255),  # Cyan - Packet received
    "PACKET_VALID": (0, 128, 0),  # Dark green - Valid packet
    "PACKET_INVALID": (255, 128, 0),  # Orange - Invalid packet
}


def set_debug_color(color_name):
    """Set debug RGB color with error handling"""
    try:
        if color_name in DEBUG_COLORS:
            color = DEBUG_COLORS[color_name]
            debug_rgb[0] = color
            debug_rgb.write()
    except Exception as e:
        debug.log_error(
            debug.CAT_SYSTEM, f"Failed to set debug color {color_name}: {e}"
        )


def switch_usb(usb_type):
    """Switch USB connection"""
    usb_switch_table = {"SAM_USB": [0, 0], "SOM_USB": [1, 0]}
    if usb_type in usb_switch_table:
        s, _ = usb_switch_table[usb_type]
        usb_switch_s.value(s)
        debug.log_info(debug.CAT_SYSTEM, f"USB switched to {usb_type}")
    else:
        debug.log_error(debug.CAT_SYSTEM, f"Invalid USB type: {usb_type}")


# Initialize components
set_debug_color("INIT")
debug.log_info(debug.CAT_SYSTEM, "=== RP2040 SAM Firmware v0.2.3 Starting ===")

# USB switch setup
switch_usb("SOM_USB")

# Initialize protocol handler
protocol = PamirUartProtocols()
debug.protocol = protocol  # Allow debug handler to create debug packets

# Initialize UART
uart0 = machine.UART(
    0, baudrate=UART_BAUDRATE, tx=machine.Pin(UART_TX_PIN), rx=machine.Pin(UART_RX_PIN)
)
debug.log_info(debug.CAT_UART, f"UART initialized: {UART_BAUDRATE} baud")

# Initialize improved UART handler
uart_handler = UartHandler(uart0, protocol, debug)
debug.log_info(debug.CAT_UART, "UART handler initialized")

# Initialize task manager
task_manager = ThreadedTaskManager(debug)
debug.log_info(debug.CAT_SYSTEM, "Threaded task manager initialized")

# Initialize power manager
power_manager = PowerManager(design_capacity_mah=3000, debug_enabled=not PRODUCTION)
debug.log_info(debug.CAT_POWER, "Power manager initialized")


# LED completion callback
def led_completion_callback(led_id, sequence_length):
    """Callback for LED animation completion"""

    def send_led_ack():
        try:
            if sequence_length > 0:
                packet = protocol.create_led_completion_packet(led_id, sequence_length)
                uart0.write(packet)
                debug.log_info(
                    debug.CAT_LED,
                    f"LED ACK: LED{led_id}, {sequence_length} commands -> TX: {packet.hex()}",
                )
            elif sequence_length < 0:
                error_code = abs(sequence_length)
                packet = protocol.create_led_error_packet(led_id, error_code)
                uart0.write(packet)
                debug.log_error(
                    debug.CAT_LED,
                    f"LED error ACK: LED{led_id}, error {error_code} -> TX: {packet.hex()}",
                )
        except Exception as e:
            debug.log_error(debug.CAT_LED, f"LED ACK failed: {e}")

    # Submit ACK as high-priority task
    task_manager.submit_task(
        "LED_ACK", send_led_ack, priority=task_manager.PRIORITY_HIGH
    )


# Initialize NeoPixel controller
np_controller = NeoPixelController(
    pin=LED_BAR_PIN,
    num_leds=7,
    default_brightness=0.2,
    completion_callback=led_completion_callback,
)
debug.log_info(debug.CAT_LED, "NeoPixel controller initialized")

# E-ink display initialization
einkMux.low()  # EINK OFF initially
einkStatus.high()  # SOM CONTROL E-INK

debug.log_info(debug.CAT_SYSTEM, "Hardware initialization complete")

# Global state
button_state_cache = {"up": False, "down": False, "select": False, "power": False}
display_release_received = False  # Flag to track display release signal

# E-ink state variables (needed for proper eink control)
import _thread
eink_lock = _thread.allocate_lock()
einkRunning = False


# Packet processing functions

def process_led_packet(packet_data):
    """Process LED control packet - directly control debug RGB LED with animation support"""
    valid, led_data = protocol.parse_led_packet(packet_data)
    if valid:
        debug.log_verbose(debug.CAT_LED, f"LED command: {led_data}")

        try:
            # Extract color data (use 4-bit values directly, no scaling)
            color = led_data["color"]
            r = color[0]  # 4-bit value (0-15)
            g = color[1]  # 4-bit value (0-15)
            b = color[2]  # 4-bit value (0-15)
            time_value = led_data["time_value"]
            led_mode = led_data["led_mode"]

            # Get LED ID (0-15, map to available hardware LEDs)
            led_id = led_data["led_id"]
            
            # Map LED ID to hardware index (constrain to available LEDs)
            hardware_index = led_id % np_controller.num_leds  # Wrap around if LED ID > num_leds
            
            # Determine animation mode based on led_mode bits
            if led_mode == protocol.LED_MODE_STATIC:
                # Static mode - set color immediately using NeoPixel controller
                rgb_color = np_controller.rgb444_to_rgb888((r, g, b))
                np_controller.set_color(rgb_color, index=hardware_index)
                debug.log_info(
                    debug.CAT_LED,
                    f"LED {led_id} (hw_index={hardware_index}) set to RGB({r}, {g}, {b}) -> RGB LED updated (static)",
                )

            elif led_mode == protocol.LED_MODE_BLINK:
                # Blink mode
                debug.log_info(
                    debug.CAT_LED,
                    f"LED {led_id} (hw_index={hardware_index}) blink mode: RGB({r}, {g}, {b}) time={time_value}",
                )

                # Use NeoPixel controller for animation
                np_controller.stop_animation()
                np_controller.add_to_queue(hardware_index, np_controller.MODE_BLINK, (r, g, b), time_value)
                np_controller.execute_queue()

            elif led_mode == protocol.LED_MODE_FADE:
                # Fade mode
                debug.log_info(
                    debug.CAT_LED,
                    f"LED {led_id} (hw_index={hardware_index}) fade mode: RGB({r}, {g}, {b}) time={time_value}",
                )

                # Use NeoPixel controller for animation
                np_controller.stop_animation()
                np_controller.add_to_queue(hardware_index, np_controller.MODE_FADE, (r, g, b), time_value)
                np_controller.execute_queue()

            elif led_mode == protocol.LED_MODE_RAINBOW:
                # Rainbow mode
                debug.log_info(debug.CAT_LED, f"LED {led_id} (hw_index={hardware_index}) rainbow mode: time={time_value}")

                # Use NeoPixel controller for animation
                np_controller.stop_animation()
                np_controller.add_to_queue(hardware_index, np_controller.MODE_RAINBOW, (0, 0, 0), time_value)
                np_controller.execute_queue()

            else:
                # Unknown mode - default to static
                debug.log_info(
                    debug.CAT_LED, f"Unknown LED mode: 0x{led_mode:02X}, using static"
                )
                rgb_color = np_controller.rgb444_to_rgb888((r, g, b))
                np_controller.set_color(rgb_color, index=hardware_index)

            # Send acknowledgment if needed
            if led_data.get("execute", False):
                try:
                    led_id = led_data["led_id"]
                    ack_packet = protocol.create_led_completion_packet(led_id, 1)
                    uart0.write(ack_packet)
                    debug.log_info(debug.CAT_LED, f"LED ACK sent: {ack_packet.hex()}")
                except Exception as e:
                    debug.log_error(debug.CAT_LED, f"LED ACK failed: {e}")

        except Exception as e:
            debug.log_error(debug.CAT_LED, f"LED command failed: {e}")
    else:
        # Try parsing as acknowledgment
        valid_ack, ack_data = protocol.parse_led_acknowledgment(packet_data)
        if valid_ack and ack_data["type"] == "status":
            debug.log_info(debug.CAT_LED, f"LED status request: {ack_data}")


def process_power_packet(packet_data):
    """Process power management packet"""
    valid, power_data = protocol.parse_power_packet(packet_data)
    if valid:
        debug.log_verbose(debug.CAT_POWER, f"Power command: {power_data}")

        def execute_power_command():
            try:
                cmd_type = power_data.get("command", "unknown")

                if cmd_type == "query":
                    current_state = power_manager.get_power_state()
                    debug.log_info(
                        debug.CAT_POWER,
                        f"Power query received: current_state: 0x{current_state:02X}",
                    )

                elif cmd_type == "set_state":
                    new_state = power_data.get(
                        "power_state", power_manager.current_power_state
                    )
                    power_manager.set_power_state(new_state)
                    packet = protocol.create_power_status_packet_rp2040_to_som(
                        new_state, 0x00
                    )
                    uart0.write(packet)
                    debug.log_info(
                        debug.CAT_POWER, f"Power state set: 0x{new_state:02X}"
                    )

                elif cmd_type == "sleep":
                    delay_seconds = power_data.get("delay_seconds", 0)
                    sleep_flags = power_data.get("sleep_flags", 0)
                    power_manager.handle_sleep_command(delay_seconds, sleep_flags)

                elif cmd_type == "shutdown":
                    shutdown_mode = power_data.get("shutdown_mode", 0)
                    reason_code = power_data.get("reason_code", 0)
                    power_manager.handle_shutdown_command(shutdown_mode, reason_code)
                    packet = protocol.create_power_status_packet_rp2040_to_som(
                        protocol.POWER_STATE_OFF, shutdown_mode
                    )
                    uart0.write(packet)
                    debug.log_info(
                        debug.CAT_POWER, f"Shutdown ACK: mode={shutdown_mode}"
                    )

                elif cmd_type == "request_metrics":
                    metrics = power_manager.get_all_metrics()

                    # Send all metrics
                    for metric_type, value in [
                        (protocol.POWER_CMD_CURRENT, metrics["current_ma"]),
                        (protocol.POWER_CMD_BATTERY, metrics["battery_percent"]),
                        (protocol.POWER_CMD_TEMP, metrics["temperature_0_1c"]),
                        (protocol.POWER_CMD_VOLTAGE, metrics["voltage_mv"]),
                    ]:
                        packet = protocol.create_power_metrics_packet_rp2040_to_som(
                            metric_type, value
                        )
                        uart0.write(packet)

                    debug.log_info(debug.CAT_POWER, f"Metrics sent: {metrics}")

            except Exception as e:
                debug.log_error(debug.CAT_POWER, f"Power command failed: {e}")

        # Execute power command immediately to avoid task manager threading issues
        try:
            execute_power_command()
        except Exception as e:
            debug.log_error(debug.CAT_POWER, f"Immediate power command execution failed: {e}")
            
        # Also submit to task manager for statistics
        task_manager.submit_task(
            "POWER_COMMAND", execute_power_command, priority=task_manager.PRIORITY_HIGH
        )


def process_system_packet(packet_data):
    """Process system command packet"""
    valid, system_data = protocol.parse_system_packet(packet_data)
    if valid:
        debug.log_verbose(debug.CAT_SYSTEM, f"System command: {system_data}")

        if system_data["command"] == "ping":

            def send_ping_response():
                try:
                    ping_response = protocol.create_system_ping_packet()
                    uart0.write(ping_response)
                    debug.log_info(
                        debug.CAT_SYSTEM,
                        f"Ping response sent -> TX: {ping_response.hex()}",
                    )
                except Exception as e:
                    debug.log_error(debug.CAT_SYSTEM, f"Ping response failed: {e}")

            task_manager.submit_task(
                "PING_RESPONSE", send_ping_response, priority=task_manager.PRIORITY_HIGH
            )


def process_display_packet(packet_data):
    """Process display control packet"""
    valid, display_data = protocol.parse_display_packet(packet_data)
    if valid:
        debug.log_verbose(debug.CAT_DISPLAY, f"Display command: {display_data}")

        if display_data["command"] == "release":
            global display_release_received
            display_release_received = True
            debug.log_info(debug.CAT_DISPLAY, "Display release signal received - Pi has booted")
            
            # Send acknowledgment
            def send_display_ack():
                try:
                    ack_packet = protocol.create_display_completion_packet()
                    uart0.write(ack_packet)
                    debug.log_info(
                        debug.CAT_DISPLAY,
                        f"Display release ACK sent -> TX: {ack_packet.hex()}",
                    )
                except Exception as e:
                    debug.log_error(debug.CAT_DISPLAY, f"Display release ACK failed: {e}")

            task_manager.submit_task(
                "DISPLAY_RELEASE_ACK", send_display_ack, priority=task_manager.PRIORITY_HIGH
            )


def process_uart_packet(packet_data):
    """Process received UART packet"""
    packet_type = protocol.get_packet_type(packet_data)

    # Route to appropriate handler
    if packet_type == protocol.TYPE_LED:
        debug.log_info(debug.CAT_LED, f"Processing LED packet: {packet_data.hex()}")
        process_led_packet(packet_data)
    elif packet_type == protocol.TYPE_POWER:
        debug.log_info(debug.CAT_POWER, f"Processing POWER packet: {packet_data.hex()}")
        process_power_packet(packet_data)
    elif packet_type == protocol.TYPE_DISPLAY:
        debug.log_info(
            debug.CAT_DISPLAY, f"Processing DISPLAY packet: {packet_data.hex()}"
        )
        process_display_packet(packet_data)
    elif packet_type == protocol.TYPE_SYSTEM:
        debug.log_info(
            debug.CAT_SYSTEM, f"Processing SYSTEM packet: {packet_data.hex()}"
        )
        process_system_packet(packet_data)
    else:
        debug.log_info(
            debug.CAT_UART,
            f"Unhandled packet type: 0x{packet_type:02X} data={packet_data.hex()}",
        )


# Button handling
def debounce_button(pin):
    """Non-blocking button debounce - returns current state immediately"""
    return pin.value()


def get_button_states():
    """Get current button states with immediate response"""
    return {
        "up": upBTN.value(),
        "down": downBTN.value(),
        "select": selectBTN.value(),
        "power": False,  # Power button handled separately
    }


def button_interrupt_handler(pin):
    """button interrupt handler"""
    try:
        # Brief delay for hardware settling
        utime.sleep_ms(1)

        # Process button state immediately
        states = get_button_states()

        # Only send if state changed
        global button_state_cache
        if states != button_state_cache:
            packet = protocol.create_button_packet(
                up_pressed=states["up"],
                down_pressed=states["down"],
                select_pressed=states["select"],
                power_pressed=states["power"],
            )

            uart0.write(packet)
            debug.log_info(
                debug.CAT_BUTTON, f"Button state: {states} -> TX: {packet.hex()}"
            )
            button_state_cache = states.copy()

    except Exception as e:
        debug.log_error(debug.CAT_BUTTON, f"Button interrupt handler error: {e}")


# Set up button interrupts
selectBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
    handler=button_interrupt_handler,
)
upBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
    handler=button_interrupt_handler,
)
downBTN.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
    handler=button_interrupt_handler,
)


# Boot notification
def send_boot_notification():
    """Send boot notification to SoM"""
    try:
        packet = protocol.create_power_status_packet_rp2040_to_som(
            protocol.POWER_STATE_RUNNING, 0x00
        )
        uart0.write(packet)
        debug.log_info(debug.CAT_POWER, "Boot notification sent to SoM")
    except Exception as e:
        debug.log_error(debug.CAT_POWER, f"Boot notification failed: {e}")


# UART communication task (runs on Core 1)
def uart_communication_task():
    """Main UART communication task - runs on Core 1 with highest priority"""
    global uart_handler  # Make uart_handler accessible in this thread

    debug.log_info(debug.CAT_UART, "UART communication task started on Core 1")
    set_debug_color("UART_READY")

    # Send boot notification
    send_boot_notification()

    # Statistics tracking
    last_stats_time = utime.ticks_ms()
    stats_interval = 10000  # 10 seconds

    while True:
        try:
            wdt.feed()

            # Receive data from UART
            bytes_received = uart_handler.receive_data()
            if bytes_received > 0:
                set_debug_color("PACKET_RX")
                debug.log_verbose(debug.CAT_UART, f"Received {bytes_received} bytes")

            # Process any complete packets
            processed_packets = uart_handler.process_packets()

            for valid, packet_data in processed_packets:
                if valid:
                    set_debug_color("PACKET_VALID")
                    packet_type = protocol.get_packet_type(packet_data)
                    debug.log_info(
                        debug.CAT_UART,
                        f"Valid packet: type=0x{packet_type:02X} data={packet_data.hex()}",
                    )
                    process_uart_packet(packet_data)
                else:
                    set_debug_color("PACKET_INVALID")
                    debug.log_error(debug.CAT_UART, "Invalid packet received")

            # Check UART handler health periodically
            current_time = utime.ticks_ms()
            if utime.ticks_diff(current_time, last_stats_time) >= stats_interval:
                uart_stats = uart_handler.get_statistics()
                uart_msg = "UART: {}% success, {}% buffer, state: {}".format(
                    uart_stats["success_rate_percent"],
                    uart_stats["buffer_usage_percent"],
                    uart_stats["sync_state"],
                )
                debug.log_info(debug.CAT_PERFORMANCE, uart_msg)

                task_stats = task_manager.get_statistics()
                task_msg = "Tasks: {} done, Core0: {}%, Core1: {}%".format(
                    task_stats["tasks_completed"],
                    task_stats["core0_utilization_percent"],
                    task_stats["core1_utilization_percent"],
                )
                debug.log_info(debug.CAT_PERFORMANCE, task_msg)

                last_stats_time = current_time

            # Brief sleep to prevent overwhelming CPU
            utime.sleep_ms(0)

        except Exception as e:
            debug.log_error(debug.CAT_UART, f"UART task error: {e}")
            set_debug_color("ERROR")
            utime.sleep_ms(10)


# Submit UART task to run on Core 1 with critical priority
task_manager.submit_uart_task("UART_COMMUNICATION", uart_communication_task)


# E-ink display task (high priority, blocking)
def eink_display_task():
    """E-ink display animation task - runs with high priority until Pi boots"""
    global display_release_received, einkRunning
    
    try:
        debug.log_info(debug.CAT_DISPLAY, "Starting E-ink display task - waiting for Pi boot")

        # Import and initialize E-ink 
        from eink_driver_sam import einkDSP_SAM
        
        # Set debug color to EINK_RUNNING
        set_debug_color("EINK_RUNNING")

        einkStatus.high()  # Provide power to e-ink
        einkMux.high()  # SAM control e-ink

        eink = einkDSP_SAM()

        # Use proper V0.2.1 eink initialization logic
        einkRunning = True
        if eink.init == False:
            eink.re_init()
        
        if LUT_MODE:
            eink.epd_init_lut()
        else:
            eink.epd_init_fast()
            
        try:
            eink.PIC_display(None, './loading1.bin')
            debug.log_info(debug.CAT_DISPLAY, "Initial loading screen displayed")
        except OSError:
            debug.log_error(debug.CAT_DISPLAY, "Loading files not found")
            einkRunning = False
        
        if LUT_MODE:
            utime.sleep_ms(1300)  # give time for first refresh, no lower than 1300
            debug.log_info(debug.CAT_DISPLAY, "LUT mode - waiting for refresh completion")
      
        # Animation loop using V0.2.1 logic, but with display_release_received check
        repeat = 0
        while einkRunning and not display_release_received:
            with eink_lock:
                if not einkRunning or repeat >= EINK_ANIMATION_TIMEOUT:
                    break
            
            # Check for display release signal before each animation step
            if display_release_received:
                debug.log_info(debug.CAT_DISPLAY, "Display release signal received during animation")
                break
                
            try:
                eink.epd_init_part()
                eink.PIC_display('./loading1.bin', './loading2.bin')
                
                # Check during animation
                if display_release_received:
                    break
                    
                eink.epd_init_part()
                eink.PIC_display('./loading2.bin', './loading1.bin')
                
                wdt.feed()
                repeat += 1
                
                debug.log_verbose(debug.CAT_DISPLAY, f"Animation cycle {repeat} completed")
                
                # Yield and check for release signal
                for i in range(5):  # Brief yield with checks
                    if display_release_received:
                        break
                    utime.sleep_ms(10)
                    
            except Exception as e:
                debug.log_error(debug.CAT_DISPLAY, f"Animation cycle error: {e}")
                break
        
        # Clean shutdown
        eink.de_init()
        with eink_lock:
            einkRunning = False
        einkMux.low()
        
        if display_release_received:
            debug.log_info(debug.CAT_DISPLAY, "Pi boot detected - releasing eink control")
        elif repeat >= EINK_ANIMATION_TIMEOUT:
            debug.log_info(debug.CAT_DISPLAY, f"Animation timeout reached ({EINK_ANIMATION_TIMEOUT} cycles) - releasing eink control")
        else:
            debug.log_info(debug.CAT_DISPLAY, "Animation completed - releasing eink control")
            
        debug.log_info(debug.CAT_DISPLAY, "E-ink display task completed - control handed to Pi")

    except Exception as e:
        set_debug_color("ERROR")
        debug.log_error(debug.CAT_DISPLAY, f"E-ink task failed: {e}")
        # Ensure eink is released even on error
        try:
            eink.de_init()
            with eink_lock:
                einkRunning = False
            einkMux.low()
        except:
            pass


# Run E-ink task directly on core 0 (blocking)
# eink_display_task()

# Main loop (runs on Core 0)
debug.log_info(debug.CAT_SYSTEM, "=== Main loop starting ===")
set_debug_color("MAIN_LOOP")

# Main application loop
last_heartbeat = utime.ticks_ms()
HEARTBEAT_INTERVAL = 10000  # 10 seconds - reduced for better responsiveness

while True:
    try:
        wdt.feed()

        # Periodic health checks
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, last_heartbeat) >= HEARTBEAT_INTERVAL:

            # Check UART handler health
            uart_health = uart_handler.check_health()
            if uart_health["status"] != "HEALTHY":
                health_msg = "UART health: {} - {}".format(
                    uart_health["status"], uart_health["issues"]
                )
                debug.log_error(debug.CAT_SYSTEM, health_msg)

            # Send heartbeat debug code to kernel
            debug.send_debug_code(uart0, 0, 0x01, 0)  # System category, heartbeat code

            debug.log_info(debug.CAT_SYSTEM, "System heartbeat")
            last_heartbeat = current_time

        # Sleep to allow other tasks to run
        utime.sleep_ms(100)

    except Exception as e:
        debug.log_error(debug.CAT_SYSTEM, f"Main loop error: {e}")
        set_debug_color("ERROR")
        utime.sleep_ms(1000)  # Longer delay on error
