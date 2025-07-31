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
pmic_enable = machine.Pin(3, machine.Pin.OPEN_DRAIN, value=0)  # CM5 OFF by default
pmic_enable.value(0)

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

# Timing constants
SELECT_LONG_PRESS_MS = 3000  # 3 seconds for power on
FORCE_SHUTDOWN_PRESS_MS = 5000  # 5 seconds for force shutdown
NORMAL_SHUTDOWN_DELAY_MS = 2000  # 2 seconds delay for normal shutdown
BATTERY_CHECK_INTERVAL = 10000  # 10 seconds between battery display checks
EMERGENCY_SHUTDOWN_INTERVAL = 5000  # 5 seconds between emergency shutdown attempts

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
cm5_power_btn = machine.Pin(4, machine.Pin.OPEN_DRAIN, value=1)

def set_debug_color(color_name):
    """Set debug RGB color with error handling"""
    scale_down_factor = 2
    try:
        if color_name in DEBUG_COLORS:
            color = DEBUG_COLORS[color_name]
            # Scale down color values by 1/3 
            debug_rgb[0] = (color[0] // scale_down_factor, color[1] // scale_down_factor, color[2] // scale_down_factor)
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


DEBUG_COLORS = {
    "OFF": (0, 0, 0),
    "INIT": (255, 0, 0),  # Red - Initialization
    "EINK_RUNNING": (255, 255, 255),  # White - EINK_RUNNING
    "UART_READY": (0, 255, 0),  # Green - UART ready
    "MAIN_LOOP": (0, 0, 0),  # Blue - Main loop
    "ERROR": (255, 0, 255),  # Magenta - Error
    "PACKET_RX": (0, 255, 255),  # Cyan - Packet received
    "PACKET_VALID": (0, 128, 0),  # Dark green - Valid packet
    "PACKET_INVALID": (255, 128, 0),  # Orange - Invalid packet
    "DEBUG_MODE": (255, 255, 0),  # Yellow - Debug mode
    "CHARGING": (255, 0, 0),  # Red - Charging
}

# CM5 Firmware Upload Mode
if upBTN.value() and not selectBTN.value() and not downBTN.value():
    wdt.feed()
    # Switch USB to SOM_USB
    pmic_enable.value(1)
    switch_usb("SOM_USB")
    set_debug_color("DEBUG_MODE")
    while True:
        wdt.feed()
        utime.sleep_ms(1000) 
        


def set_cm5_power(enable):
    """Control CM5 power via pmic_enable pin (OPEN_DRAIN mode)
    
    Args:
        enable (bool): True to power on (high-z), False to power off (pull low)
    """
    global power_state
    
    try:
        if enable:
            # Power on: Set pin to high-z (floating) - CM5 can power on
            pmic_enable.value(1)
            power_state["cm5_powered"] = True
            debug.log_info(debug.CAT_POWER, "CM5 powered ON (pmic_enable -> high-z)")
            set_debug_color("UART_READY")
        else:
            # Power off: Pull pin low - CM5 powers off
            pmic_enable.value(0)
            power_state["cm5_powered"] = False
            debug.log_info(debug.CAT_POWER, "CM5 powered OFF (pmic_enable -> low)")
            set_debug_color("INIT")
            
            # Reset system state when powering off
            reset_system_state()
            
        # Update power manager state
        if enable:
            power_manager.set_power_state(protocol.POWER_STATE_RUNNING)
        else:
            power_manager.set_power_state(protocol.POWER_STATE_OFF)
            
    except Exception as e:
        debug.log_error(debug.CAT_POWER, f"Failed to set CM5 power: {e}")

# Initialize components
set_debug_color("INIT")
debug.log_info(debug.CAT_SYSTEM, "=== RP2040 SAM Firmware v0.2.4 Starting ===")


# SAM UART MODE
if downBTN.value() and not selectBTN.value() and not upBTN.value():
    wdt.feed()
    # Switch USB to SAM_USB
    switch_usb("SAM_USB")
    set_debug_color("DEBUG_MODE")
else:    
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


# Battery display management
def get_battery_display_requirement():
    """Check battery level and return required display action
    
    Returns:
        dict: {
            "action": "none" | "low_battery" | "dead_battery",
            "image_path": path to image file or None,
            "battery_percent": current battery percentage or None,
            "current_ma": current draw in mA,
            "is_charging": whether battery is charging
        }
    """
    try:
        # Don't display anything if no battery is connected
        if power_manager.bq27441 is None:
            debug.log_verbose(debug.CAT_POWER, "No battery connected - no display needed")
            return {
                "action": "none", 
                "image_path": None, 
                "battery_percent": None,
                "current_ma": None,
                "is_charging": False
            }
        
        # Get current battery metrics
        battery_percent = power_manager.get_battery_percent()
        current_ma = power_manager.get_current_ma()
        is_charging = current_ma > 0  # Positive current means charging
        
        # Determine required action based on battery level
        if battery_percent <= 2:
            # Dead battery - only show if not charging
            if is_charging:
                debug.log_verbose(
                    debug.CAT_POWER, 
                    f"Battery dead ({battery_percent}%) but charging ({current_ma}mA) - no warning needed"
                )
                return {
                    "action": "none", 
                    "image_path": None, 
                    "battery_percent": battery_percent,
                    "current_ma": current_ma,
                    "is_charging": is_charging
                }
            else:
                return {
                    "action": "dead_battery",
                    "image_path": "./battery-dead.bin",
                    "battery_percent": battery_percent,
                    "current_ma": current_ma,
                    "is_charging": is_charging
                }
        elif battery_percent <= 5:
            # Low battery - only show if not charging
            if is_charging:
                debug.log_verbose(
                    debug.CAT_POWER, 
                    f"Battery low ({battery_percent}%) but charging ({current_ma}mA) - no warning needed"
                )
                return {
                    "action": "none", 
                    "image_path": None, 
                    "battery_percent": battery_percent,
                    "current_ma": current_ma,
                    "is_charging": is_charging
                }
            else:
                return {
                    "action": "low_battery", 
                    "image_path": "./battery-low.bin",
                    "battery_percent": battery_percent,
                    "current_ma": current_ma,
                    "is_charging": is_charging
                }
        else:
            return {
                "action": "none", 
                "image_path": None, 
                "battery_percent": battery_percent,
                "current_ma": current_ma,
                "is_charging": is_charging
            }
            
    except Exception as e:
        debug.log_error(debug.CAT_POWER, f"Battery display check failed: {e}")
        return {
            "action": "none", 
            "image_path": None, 
            "battery_percent": None,
            "current_ma": None,
            "is_charging": False
        }


def check_and_display_battery_status():
    """Check battery status and display appropriate image if needed (only when CM5 is off)"""
    global power_state, last_battery_display_check, current_battery_display_action, current_eink_display_state
    
    # Only check when CM5 is powered off
    if power_state["cm5_powered"]:
        return
    
    # Throttle battery checks to avoid excessive power manager calls
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_battery_display_check) < BATTERY_CHECK_INTERVAL:
        return
    
    last_battery_display_check = current_time
    
    try:
        battery_requirement = get_battery_display_requirement()
        
        # Always print battery info when this function is called
        if battery_requirement["battery_percent"] is not None:
            voltage_mv = power_manager.get_voltage_mv()
            temperature_0_1c = power_manager.get_temperature_0_1c()
            charging_status = "charging" if battery_requirement["is_charging"] else "discharging"
            
            debug.log_info(debug.CAT_POWER, "[Battery Info] {}% | {}mV | {}mA ({}) | {:.1f}°C".format(
                battery_requirement['battery_percent'], voltage_mv, battery_requirement['current_ma'], 
                charging_status, temperature_0_1c/10))
        else:
            debug.log_info(debug.CAT_POWER, "[Battery Info] No battery connected")
        
        # Determine what should be displayed based on battery requirement
        required_eink_state = "none"
        if battery_requirement["action"] == "dead_battery":
            required_eink_state = "battery_dead"
        elif battery_requirement["action"] == "low_battery":
            required_eink_state = "battery_low"
        
        # Only display if required state is different from current eink state
        if required_eink_state != "none" and required_eink_state != current_eink_display_state:
            debug.log_info(
                debug.CAT_POWER, 
                "Battery {}% - displaying {} image (current: {})".format(
                    battery_requirement['battery_percent'], 
                    battery_requirement['action'],
                    current_eink_display_state
                )
            )
            
            # Call eink display task directly (blocking)
            try:
                eink_display_task([battery_requirement["image_path"]])
                
                # Update eink display state after successful display
                current_eink_display_state = required_eink_state
                current_battery_display_action = battery_requirement["action"]
                
                debug.log_info(debug.CAT_POWER, "Eink display state updated to: {}".format(current_eink_display_state))
                
            except Exception as e:
                debug.log_error(debug.CAT_POWER, f"Battery image display failed: {e}")
        else:
            # Log why we're not displaying
            if required_eink_state == "none":
                debug.log_verbose(debug.CAT_POWER, "No battery warning needed")
            else:
                debug.log_verbose(debug.CAT_POWER, "Battery warning already displayed (current: {})".format(current_eink_display_state))
        
    except Exception as e:
        debug.log_error(debug.CAT_POWER, f"Battery status check failed: {e}")


# Power control functions (must be defined before first use)
def reset_system_state():
    """Reset system state when CM5 is powered off"""
    global display_release_received, einkRunning, power_state, current_battery_display_action, current_eink_display_state
    
    # Reset display state
    display_release_received = False
    
    # Stop any running eink operations
    try:
        with eink_lock:
            einkRunning = False
    except:
        pass
    
    # Reset power-related timers
    power_state["shutdown_pending"] = False
    power_state["shutdown_timer"] = 0
    
    # Reset battery display state to allow new battery displays when CM5 is off
    current_battery_display_action = "none"
    
    # Reset eink display state since CM5 can control eink and we don't know what it displayed
    current_eink_display_state = "unknown"
    
    debug.log_info(debug.CAT_SYSTEM, "System state reset for CM5 power off - eink state now unknown")


def check_power_on_trigger():
    """Check for SELECT button long press to power on CM5"""
    global power_state
    
    current_time = utime.ticks_ms()
    
    # If CM5 is already powered, nothing to do
    if power_state["cm5_powered"]:
        return
        
    # Check SELECT button state
    if selectBTN.value():
        # SELECT button is pressed
        if power_state["select_press_start"] == 0:
            # First press detected
            power_state["select_press_start"] = current_time
            debug.log_info(debug.CAT_POWER, "SELECT button press detected for power-on")
        else:
            # Check if long press duration reached
            press_duration = utime.ticks_diff(current_time, power_state["select_press_start"])
            if press_duration >= SELECT_LONG_PRESS_MS:
                # Check battery level before allowing power on
                battery_requirement = get_battery_display_requirement()
                if (battery_requirement["battery_percent"] is not None and 
                    battery_requirement["battery_percent"] <= 2 and 
                    not battery_requirement["is_charging"]):
                    # Battery too low and not charging - prevent power on and show dead battery image
                    debug.log_info(
                        debug.CAT_POWER, 
                        "Power on blocked - battery too low ({}%) and not charging".format(
                            battery_requirement['battery_percent']
                        )
                    )
                    power_state["select_press_start"] = 0  # Reset
                    
                    # Display dead battery image to indicate why power on failed (blocking call)
                    try:
                        eink_display_task(["./battery-dead.bin"])
                        # Update eink display state since we displayed dead battery
                        global current_eink_display_state
                        current_eink_display_state = "battery_dead"
                        debug.log_info(debug.CAT_POWER, "Eink display state updated to: battery_dead (power-on blocked)")
                    except Exception as e:
                        debug.log_error(debug.CAT_POWER, f"Low battery warning display failed: {e}")
                elif (battery_requirement["battery_percent"] is not None and 
                      battery_requirement["battery_percent"] <= 2 and 
                      battery_requirement["is_charging"]):
                    # Battery low but charging - allow power on
                    debug.log_info(
                        debug.CAT_POWER, 
                        "Battery low ({}%) but charging ({}mA) - allowing power on".format(
                            battery_requirement['battery_percent'],
                            battery_requirement['current_ma']
                        )
                    )
                    debug.log_info(debug.CAT_POWER, f"SELECT long press ({press_duration}ms) - powering on CM5")
                    set_cm5_power(True)
                    power_state["select_press_start"] = 0  # Reset
                    
                    # Start eink task after power on
                    eink_boot_screen_task()
                else:
                    # Battery OK or no battery sensor - allow power on
                    debug.log_info(debug.CAT_POWER, f"SELECT long press ({press_duration}ms) - powering on CM5")
                    set_cm5_power(True)
                    power_state["select_press_start"] = 0  # Reset
                    
                    # Start eink task after power on
                    eink_boot_screen_task()
    else:
        # SELECT button released - reset timer
        if power_state["select_press_start"] != 0:
            press_duration = utime.ticks_diff(current_time, power_state["select_press_start"])
            debug.log_verbose(debug.CAT_POWER, f"SELECT button released after {press_duration}ms")
            power_state["select_press_start"] = 0


def check_force_shutdown_trigger():
    """Check for UP+DOWN button long press to force shutdown"""
    global power_state
    
    current_time = utime.ticks_ms()
    
    # Check if both UP and DOWN buttons are pressed
    both_pressed = upBTN.value() and selectBTN.value()
    
    if both_pressed:
        # Both buttons pressed
        if power_state["force_press_start"] == 0:
            # First detection of both buttons pressed
            power_state["force_press_start"] = current_time
            debug.log_info(debug.CAT_POWER, "UP+DOWN buttons pressed - force shutdown trigger detected")
        else:
            # Check if long press duration reached
            press_duration = utime.ticks_diff(current_time, power_state["force_press_start"])
            if press_duration >= FORCE_SHUTDOWN_PRESS_MS:
                # Initiate force shutdown
                set_cm5_power(False) 
                
                set_debug_color("ERROR")
                utime.sleep_ms(1000) 
                
                machine.reset()
    else:
        # Buttons released - reset timer
        if power_state["force_press_start"] != 0:
            press_duration = utime.ticks_diff(current_time, power_state["force_press_start"])
            debug.log_verbose(debug.CAT_POWER, f"UP+DOWN buttons released after {press_duration}ms")
            power_state["force_press_start"] = 0


def handle_power_timers():
    """Handle shutdown delay timers"""
    global power_state
    
    current_time = utime.ticks_ms()
    
    # Handle normal shutdown timer
    if power_state["shutdown_pending"]:
        elapsed = utime.ticks_diff(current_time, power_state["shutdown_timer"])
        if elapsed >= NORMAL_SHUTDOWN_DELAY_MS:
            debug.log_info(debug.CAT_POWER, f"Normal shutdown delay ({elapsed}ms) complete - powering off CM5")
            set_cm5_power(False)
            power_state["shutdown_pending"] = False
            power_state["shutdown_timer"] = 0
    
            


def check_emergency_battery_shutdown():
    """Check for emergency battery shutdown condition and signal CM5 if needed"""
    global last_emergency_shutdown_check, power_state
    
    # Only check when CM5 is powered on
    if not power_state["cm5_powered"]:
        return
    
    # Throttle emergency shutdown checks to prevent excessive triggers
    current_time = utime.ticks_ms()
    if utime.ticks_diff(current_time, last_emergency_shutdown_check) < EMERGENCY_SHUTDOWN_INTERVAL:
        return
    
    try:
        # Check if no battery is connected
        if power_manager.bq27441 is None:
            debug.log_verbose(debug.CAT_POWER, "No battery connected - no emergency shutdown needed")
            return
        
        # Get current battery metrics
        battery_percent = power_manager.get_battery_percent()
        current_ma = power_manager.get_current_ma()
        is_charging = current_ma > 0  # Positive current means charging
        
        # Check emergency shutdown conditions:
        # 1. CM5 is powered on (already checked above)
        # 2. Battery is not charging
        # 3. Battery percentage is exactly 1
        if not is_charging and battery_percent == 1:
            debug.log_info(
                debug.CAT_POWER, 
                f"EMERGENCY: Battery at {battery_percent}% and not charging ({current_ma}mA) - signaling CM5 shutdown"
            )
            
            # Signal CM5 by pulsing cm5_power_btn (blocking operation)
            # Set to 0 (active/low) for 100ms, then back to 1 (high-z)
            cm5_power_btn.value(0)  # Pull low (active)
            debug.log_info(debug.CAT_POWER, "Emergency shutdown signal sent (cm5_power_btn -> low)")
            
            # Blocking delay for 100ms
            utime.sleep_ms(100)
            
            cm5_power_btn.value(1)  # Back to high-z (inactive)
            debug.log_info(debug.CAT_POWER, "Emergency shutdown signal released (cm5_power_btn -> high-z)")
            
            # Clear any pending shutdown states before emergency power off
            power_state["shutdown_pending"] = False
            power_state["shutdown_timer"] = 0
            
            # Wait for 2 seconds to change the power state
            utime.sleep_ms(2000)
            set_cm5_power(False)
            debug.log_info(debug.CAT_POWER, "Emergency shutdown complete - CM5 powered off")
            
            # Update timestamp to prevent repeated triggers
            last_emergency_shutdown_check = current_time
            
        else:
            # Log why emergency shutdown was not triggered (only when conditions are close)
            if battery_percent <= 5:  # Only log when battery is getting low
                debug.log_verbose(
                    debug.CAT_POWER,
                    f"Emergency shutdown check: battery {battery_percent}%, charging={is_charging} ({current_ma}mA) - no action needed"
                )
        
    except Exception as e:
        debug.log_error(debug.CAT_POWER, f"Emergency battery shutdown check failed: {e}")


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

# Power management state
power_state = {
    "cm5_powered": False,  # Current CM5 power state
    "shutdown_pending": False,  # Shutdown notification received
    "shutdown_timer": 0,  # Countdown for shutdown delay
    "force_shutdown_pending": False,  # Force shutdown initiated
    "force_shutdown_timer": 0,  # Countdown for force shutdown
    "select_press_start": 0,  # Time when SELECT button was first pressed
    "force_press_start": 0,  # Time when UP+DOWN was first pressed
}

# Battery display state
last_battery_display_check = 0  # Last time battery was checked for display
current_battery_display_action = "none"  # Current battery display action to prevent repeats
current_eink_display_state = "unknown"  # What's currently displayed on eink: "unknown", "battery_low", "battery_dead", "other"

# Emergency battery shutdown state
last_emergency_shutdown_check = 0  # Last time emergency shutdown was triggered

# Initial power control - turn off CM5 at startup (after all components initialized)
debug.log_info(debug.CAT_POWER, "Setting initial CM5 power state to OFF")
set_cm5_power(False)



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
                    
                    # Handle CM5 shutdown notification with delay
                    global power_state
                    if power_state["cm5_powered"] and not power_state["shutdown_pending"]:
                        power_state["shutdown_pending"] = True
                        power_state["shutdown_timer"] = utime.ticks_ms()
                        debug.log_info(
                            debug.CAT_POWER, 
                            f"Shutdown notification received: mode={shutdown_mode}, reason={reason_code}, will power off CM5 in {NORMAL_SHUTDOWN_DELAY_MS}ms"
                        )
                    
                    # Send acknowledgment
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
            global display_release_received, power_state
            display_release_received = True
            
            # Only process if CM5 is actually powered (should be the case)
            if power_state["cm5_powered"]:
                debug.log_info(debug.CAT_DISPLAY, "Display release signal received - CM5 has booted")
            else:
                debug.log_info(debug.CAT_DISPLAY, "Display release received but CM5 not powered - ignoring")
                return
            
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
    """Send boot notification to SoM (only if CM5 is powered)"""
    global power_state
    try:
        if power_state["cm5_powered"]:
            packet = protocol.create_power_status_packet_rp2040_to_som(
                protocol.POWER_STATE_RUNNING, 0x00
            )
            uart0.write(packet)
            debug.log_info(debug.CAT_POWER, "Boot notification sent to CM5")
        else:
            debug.log_verbose(debug.CAT_POWER, "Boot notification skipped - CM5 not powered")
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
def eink_boot_screen_task():
    """E-ink boot screen animation task - runs with high priority until Pi boots"""
    global display_release_received, einkRunning, power_state
    
    try:
        # Check if CM5 is powered before starting
        if not power_state["cm5_powered"]:
            debug.log_info(debug.CAT_DISPLAY, "E-ink task aborted - CM5 not powered")
            return
            
        debug.log_info(debug.CAT_DISPLAY, "Starting E-ink display task - waiting for CM5 boot")

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
        
        # Feed watchdog before displaying images
        wdt.feed()
        
        try:
            eink.PIC_display(None, './loading1.bin')
            debug.log_info(debug.CAT_DISPLAY, "Initial loading screen displayed")
        except OSError:
            debug.log_error(debug.CAT_DISPLAY, "Loading files not found")
            einkRunning = False
        
        wdt.feed()
               
        if LUT_MODE:
            utime.sleep_ms(1300)  # give time for first refresh, no lower than 1300
            debug.log_info(debug.CAT_DISPLAY, "LUT mode - waiting for refresh completion")
      
        wdt.feed()
        # Animation loop with additional power state checks
        repeat = 0
        while einkRunning and not display_release_received and power_state["cm5_powered"]:
            with eink_lock:
                if not einkRunning or repeat >= EINK_ANIMATION_TIMEOUT:
                    break
            
            # Check for display release signal or power off before each animation step
            if display_release_received:
                debug.log_info(debug.CAT_DISPLAY, "Display release signal received during animation")
                break
                
            if not power_state["cm5_powered"]:
                debug.log_info(debug.CAT_DISPLAY, "CM5 powered off during animation - stopping")
                break
                
            try:
                eink.epd_init_part()
                eink.PIC_display('./loading1.bin', './loading2.bin')
                
                # Check during animation
                if display_release_received or not power_state["cm5_powered"]:
                    break
                    
                eink.epd_init_part()
                eink.PIC_display('./loading2.bin', './loading1.bin')
                
                wdt.feed()
                repeat += 1
                
                debug.log_verbose(debug.CAT_DISPLAY, f"Animation cycle {repeat} completed")
                
                # Yield and check for release signal or power state
                for i in range(5):  # Brief yield with checks
                    if display_release_received or not power_state["cm5_powered"]:
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
            debug.log_info(debug.CAT_DISPLAY, "CM5 boot detected - releasing eink control")
        elif not power_state["cm5_powered"]:
            debug.log_info(debug.CAT_DISPLAY, "CM5 powered off - releasing eink control")
        elif repeat >= EINK_ANIMATION_TIMEOUT:
            debug.log_info(debug.CAT_DISPLAY, f"Animation timeout reached ({EINK_ANIMATION_TIMEOUT} cycles) - releasing eink control")
        else:
            debug.log_info(debug.CAT_DISPLAY, "Animation completed - releasing eink control")
            
        debug.log_info(debug.CAT_DISPLAY, "E-ink boot screen task completed")
        
        # Update eink display state to "unknown" since CM5 will take control
        global current_eink_display_state
        current_eink_display_state = "unknown"

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


def eink_display_task(image_paths):
    """E-ink display animation task with custom images - runs when CM5 is powered off"""
    global display_release_received, einkRunning, power_state
    
    try:
        # Check if CM5 is powered off before starting (opposite of boot screen)
        if power_state["cm5_powered"]:
            debug.log_info(debug.CAT_DISPLAY, "E-ink task aborted - CM5 is powered (use boot screen instead)")
            return
            
        # Validate image paths array
        if not image_paths or len(image_paths) == 0:
            debug.log_error(debug.CAT_DISPLAY, "E-ink task aborted - no image paths provided")
            return
            
        debug.log_info(debug.CAT_DISPLAY, f"Starting E-ink display task with {len(image_paths)} images")

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
        
        wdt.feed()
        
        if LUT_MODE:
            eink.epd_init_lut()
        else:
            eink.epd_init_fast()
            
        try:
            # Show the first image in the array
            eink.PIC_display(None, image_paths[0])
            debug.log_info(debug.CAT_DISPLAY, f"Initial image displayed: {image_paths[0]}")
        except OSError:
            debug.log_error(debug.CAT_DISPLAY, f"Image file not found: {image_paths[0]}")
            einkRunning = False
        
        wdt.feed()
        
        if LUT_MODE:
            utime.sleep_ms(1300)  # give time for first refresh, no lower than 1300
            debug.log_info(debug.CAT_DISPLAY, "LUT mode - waiting for refresh completion")
      
        wdt.feed()
        
        # Initialize animation variables
        repeat = 0
        
        # Animation loop only if we have multiple images
        if len(image_paths) == 1:
            debug.log_info(debug.CAT_DISPLAY, "Single image mode - no animation loop")
        else:
            # Animation loop with cycling through all images
            image_index = 0
            while einkRunning and not power_state["cm5_powered"] and repeat < EINK_ANIMATION_TIMEOUT:
                wdt.feed()
                with eink_lock:
                    if not einkRunning or repeat >= EINK_ANIMATION_TIMEOUT:
                        break
                
                # Check for power on before each animation step
                if power_state["cm5_powered"]:
                    debug.log_info(debug.CAT_DISPLAY, "CM5 powered on during animation - stopping")
                    break
                    
                try:
                    # Calculate current and next image indices
                    current_index = image_index % len(image_paths)
                    next_index = (image_index + 1) % len(image_paths)
                    
                    eink.epd_init_part()
                    eink.PIC_display(image_paths[current_index], image_paths[next_index])
                    
                    # Check during animation
                    if power_state["cm5_powered"]:
                        break
                    
                    wdt.feed()
                    repeat += 1
                    image_index += 1
                    
                    debug.log_verbose(debug.CAT_DISPLAY, f"Animation cycle {repeat} completed (image {current_index} -> {next_index})")
                    
                    # Yield and check for power state
                    for i in range(5):  # Brief yield with checks
                        if power_state["cm5_powered"]:
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
        
        if power_state["cm5_powered"]:
            debug.log_info(debug.CAT_DISPLAY, "CM5 powered on - releasing eink control")
        elif repeat >= EINK_ANIMATION_TIMEOUT:
            debug.log_info(debug.CAT_DISPLAY, f"Animation timeout reached ({EINK_ANIMATION_TIMEOUT} cycles) - releasing eink control")
        else:
            debug.log_info(debug.CAT_DISPLAY, "Animation completed - releasing eink control")
            
        debug.log_info(debug.CAT_DISPLAY, "E-ink display task completed")

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


# Note: E-ink boot screen task will only run when CM5 is powered on via SELECT button long press
# E-ink display task will only run when CM5 is powered off
# This prevents conflicts between the two display modes

# Main loop (runs on Core 0)
debug.log_info(debug.CAT_SYSTEM, "=== Main loop starting ===")
debug.log_info(debug.CAT_POWER, "System ready - press SELECT for 3s to power on CM5, or UP+DOWN for 5s to force shutdown")
set_debug_color("MAIN_LOOP")

# Main application loop
last_heartbeat = utime.ticks_ms()
HEARTBEAT_INTERVAL = 10000  # 10 seconds - reduced for better responsiveness

while True:
    try:
        wdt.feed()
        
        try:
            if power_manager is not None and power_manager.bq27441 is not None:
                if power_manager.get_current_ma() > 0:
                    set_debug_color("CHARGING")
                else:
                    set_debug_color("MAIN_LOOP") 
            else:
                set_debug_color("MAIN_LOOP")
        except Exception as e:
            debug.log_error(debug.CAT_SYSTEM, f"Error setting debug color: {e}")
            set_debug_color("MAIN_LOOP")
      
        # Periodic health checks (non-critical - can fail without affecting power management)
        try:
            current_time = utime.ticks_ms()
            if utime.ticks_diff(current_time, last_heartbeat) >= HEARTBEAT_INTERVAL:

                # Only do UART-related health checks and heartbeat when CM5 is powered
                if power_state["cm5_powered"]:
                    # Check UART handler health
                    uart_health = uart_handler.check_health()
                    if uart_health["status"] != "HEALTHY":
                        health_msg = "UART health: {} - {}".format(
                            uart_health["status"], uart_health["issues"]
                        )
                        debug.log_error(debug.CAT_SYSTEM, health_msg)

                    # Send heartbeat debug code to kernel
                    debug.send_debug_code(uart0, 0, 0x01, 0)  # System category, heartbeat code

                    debug.log_info(debug.CAT_SYSTEM, "System heartbeat (CM5 powered)")
                else:
                    # CM5 is off - just log local heartbeat without UART communication
                    debug.log_info(debug.CAT_SYSTEM, "System heartbeat (CM5 off - no UART)")

                last_heartbeat = current_time

        except Exception as e:
            # Heartbeat failed, but don't let it affect power management
            debug.log_error(debug.CAT_SYSTEM, f"Heartbeat error (non-critical): {e}")
            # Reset heartbeat timer to prevent continuous errors
            last_heartbeat = utime.ticks_ms()

        # Critical power management triggers (must always run)
        try:
            check_power_on_trigger()
            check_force_shutdown_trigger()
            handle_power_timers()
            check_emergency_battery_shutdown()
            wdt.feed()
        except Exception as e:
            # Power management error is serious - log but continue trying
            debug.log_error(debug.CAT_POWER, f"Power State (power on and off) error: {e}")
            set_debug_color("ERROR")

        # Battery status display (when CM5 is off)
        try:
            check_and_display_battery_status()
        except Exception as e:
            # Battery display error is non-critical - log but continue
            debug.log_error(debug.CAT_POWER, f"Battery display check error (non-critical): {e}")

        # Sleep to allow other tasks to run
        utime.sleep_ms(100)

    except Exception as e:
        # Outer catch-all for catastrophic errors
        debug.log_error(debug.CAT_SYSTEM, f"Main loop catastrophic error: {e}")
        set_debug_color("ERROR")
        utime.sleep_ms(1000)  # Longer delay only on catastrophic error
