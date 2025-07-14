#Author: PamirAI 
#Date: 2025-07-14
#Version: 1.0.0
#Description: This is the main program for the RP2040 SAM

import machine
import utime
from eink_driver_sam import einkDSP_SAM
import _thread
from machine import WDT
import math
from pamir_uart_protocols import PamirUartProtocols
from neopixel_controller import NeoPixelController
from power_manager import PowerManager

pmic_enable = machine.Pin(3, machine.Pin.IN, pull=None)

PRODUCTION = True  #for production flash, set to true for usb debug
UART_DEBUG = False #for UART debug, set to true for UART debug

# Initialize protocol handler
protocol = PamirUartProtocols()

USB_SWITCH_TABLE = {"SAM_USB": [0,0], "SOM_USB": [1,0]} # S, OE_N
# usb_switch_oe_n = machine.Pin(19, machine.Pin.OUT, value = 0)
usb_switch_s = machine.Pin(23, machine.Pin.OUT, value = 0)

def switch_usb(usb_type):
    if usb_type in USB_SWITCH_TABLE:
        S, _ = USB_SWITCH_TABLE[usb_type]
        usb_switch_s.value(S)
        # usb_switch_oe_n.value(OE_N)
    else:
        print(f"Invalid USB type: {usb_type}")


wdt = WDT(timeout=2000)
# Set up GPIO pins
selectBTN = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_DOWN)
upBTN = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_DOWN)
downBTN = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_DOWN)
einkStatus = machine.Pin(9, machine.Pin.OUT)
einkMux = machine.Pin(22, machine.Pin.OUT)
sam_interrupt = machine.Pin(2, machine.Pin.OUT)
# Debounce time in milliseconds
debounce_time = 50

if PRODUCTION:
    switch_usb("SOM_USB") # Disable SAM USB
    
# Setup UART2 on GPIO4 (TX) and GPIO5 (RX) - matches CM5 UART2 connection
uart2 = machine.UART(0, baudrate=115200, tx=machine.Pin(4), rx=machine.Pin(5))
einkMux.low()  # EINK OFF
einkStatus.low()  # SOM CONTROL E-INK

print("Starting...")

# Function to handle UART debug messages
def debug_print(message):
    if UART_DEBUG:
        uart2.write(message)
    print(message)


# Add lock for shared variable
eink_lock = _thread.allocate_lock()
einkRunning = False
uart_lock = _thread.allocate_lock()  # Add lock for UART handling

# Inter-core communication queues
led_command_queue = []
led_queue_lock = _thread.allocate_lock()

power_command_queue = []
power_queue_lock = _thread.allocate_lock()

# LED completion callback function
def led_completion_callback(led_id, sequence_length):
    """Callback function called when LED animations complete"""
    global protocol, uart2, uart_lock, PRODUCTION
    
    try:
        with uart_lock:
            if sequence_length > 0:
                # Send completion acknowledgment
                packet = protocol.create_led_completion_packet(led_id, sequence_length)
                uart2.write(packet)
                uart2.flush()  # Ensure immediate transmission
                if not PRODUCTION:
                    print(f"LED completion ACK sent: LED{led_id}, {sequence_length} commands")
            elif sequence_length < 0:
                # Send error report (sequence_length is negative error code)
                error_code = abs(sequence_length)
                packet = protocol.create_led_error_packet(led_id, error_code)
                uart2.write(packet)
                uart2.flush()  # Ensure immediate transmission
                if not PRODUCTION:
                    print(f"LED error ACK sent: LED{led_id}, error {error_code}")
    except Exception as e:
        print(f"LED acknowledgment failed: {e}")

# Initialize controllers
np_controller = NeoPixelController(pin=20, num_leds=1, default_brightness=0.5, 
                                   completion_callback=led_completion_callback)

# Initialize power manager with BQ27441 (3000mAh design capacity)
power_manager = PowerManager(design_capacity_mah=3000, debug_enabled=not PRODUCTION)

def add_led_command_to_queue(command):
    """Thread-safe function to add LED command to inter-core queue"""
    global led_command_queue
    with led_queue_lock:
        led_command_queue.append(command)

def get_led_commands_from_queue():
    """Thread-safe function to get all LED commands from queue"""
    global led_command_queue
    with led_queue_lock:
        commands = led_command_queue.copy()
        led_command_queue.clear()
        return commands

def add_power_command_to_queue(command):
    """Thread-safe function to add power command to inter-core queue"""
    global power_command_queue
    with power_queue_lock:
        power_command_queue.append(command)

def get_power_commands_from_queue():
    """Thread-safe function to get all power commands from queue"""
    global power_command_queue
    with power_queue_lock:
        commands = power_command_queue.copy()
        power_command_queue.clear()
        return commands

def process_uart_packet(packet_bytes):
    """Process received UART packet and queue appropriate commands"""
    packet_type = protocol.get_packet_type(packet_bytes)
    
    if packet_type == protocol.TYPE_LED:
        # Parse LED packet
        valid, led_data = protocol.parse_led_packet(packet_bytes)
        if valid:
            # Add to LED command queue for core 0 to process
            add_led_command_to_queue(led_data)
            if not PRODUCTION:
                print(f"LED command queued: {led_data}")
        else:
            # Try parsing as LED acknowledgment/status request
            valid_ack, ack_data = protocol.parse_led_acknowledgment(packet_bytes)
            if valid_ack and ack_data['type'] == 'status':
                # Handle LED status request
                add_led_command_to_queue({
                    'type': 'status_request',
                    'led_id': ack_data['led_id'],
                    'status_code': ack_data['status_code']
                })
                if not PRODUCTION:
                    print(f"LED status request: {ack_data}")
    
    elif packet_type == protocol.TYPE_POWER:
        # Parse power packet (SoM → RP2040 commands)
        valid, power_data = protocol.parse_power_packet(packet_bytes)
        if valid:
            # Add to power command queue for core 0 to process
            add_power_command_to_queue(power_data)
            if not PRODUCTION:
                print(f"Power command queued: {power_data}")
        else:
            if not PRODUCTION:
                print(f"Invalid power packet: {[hex(b) for b in packet_bytes]}")
    
    elif packet_type == protocol.TYPE_BUTTON:
        # Handle button packets if needed (currently only send, not receive)
        pass
    
    elif packet_type == protocol.TYPE_SYSTEM:
        # Parse system packet (SoM → RP2040 system commands)
        valid, system_data = protocol.parse_system_packet(packet_bytes)
        if valid:
            cmd_type = system_data.get('command', 'unknown')
            
            if cmd_type == 'ping':
                # Respond to ping with pong
                try:
                    with uart_lock:
                        pong_packet = protocol.create_system_pong_packet()
                        uart2.write(pong_packet)
                        uart2.flush()  # Ensure immediate transmission
                        if not PRODUCTION:
                            print("Ping received, pong sent")
                except Exception as e:
                    print(f"Failed to send pong: {e}")
            
            elif cmd_type == 'version_request':
                # Send firmware version response
                try:
                    with uart_lock:
                        version_packet = protocol.create_firmware_version_packet()
                        uart2.write(version_packet)
                        uart2.flush()  # Ensure immediate transmission
                        if not PRODUCTION:
                            print(f"Version sent: {protocol.FIRMWARE_VERSION_MAJOR}.{protocol.FIRMWARE_VERSION_MINOR}.{protocol.FIRMWARE_VERSION_PATCH}")
                except Exception as e:
                    print(f"Failed to send version: {e}")
            
            elif cmd_type == 'reset':
                # Handle reset command
                reset_type = system_data.get('reset_type', 0)
                if not PRODUCTION:
                    print(f"Reset command received: type={reset_type}")
                # TODO: Implement actual reset logic based on reset_type
            
            else:
                if not PRODUCTION:
                    print(f"Unknown system command: {cmd_type}")
        else:
            if not PRODUCTION:
                print(f"Invalid system packet: {[hex(b) for b in packet_bytes]}")
    
    else:
        if not PRODUCTION:
            print(f"Unknown packet type: 0x{packet_type:02X}")




# Function to debounce button press
def debounce(pin):
    state = pin.value()
    utime.sleep_ms(debounce_time)
    if pin.value() != state:
        return False
    return True

def get_debounced_state(pin):
    return pin.value() and debounce(pin) 

def send_button_state():
    # Get current button states with debouncing
    up_pressed = get_debounced_state(upBTN)
    down_pressed = get_debounced_state(downBTN)
    select_pressed = get_debounced_state(selectBTN)
    power_pressed = False  # Power button logic handled separately
    
    # Create protocol packet according to Pamir UART specification
    packet = protocol.create_button_packet(
        up_pressed=up_pressed,
        down_pressed=down_pressed, 
        select_pressed=select_pressed,
        power_pressed=power_pressed
    )
    
    if not PRODUCTION:
        print(f"Button packet: {[hex(b) for b in packet]}")
        print(f"Button states - UP: {up_pressed}, DOWN: {down_pressed}, SELECT: {select_pressed}")
    
    uart2.write(packet)
    uart2.flush()  # Ensure immediate transmission

# Interrupt handler for down button
def button_handler(pin):
    if debounce(pin):
        send_button_state()

def loading_terminator(pin):
    #Reserved for future use
    pass


# Set up interrupt handlers
selectBTN.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler)
upBTN.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler)
downBTN.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=button_handler)
sam_interrupt.irq(trigger=machine.Pin.IRQ_RISING, handler=loading_terminator)

debug_print(f"[RP2040 DEBUG] Initialized NeoPixel Controller\n")

einkStatus.high() # provide power to eink
einkMux.high() # SAM CONTROL E-INK

# Initialize e-ink display
eink = einkDSP_SAM()

debug_print("StartScreen\n")

# Send boot notification to SoM (Raspberry Pi 5)
def send_boot_notification():
    """Send boot notification FROM RP2040 TO SoM to indicate RP2040 is running"""
    try:
        with uart_lock:
            # Send power status packet indicating we're running
            packet = protocol.create_power_status_packet_rp2040_to_som(
                protocol.POWER_STATE_RUNNING, 0x00)
            uart2.write(packet)
            # Ensure packet is transmitted immediately
            uart2.flush()
            if not PRODUCTION:
                print("[Boot] Boot notification sent to SoM")
    except Exception as e:
        print(f"[Boot] Failed to send boot notification: {e}")

# Boot notification will be sent after UART reception starts

# Shared flag to coordinate thread handoff
thread_handoff_complete = False

# Thread to handle both eink and UART tasks
def core1_task():
    global einkRunning, thread_handoff_complete
    
    # First, run the eink task
    try:
        einkRunning = True
        if eink.init == False:
            eink.re_init()
        
        eink.epd_init_fast()
            
        try:
            with open('./loading1.bin', 'rb') as f:
                base_image = f.read()
            eink.epd_set_basemap(base_image)
            
        except OSError:
            print("Loading files not found")
            einkRunning = False
        
        repeat = 0
        while True:
            with eink_lock:
                if not einkRunning or repeat >= 3:
                    break
            
            try:
                with open('./loading2.bin', 'rb') as f:
                    image2_data = f.read()
                eink.epd_display_part_all(image2_data)
                utime.sleep_ms(50)  # Short delay between frames
                
                with open('./loading1.bin', 'rb') as f:
                    image1_data = f.read()
                eink.epd_display_part_all(image1_data)
                utime.sleep_ms(50)
            except OSError:
                print("Animation files not found")
                break
                
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
    
    # Signal that eink is done and we can transition to UART
    thread_handoff_complete = True
    
    # Start UART packet reception loop (Core 1)
    print("Starting UART reception loop on Core 1")
    uart_buffer = bytearray()
    
    # Send boot notification now that UART reception is ready
    send_boot_notification()
    
    while True:
        try:
            wdt.feed()
            
            # Check for incoming UART data
            if uart2.any():
                with uart_lock:
                    data = uart2.read()
                    if data:
                        uart_buffer.extend(data)
                        
                        # Process complete 4-byte packets
                        while len(uart_buffer) >= 4:
                            packet = bytes(uart_buffer[:4])
                            uart_buffer = uart_buffer[4:]
                            
                            # Validate and process packet
                            if protocol.validate_packet(packet)[0]:
                                process_uart_packet(packet)
                            else:
                                if not PRODUCTION:
                                    print(f"Invalid packet: {[hex(b) for b in packet]}")
            
            utime.sleep_ms(1)  # Small delay to prevent overwhelming the CPU
            
        except Exception as e:
            print(f"UART reception error: {e}")
            utime.sleep_ms(10)

# Start the combined thread on core1
_thread.start_new_thread(core1_task, ())
print("Started core1 task")

# Clean main loop
while True:
    wdt.feed()
    
    # Only proceed with normal operation after handoff is complete
    if thread_handoff_complete:
        # Process LED commands from queue (Core 0)
        led_commands = get_led_commands_from_queue()
        for command in led_commands:
            try:
                if command.get('type') == 'status_request':
                    # Handle LED status request
                    led_id = command['led_id']
                    status_code = command.get('status_code', 0)
                    
                    # Get controller status
                    controller_status = np_controller.get_status()
                    
                    # Send status response based on status_code
                    if status_code == 0:  # General status
                        status_value = 1 if controller_status['animation_running'] else 0
                    elif status_code == 1:  # Queue length
                        status_value = controller_status['queue_length']
                    elif status_code == 2:  # Brightness
                        status_value = int(controller_status['brightness'] * 255)
                    else:  # Unknown status code
                        status_value = 0
                    
                    # Send status packet
                    with uart_lock:
                        packet = protocol.create_led_status_packet(led_id, status_code, status_value)
                        uart2.write(packet)
                        if not PRODUCTION:
                            print(f"LED status response: LED{led_id}, code{status_code}, value{status_value}")
                
                elif command.get('execute'):
                    # Execute all queued LED commands
                    np_controller.execute_queue()
                else:
                    # Add command to neopixel controller queue
                    led_id = command['led_id']
                    if led_id == 15:  # Convert protocol LED_ALL to controller format
                        led_id = 255
                    
                    # Determine animation mode based on time_value and other factors
                    time_value = command['time_value']
                    if time_value == 0:
                        mode = np_controller.MODE_STATIC
                    elif time_value <= 5:
                        mode = np_controller.MODE_BLINK
                    elif time_value <= 10:
                        mode = np_controller.MODE_FADE
                    else:
                        mode = np_controller.MODE_RAINBOW
                    
                    np_controller.add_to_queue(
                        led_id=led_id,
                        mode=mode,
                        color_data=command['color'],
                        time_value=time_value
                    )
                    
                    if not PRODUCTION:
                        print(f"LED command added to controller queue: LED{led_id}, color{command['color']}, mode{mode}")
                        
            except Exception as e:
                print(f"Error processing LED command: {e}")
                # Send error report
                try:
                    error_led_id = command.get('led_id', 0)
                    if error_led_id == 15:  # Convert protocol LED_ALL back
                        error_led_id = 255
                    np_controller.send_error_report(error_led_id, 1, str(e))
                except:
                    pass  # Don't let error reporting crash the system
        
        # Process power commands from queue (Core 0)
        power_commands = get_power_commands_from_queue()
        for command in power_commands:
            try:
                cmd_type = command.get('command', 'unknown')
                
                if cmd_type == 'query':
                    # SoM → RP2040: Query power status
                    current_state = power_manager.get_power_state()
                    with uart_lock:
                        packet = protocol.create_power_status_packet_rp2040_to_som(current_state, 0x00)
                        uart2.write(packet)
                        if not PRODUCTION:
                            print(f"Power status response sent: state=0x{current_state:02X}")
                
                elif cmd_type == 'set_state':
                    # SoM → RP2040: Set power state
                    new_state = command.get('power_state', power_manager.current_power_state)
                    power_manager.set_power_state(new_state)
                    # Send acknowledgment
                    with uart_lock:
                        packet = protocol.create_power_status_packet_rp2040_to_som(new_state, 0x00)
                        uart2.write(packet)
                        if not PRODUCTION:
                            print(f"Power state set to: 0x{new_state:02X}")
                
                elif cmd_type == 'sleep':
                    # SoM → RP2040: Enter sleep mode
                    delay_seconds = command.get('delay_seconds', 0)
                    sleep_flags = command.get('sleep_flags', 0)
                    power_manager.handle_sleep_command(delay_seconds, sleep_flags)
                
                elif cmd_type == 'shutdown':
                    # SoM → RP2040: Prepare for shutdown
                    shutdown_mode = command.get('shutdown_mode', 0)
                    reason_code = command.get('reason_code', 0)
                    power_manager.handle_shutdown_command(shutdown_mode, reason_code)
                    
                    # Send shutdown acknowledgment
                    with uart_lock:
                        packet = protocol.create_power_status_packet_rp2040_to_som(
                            protocol.POWER_STATE_OFF, shutdown_mode)
                        uart2.write(packet)
                        if not PRODUCTION:
                            print(f"Shutdown ACK sent: mode={shutdown_mode}")
                
                elif cmd_type == 'request_metrics':
                    # SoM → RP2040: Send all sensor metrics
                    metrics = power_manager.get_all_metrics()
                    
                    with uart_lock:
                        # Send current measurement
                        current_packet = protocol.create_power_metrics_packet_rp2040_to_som(
                            protocol.POWER_CMD_CURRENT, metrics['current_ma'])
                        uart2.write(current_packet)
                        uart2.flush()  # Ensure immediate transmission
                        
                        # Send battery percentage
                        battery_packet = protocol.create_power_metrics_packet_rp2040_to_som(
                            protocol.POWER_CMD_BATTERY, metrics['battery_percent'])
                        uart2.write(battery_packet)
                        uart2.flush()  # Ensure immediate transmission
                        
                        # Send temperature
                        temp_packet = protocol.create_power_metrics_packet_rp2040_to_som(
                            protocol.POWER_CMD_TEMP, metrics['temperature_0_1c'])
                        uart2.write(temp_packet)
                        uart2.flush()  # Ensure immediate transmission
                        
                        # Send voltage
                        voltage_packet = protocol.create_power_metrics_packet_rp2040_to_som(
                            protocol.POWER_CMD_VOLTAGE, metrics['voltage_mv'])
                        uart2.write(voltage_packet)
                        uart2.flush()  # Ensure immediate transmission
                        
                        if not PRODUCTION:
                            print(f"Metrics sent: {metrics}")
                
                else:
                    if not PRODUCTION:
                        print(f"Unknown power command: {cmd_type}")
                        
            except Exception as e:
                print(f"Error processing power command: {e}")
        
        # Special button combination handling (UP + SELECT for 10 seconds = USB switch)
        if debounce(upBTN) and selectBTN.value() == 1:
            start_time = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), start_time) < 10000:
                if upBTN.value() == 0 or selectBTN.value() == 0:
                    break
                if utime.ticks_diff(utime.ticks_ms(), start_time) >= 2000:
                    # Send shutdown packet using new protocol
                    with uart_lock:
                        packet = protocol.create_power_status_packet_rp2040_to_som(
                            protocol.POWER_STATE_OFF, 0x00)
                        uart2.write(packet)
                wdt.feed()
                utime.sleep_ms(10)
            if utime.ticks_diff(utime.ticks_ms(), start_time) >= 10000:
                einkStatus.low()  
                einkMux.low() # SOM CONTROL E-INK
                uart2.write("xSAM_USB\n")
                if PRODUCTION:
                    switch_usb("SAM_USB")
                einkRunning = False
    
    utime.sleep_ms(1)






