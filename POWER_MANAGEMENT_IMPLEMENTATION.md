# Power Management Implementation for RP2040 SAM Firmware

## Overview

This implementation provides comprehensive power management for the Raspberry Pi Compute Module 5 (CM5) through the RP2040 microcontroller. The system controls CM5 power via the `pmic_enable` pin and implements various power-on/off scenarios with proper timing and safety mechanisms.

## Key Features

### 1. **Initial Power-Off State**
- When the RP2040 firmware starts, it immediately sets `pmic_enable` to output LOW
- This ensures the CM5 is powered off by default
- Prevents unexpected boot sequences and conserves power

### 2. **Power-On Trigger (SELECT Button Long Press)**
- User must press and hold the SELECT button for 3+ seconds to power on CM5
- `pmic_enable` is set to input mode (high-impedance) which allows CM5 to boot
- E-ink boot animation starts automatically after power-on
- Includes debouncing and proper timing validation

### 3. **Normal Shutdown (UART Command)**
- When CM5 sends shutdown notification via UART (POWER_CMD_SHUTDOWN)
- RP2040 acknowledges the command immediately
- After a 2-second delay, `pmic_enable` is set to output LOW
- Provides CM5 time to complete its shutdown sequence cleanly

### 4. **Force Shutdown (UP+DOWN Buttons)**
- User can press and hold UP+DOWN buttons for 5+ seconds
- Sends a power_pressed button event to warn CM5 of impending shutdown
- After 3-second delay, forcibly powers off CM5 regardless of its state
- Used for emergency shutdown or when CM5 becomes unresponsive

## Implementation Details

### Power State Management

```python
power_state = {
    "cm5_powered": False,                # Current CM5 power state
    "shutdown_pending": False,           # Normal shutdown in progress
    "shutdown_timer": 0,                 # Timestamp for shutdown delay
    "force_shutdown_pending": False,     # Force shutdown in progress
    "force_shutdown_timer": 0,           # Timestamp for force shutdown delay
    "select_press_start": 0,            # SELECT button press start time
    "force_press_start": 0,             # UP+DOWN press start time
}
```

### Timing Constants

- **SELECT_LONG_PRESS_MS**: 3000ms (3 seconds for power-on)
- **FORCE_SHUTDOWN_PRESS_MS**: 5000ms (5 seconds for force shutdown trigger)
- **NORMAL_SHUTDOWN_DELAY_MS**: 2000ms (2 seconds delay for normal shutdown)
- **FORCE_SHUTDOWN_DELAY_MS**: 3000ms (3 seconds delay for force shutdown)

### Core Functions

#### `set_cm5_power(enable)`
- Controls the `pmic_enable` pin
- `enable=True`: Set pin to input (high-Z) → CM5 powers on
- `enable=False`: Set pin to output LOW → CM5 powers off
- Automatically resets system state when powering off

#### `check_power_on_trigger()`
- Monitors SELECT button for long press
- Tracks press duration and triggers power-on after 3 seconds
- Starts E-ink boot animation after power-on

#### `check_force_shutdown_trigger()`
- Monitors UP+DOWN button combination
- Sends warning to CM5 via power_pressed button event
- Initiates force shutdown sequence after 5-second press

#### `handle_power_timers()`
- Manages shutdown delay timers
- Executes actual power-off after appropriate delays
- Handles both normal and force shutdown scenarios

### UART Protocol Integration

The implementation properly integrates with the SAM UART protocol:

#### Shutdown Notification Handling
```python
elif cmd_type == "shutdown":
    shutdown_mode = power_data.get("shutdown_mode", 0)
    reason_code = power_data.get("reason_code", 0)
    
    # Start 2-second countdown for power-off
    if power_state["cm5_powered"] and not power_state["shutdown_pending"]:
        power_state["shutdown_pending"] = True
        power_state["shutdown_timer"] = utime.ticks_ms()
        
    # Send immediate acknowledgment
    packet = protocol.create_power_status_packet_rp2040_to_som(
        protocol.POWER_STATE_OFF, shutdown_mode
    )
    uart0.write(packet)
```

#### Force Shutdown Warning
```python
# Send power_pressed event to CM5
packet = protocol.create_button_packet(
    up_pressed=False, down_pressed=False, 
    select_pressed=False, power_pressed=True
)
uart0.write(packet)
```

### E-ink Integration

The E-ink boot animation system is integrated with power management:

- **Boot Animation**: Only runs when CM5 is powered on
- **Power State Monitoring**: Animation stops if CM5 is powered off during execution
- **Display Handover**: Properly handles display release signals from CM5
- **Resource Cleanup**: Ensures E-ink resources are released on power-off

### Safety Features

#### State Reset on Power-Off
```python
def reset_system_state():
    """Reset system state when CM5 is powered off"""
    global display_release_received, einkRunning, power_state
    
    display_release_received = False
    einkRunning = False  # Stop any running E-ink operations
    power_state["shutdown_pending"] = False
    power_state["force_shutdown_pending"] = False
    # Reset all timers
```

#### UART Communication Guards
- Boot notifications only sent when CM5 is powered
- Display commands properly validated against power state
- Error handling for communication attempts when CM5 is off

#### Debouncing and Timing
- Proper button debouncing for all power control triggers
- Non-blocking timing implementation using `utime.ticks_ms()`
- Graceful handling of timing edge cases

## Power State Flow Diagrams

### Normal Boot Sequence
```
1. RP2040 starts → pmic_enable = LOW (CM5 OFF)
2. User presses SELECT for 3s → pmic_enable = HIGH-Z (CM5 ON)
3. E-ink animation starts → CM5 boots
4. CM5 sends display release → E-ink control handed to CM5
5. Normal operation with UART communication
```

### Normal Shutdown Sequence
```
1. CM5 initiates shutdown → Sends POWER_CMD_SHUTDOWN via UART
2. RP2040 acknowledges → Starts 2-second timer
3. RP2040 waits 2s → CM5 completes shutdown
4. Timer expires → pmic_enable = LOW (CM5 OFF)
5. System state reset → Ready for next power cycle
```

### Force Shutdown Sequence
```
1. User holds UP+DOWN for 5s → Force shutdown triggered
2. RP2040 sends power_pressed event → Warns CM5
3. RP2040 starts 3s timer → Countdown for force power-off
4. Timer expires → pmic_enable = LOW (CM5 OFF immediately)
5. System state reset → Ready for next power cycle
```

## Testing and Validation

### Test Scenarios

1. **Cold Boot Test**
   - Power on RP2040 → Verify CM5 is OFF
   - Press SELECT for 3s → Verify CM5 powers on
   - Verify E-ink animation starts

2. **Normal Shutdown Test**
   - With CM5 running → Send shutdown command via UART
   - Verify 2-second delay → Verify CM5 powers off
   - Verify state reset

3. **Force Shutdown Test**
   - With CM5 running → Hold UP+DOWN for 5s
   - Verify warning sent → Verify 3-second delay
   - Verify CM5 powers off immediately

4. **Edge Case Tests**
   - Button release before timeout
   - Multiple button presses
   - Power-off during E-ink animation
   - UART communication during power transitions

### Debug and Monitoring

The implementation includes comprehensive debug logging:

```python
debug.log_info(debug.CAT_POWER, "CM5 powered ON (pmic_enable -> high-z)")
debug.log_info(debug.CAT_POWER, "SELECT long press (3241ms) - powering on CM5")
debug.log_info(debug.CAT_POWER, "Shutdown notification received: mode=0, reason=5, will power off CM5 in 2000ms")
debug.log_info(debug.CAT_POWER, "Force shutdown delay (3005ms) complete - powering off CM5")
```

## Benefits

1. **Power Efficiency**: CM5 is off by default, reducing power consumption
2. **User Control**: Clear button sequences for power control
3. **Safe Shutdown**: Proper delays for clean shutdown sequences
4. **Emergency Control**: Force shutdown for unresponsive situations
5. **State Management**: Comprehensive state tracking and reset
6. **Protocol Integration**: Full integration with SAM UART protocol
7. **Debug Visibility**: Extensive logging for troubleshooting

## Future Enhancements

1. **Configurable Timing**: Make button press and delay times configurable
2. **Power Monitoring**: Add current/voltage monitoring during power transitions
3. **Wake-on-Event**: Support for additional wake sources
4. **Sleep Modes**: Implement RP2040 sleep modes when CM5 is off
5. **Battery Integration**: Enhanced battery management during power transitions

## Usage

The power management system is fully automatic once the firmware is loaded:

1. **Power On**: Hold SELECT button for 3+ seconds
2. **Normal Use**: CM5 boots and operates normally with UART communication
3. **Normal Shutdown**: CM5 sends shutdown command, automatic 2s delay power-off
4. **Emergency Shutdown**: Hold UP+DOWN for 5+ seconds for immediate power-off

The system is designed to be robust, safe, and user-friendly while providing full control over CM5 power management through the RP2040 microcontroller. 