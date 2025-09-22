"""Power Management Controller for Pamir SAM with BQ27441 integration"""

# 7-23 fixed

# pylint: disable=import-error,broad-exception-caught,global-statement,invalid-name
import _thread
import machine
from battery import BQ27441


class PowerManager:
    """Power Manager for Pamir SAM with BQ27441 battery management IC"""

    def __init__(self, design_capacity_mah=3000, debug_enabled=True):
        """Initialize Power Manager with BQ27441 battery management IC

        Args:
            design_capacity_mah: Battery design capacity for percentage calculations
            debug_enabled: Enable debug printing for sensor errors
        """
        self.design_capacity_mah = design_capacity_mah
        self.debug_enabled = debug_enabled

        # Power state tracking
        self.current_power_state = 0x01  # POWER_STATE_RUNNING
        self.shutdown_requested = False

        # Thread safety for sensor access
        self.sensor_lock = _thread.allocate_lock()

        # Cached sensor values (return these on I2C failures)
        self.cached_current_ma = 0
        self.cached_battery_percent = 0
        self.cached_temperature_0_1c = 0
        self.cached_voltage_mv = 0
        self.cache_valid = False

        # Initialize BQ27441 battery management IC
        self.bq27441 = None
        self._init_bq27441()

    def _init_bq27441(self):
        """Initialize BQ27441 battery management IC with error handling"""
        try:
            # Initialize I2C on pins 24 (SDA) and 25 (SCL)
            i2c = machine.I2C(0, sda=machine.Pin(24), scl=machine.Pin(25))

            # Initialize BQ27441 with design capacity
            self.bq27441 = BQ27441(i2c=i2c, address=0x55)

            if self.debug_enabled:
                print("[PowerManager] BQ27441 initialized successfully")

        except Exception as e:
            if self.debug_enabled:
                print(f"[PowerManager] BQ27441 initialization failed: {e}")
            self.bq27441 = None

    def _read_sensor_safe(self, sensor_method, sensor_name, fallback_value=0):
        """Safely read from BQ27441 sensor with error handling

        Args:
            sensor_method: BQ27441 method to call
            sensor_name: Name for debug printing
            fallback_value: Value to return on failure (always 0 as requested)

        Returns:
            Sensor value or 0 if failed
        """
        if self.bq27441 is None:
            if self.debug_enabled:
                print(f"[PowerManager] {sensor_name}: BQ27441 not initialized")
            return fallback_value

        try:
            with self.sensor_lock:
                value = sensor_method()
                return value
        except Exception as e:
            if self.debug_enabled:
                print(f"[PowerManager] {sensor_name} read failed: {e}")
            return fallback_value

    def get_current_ma(self):
        """Get current draw in milliamps from BQ27441

        Returns:
            int: Current draw in mA (positive = charging, negative = discharging)
                 Returns synthetic data if sensor unavailable
        """
        if self.bq27441 is None:
            if self.debug_enabled:
                print(f"[PowerManager] get_current_ma Sensor failed, sending 0 mA")
            return 0
            
        current_ma = self._read_sensor_safe(lambda: self.bq27441.avg_current_mA(), "Current", 0)

        # If sensor read failed, generate synthetic data
        if current_ma == 0:
            if self.debug_enabled:
                print(f"[PowerManager] get_current_ma Sensor failed, sending 0 mA")
            return 0
                
        # Cache the value for backup
        self.cached_current_ma = current_ma
        return current_ma

    def get_battery_percent(self):
        """Get battery charge percentage from BQ27441

        Returns:
            int: Battery percentage (0-100%), returns synthetic data if sensor unavailable
        """
        if self.bq27441 is None:
            if self.debug_enabled:
                print(f"[PowerManager] get_battery_percent Sensor failed, sending 0%")
            return 0

        # Get state of charge from BQ27441
        battery_percent = self._read_sensor_safe(
            lambda: self.bq27441.i2c_get_stateofcharge(), "Battery State of Charge", 0
        )

        # Cache the value
        self.cached_battery_percent = battery_percent
        return battery_percent

    def get_temperature_0_1c(self):
        """Get temperature in 0.1°C resolution from BQ27441

        Returns:
            int: Temperature in 0.1°C units (e.g., 251 = 25.1°C)
                 Returns synthetic data if sensor unavailable
        """
        if self.bq27441 is None:
            if self.debug_enabled:
                print(f"[PowerManager] get_temperature_0_1c Sensor failed, sending 0.0°C")
            return 0.0

        temp_celsius = self._read_sensor_safe(lambda: self.bq27441.temp_C(), "Temperature", 0.0)

        # Convert to 0.1°C resolution
        try:
            temp_0_1c = int(temp_celsius * 10)
            # Cache the value
            self.cached_temperature_0_1c = temp_0_1c
            return temp_0_1c

        except Exception as e:
            if self.debug_enabled:
                print(f"[PowerManager] get_temperature_0_1c Conversion failed, sending 0.0°C")
            return 0.0

    def get_voltage_mv(self):
        """Get voltage in millivolts from BQ27441

        Returns:
            int: Voltage in mV (e.g., 3800 = 3.8V)
                 Returns synthetic data if sensor unavailable
        """
        if self.bq27441 is None:
            if self.debug_enabled:
                print(f"[PowerManager] get_voltage_mv Sensor failed, sending 0V")
            return 0

        voltage_v = self._read_sensor_safe(lambda: self.bq27441.voltage_V(), "Voltage", 0.0)


        # Convert to millivolts
        try:
            voltage_mv = int(voltage_v * 1000)

            # Cache the value
            self.cached_voltage_mv = voltage_mv
            return voltage_mv

        except Exception as e:

            if self.debug_enabled:
                print(f"[PowerManager]get_voltage_mv Conversion failed, sending 0V")
            return 0

    def get_all_metrics(self):
        """Get all power metrics in one call

        Returns:
            dict: All sensor readings, uses synthetic data if sensors fail
        """
        
        # Try to get real sensor data
        metrics = {
            "current_ma": self.get_current_ma(),
            "battery_percent": self.get_battery_percent(),
            "temperature_0_1c": self.get_temperature_0_1c(),
            "voltage_mv": self.get_voltage_mv(),
        }
        
        return metrics

    def set_power_state(self, new_state):
        """Set current power state

        Args:
            new_state: New power state (POWER_STATE_* constants)
        """
        old_state = self.current_power_state
        self.current_power_state = new_state

        if self.debug_enabled:
            state_names = {0x00: "OFF", 0x01: "RUNNING", 0x02: "SUSPEND", 0x03: "SLEEP"}
            old_name = state_names.get(old_state, f"UNKNOWN({old_state})")
            new_name = state_names.get(new_state, f"UNKNOWN({new_state})")
            print(f"[PowerManager] Power state change: {old_name} → {new_name}")

    def get_power_state(self):
        """Get current power state

        Returns:
            int: Current power state
        """
        return self.current_power_state

    def handle_shutdown_command(self, shutdown_mode, reason_code):
        """Handle shutdown command from SoM

        Args:
            shutdown_mode: 0=normal, 1=emergency, 2=reboot
            reason_code: Optional reason code
        """
        self.shutdown_requested = True

        if self.debug_enabled:
            mode_names = {0: "Normal", 1: "Emergency", 2: "Reboot"}
            mode_name = mode_names.get(shutdown_mode, f"Unknown({shutdown_mode})")
            print(
                f"[PowerManager] Shutdown requested: {mode_name}, reason: {reason_code}"
            )

        # TODO: Add actual shutdown preparation here
        # - Save critical data
        # - Close files/connections
        # - Prepare for power loss

    def handle_sleep_command(self, delay_seconds, sleep_flags):
        """Handle sleep command from SoM

        Args:
            delay_seconds: Delay before entering sleep
            sleep_flags: Sleep configuration flags
        """
        if self.debug_enabled:
            print(
                f"[PowerManager] Sleep requested: delay={delay_seconds}s, flags=0x{sleep_flags:02X}"
            )

        # TODO: Add actual sleep mode implementation
        # - Reduce power consumption
        # - Disable non-essential peripherals
        # - Configure wake sources

    def get_status(self):
        """Get power manager status for debugging

        Returns:
            dict: Status information
        """
        return {
            "power_state": self.current_power_state,
            "shutdown_requested": self.shutdown_requested,
            "bq27441_initialized": self.bq27441 is not None,
            "design_capacity_mah": self.design_capacity_mah,
            "cached_metrics": {
                "current_ma": self.cached_current_ma,
                "battery_percent": self.cached_battery_percent,
                "temperature_0_1c": self.cached_temperature_0_1c,
                "voltage_mv": self.cached_voltage_mv,
            },
        }
