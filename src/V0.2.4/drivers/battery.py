"""
BQ27441 Battery Fuel Gauge Driver
I2C interface to TI BQ27441 battery monitor
"""

import machine
import utime
import struct
import config

# BQ27441 Commands
CMD_CONTROL = 0x00
CMD_TEMP = 0x02
CMD_VOLTAGE = 0x04
CMD_FLAGS = 0x06
CMD_NOM_CAPACITY = 0x08
CMD_AVAIL_CAPACITY = 0x0A
CMD_REM_CAPACITY = 0x0C
CMD_FULL_CAPACITY = 0x0E
CMD_AVG_CURRENT = 0x10
CMD_STANDBY_CURRENT = 0x12
CMD_MAX_CURRENT = 0x14
CMD_AVG_POWER = 0x18
CMD_STATE_OF_CHARGE = 0x1C
CMD_INT_TEMP = 0x1E
CMD_STATE_OF_HEALTH = 0x20
CMD_REM_CAP_UNFILTERED = 0x28
CMD_REM_CAP_FILTERED = 0x2A
CMD_FULL_CAP_UNFILTERED = 0x2C
CMD_FULL_CAP_FILTERED = 0x2E
CMD_STATE_OF_CHARGE_UNFILTERED = 0x30

# Control Subcommands
CONTROL_STATUS = 0x0000
CONTROL_DEVICE_TYPE = 0x0001
CONTROL_FW_VERSION = 0x0002
CONTROL_DM_CODE = 0x0004
CONTROL_PREV_MACWRITE = 0x0007
CONTROL_CHEM_ID = 0x0008
CONTROL_BAT_INSERT = 0x000C
CONTROL_BAT_REMOVE = 0x000D
CONTROL_SET_HIBERNATE = 0x0011
CONTROL_CLEAR_HIBERNATE = 0x0012
CONTROL_SET_CFGUPDATE = 0x0013
CONTROL_SHUTDOWN_ENABLE = 0x001B
CONTROL_SHUTDOWN = 0x001C
CONTROL_SEALED = 0x0020
CONTROL_PULSE_SOC_INT = 0x0023
CONTROL_RESET = 0x0041
CONTROL_SOFT_RESET = 0x0042
CONTROL_EXIT_CFGUPDATE = 0x0043
CONTROL_EXIT_RESIM = 0x0044

# Flags bits
FLAG_DSG = 1 << 0  # Discharging
FLAG_SOCF = 1 << 1  # State of Charge Final
FLAG_SOC1 = 1 << 2  # State of Charge Initial
FLAG_BAT_DET = 1 << 3  # Battery Detected
FLAG_CHG = 1 << 8  # Charging
FLAG_FC = 1 << 9  # Full Charge
FLAG_OTD = 1 << 14  # Overtemperature in discharge
FLAG_OTC = 1 << 15  # Overtemperature in charge


class BQ27441:
    """BQ27441 Battery Fuel Gauge Driver"""

    def __init__(self, i2c_id=0, sda=24, scl=25, addr=0x55):
        self.addr = addr
        try:
            self.i2c = machine.I2C(
                i2c_id, scl=machine.Pin(scl), sda=machine.Pin(sda), freq=config.I2C_FREQ
            )
            self.available = self.init_gauge()
        except Exception as e:
            print(f"BQ27441: Init failed - {e}")
            self.available = False

        # Cached values
        self.last_update = 0
        self.update_interval = 1000  # 1 second cache
        self.cached_soc = 0
        self.cached_voltage = 0
        self.cached_current = 0
        self.cached_capacity = 0
        self.cached_flags = 0

    def init_gauge(self):
        """Initialize the fuel gauge"""
        try:
            # Check if device is present
            devices = self.i2c.scan()
            if self.addr not in devices:
                print(f"BQ27441: Not found at address 0x{self.addr:02X}")
                return False

            # Get device type
            device_type = self.read_control(CONTROL_DEVICE_TYPE)
            if device_type != 0x0421:  # BQ27441 device ID
                print(f"BQ27441: Unknown device type 0x{device_type:04X}")
                return False

            # Enter config update mode if needed
            if not self.enter_config():
                print("BQ27441: Failed to enter config mode")
                return False

            # Set design capacity (mAh)
            self.set_capacity(3000)  # 3000mAh default

            # Exit config update mode
            if not self.exit_config():
                print("BQ27441: Failed to exit config mode")
                return False

            print("BQ27441: Initialized successfully")
            return True

        except Exception as e:
            print(f"BQ27441: Init error - {e}")
            return False

    def read_register(self, reg, count=2):
        """Read register(s) from BQ27441"""
        try:
            data = self.i2c.readfrom_mem(self.addr, reg, count)
            if count == 1:
                return data[0]
            elif count == 2:
                return struct.unpack("<H", data)[0]
            else:
                return data
        except Exception as e:
            print(f"BQ27441: Read error reg 0x{reg:02X} - {e}")
            return 0

    def write_register(self, reg, value):
        """Write register to BQ27441"""
        try:
            if isinstance(value, int):
                if value > 255:
                    data = struct.pack("<H", value)
                else:
                    data = bytes([value])
            else:
                data = value
            self.i2c.writeto_mem(self.addr, reg, data)
            return True
        except Exception as e:
            print(f"BQ27441: Write error reg 0x{reg:02X} - {e}")
            return False

    def read_control(self, subcommand):
        """Read control subcommand"""
        self.write_register(CMD_CONTROL, subcommand)
        utime.sleep_ms(10)
        return self.read_register(CMD_CONTROL)

    def enter_config(self):
        """Enter configuration update mode"""
        try:
            self.write_register(CMD_CONTROL, CONTROL_SET_CFGUPDATE)
            utime.sleep_ms(100)

            # Verify we're in config mode
            flags = self.read_register(CMD_FLAGS)
            return (flags & 0x10) != 0  # CFGUPMODE flag
        except:
            return False

    def exit_config(self):
        """Exit configuration update mode"""
        try:
            self.write_register(CMD_CONTROL, CONTROL_EXIT_CFGUPDATE)
            utime.sleep_ms(100)

            # Verify we've exited config mode
            flags = self.read_register(CMD_FLAGS)
            return (flags & 0x10) == 0  # CFGUPMODE flag cleared
        except:
            return False

    def set_capacity(self, capacity_mah):
        """Set battery design capacity in mAh"""
        # This would require extended data commands
        # Simplified for this implementation
        self.design_capacity = capacity_mah

    def update_cache(self):
        """Update cached values"""
        now = utime.ticks_ms()
        if utime.ticks_diff(now, self.last_update) < self.update_interval:
            return

        try:
            self.cached_soc = self.read_register(CMD_STATE_OF_CHARGE, 1)
            self.cached_voltage = self.read_register(CMD_VOLTAGE)
            self.cached_current = struct.unpack(
                "<h", self.i2c.readfrom_mem(self.addr, CMD_AVG_CURRENT, 2)
            )[0]
            self.cached_capacity = self.read_register(CMD_REM_CAPACITY)
            self.cached_flags = self.read_register(CMD_FLAGS)
            self.last_update = now
        except Exception as e:
            print(f"BQ27441: Cache update error - {e}")

    def get_soc(self):
        """Get State of Charge (0-100%)"""
        if not self.available:
            return 0
        self.update_cache()
        return min(100, max(0, self.cached_soc))

    def get_voltage(self):
        """Get battery voltage in volts"""
        if not self.available:
            return 0.0
        self.update_cache()
        return self.cached_voltage / 1000.0  # Convert mV to V

    def get_current(self):
        """Get average current in mA (positive = charging)"""
        if not self.available:
            return 0
        self.update_cache()
        return self.cached_current

    def get_capacity(self):
        """Get remaining capacity in mAh"""
        if not self.available:
            return 0
        self.update_cache()
        return self.cached_capacity

    def get_temperature(self):
        """Get temperature in Celsius"""
        if not self.available:
            return 25.0
        try:
            temp_k = self.read_register(CMD_TEMP) / 10.0
            return temp_k - 273.15  # Convert K to C
        except:
            return 25.0

    def is_charging(self):
        """Check if battery is charging"""
        if not self.available:
            return False
        self.update_cache()
        return (self.cached_flags & FLAG_CHG) != 0

    def is_discharging(self):
        """Check if battery is discharging"""
        if not self.available:
            return False
        self.update_cache()
        return (self.cached_flags & FLAG_DSG) != 0

    def is_full(self):
        """Check if battery is fully charged"""
        if not self.available:
            return False
        self.update_cache()
        return (self.cached_flags & FLAG_FC) != 0

    def get_flags(self):
        """Get raw flags register"""
        if not self.available:
            return 0
        self.update_cache()
        return self.cached_flags

    def get_power(self):
        """Get power in mW"""
        if not self.available:
            return 0
        try:
            return self.read_register(CMD_AVG_POWER)
        except:
            return 0

    def reset(self):
        """Reset the fuel gauge"""
        if not self.available:
            return False
        try:
            self.write_register(CMD_CONTROL, CONTROL_RESET)
            utime.sleep_ms(100)
            return True
        except:
            return False

    def get_stats(self):
        """Get battery statistics"""
        self.update_cache()
        return {
            "soc": self.cached_soc,
            "voltage": self.cached_voltage / 1000.0,
            "current": self.cached_current,
            "capacity": self.cached_capacity,
            "charging": self.is_charging(),
            "full": self.is_full(),
            "available": self.available,
        }


class BatterySimulator:
    """Battery simulator for testing"""

    def __init__(self):
        self.soc = 75
        self.voltage = 3.7
        self.current = -100  # Discharging at 100mA
        self.capacity = 2250  # 75% of 3000mAh
        self.temperature = 25.0
        self.charging = False
        self.available = True

    def get_soc(self):
        """Simulate SOC"""
        return self.soc

    def get_voltage(self):
        """Simulate voltage"""
        return self.voltage

    def get_current(self):
        """Simulate current"""
        return self.current

    def get_capacity(self):
        """Simulate capacity"""
        return self.capacity

    def get_temperature(self):
        """Simulate temperature"""
        return self.temperature

    def is_charging(self):
        """Simulate charging state"""
        return self.charging

    def is_discharging(self):
        """Simulate discharging state"""
        return not self.charging

    def is_full(self):
        """Simulate full state"""
        return self.soc >= 100

    def set_charging(self, state):
        """Set charging state for simulation"""
        self.charging = state
        self.current = 500 if state else -100

    def simulate_discharge(self, percent):
        """Simulate discharge"""
        self.soc = max(0, self.soc - percent)
        self.capacity = int(3000 * self.soc / 100)
        self.voltage = 3.0 + (0.02 * self.soc)  # 3.0V to 5.0V

    def get_stats(self):
        """Get simulated statistics"""
        return {
            "soc": self.soc,
            "voltage": self.voltage,
            "current": self.current,
            "capacity": self.capacity,
            "charging": self.charging,
            "full": self.is_full(),
            "available": self.available,
        }
