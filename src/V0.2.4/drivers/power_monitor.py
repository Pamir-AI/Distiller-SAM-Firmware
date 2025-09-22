"""
Power Management and Monitoring
Handles power modes, battery monitoring, and system power state
"""

import machine
import utime
import uasyncio
from protocol.sam_defs import *
from drivers.battery import BQ27441, BatterySimulator
import config


class PowerMonitor:
    """System power monitor and manager"""

    def __init__(self, use_simulator=False):
        # Battery monitor
        if use_simulator:
            self.battery = BatterySimulator()
        else:
            self.battery = BQ27441(
                i2c_id=config.I2C_ID,
                sda=config.I2C_SDA,
                scl=config.I2C_SCL,
                addr=config.BQ27441_ADDR,
            )

        # Power state
        self.power_mode = POWER_MODE_NORMAL
        self.last_activity = utime.ticks_ms()
        self.idle_start = None

        # Callbacks
        self.on_battery_update = None
        self.on_low_battery = None
        self.on_critical_battery = None

        # Thresholds
        self.low_battery_threshold = 15  # 15%
        self.critical_battery_threshold = 5  # 5%
        self.sleep_threshold = config.IDLE_SLEEP_TIMEOUT

        # Monitoring task
        self.running = False
        self.monitor_task_handle = None

        # Statistics
        self.stats = {
            "updates": 0,
            "low_battery_events": 0,
            "critical_battery_events": 0,
            "sleep_count": 0,
        }

        # Last reported values
        self.last_soc = 0
        self.last_voltage = 0
        self.last_charging = False

    def start(self):
        """Start power monitoring"""
        if not self.running:
            self.running = True
            self.monitor_task_handle = uasyncio.create_task(self.monitor_task())
            print("PowerMonitor: Started")

    def stop(self):
        """Stop power monitoring"""
        self.running = False
        if self.monitor_task_handle:
            self.monitor_task_handle.cancel()
            self.monitor_task_handle = None
        print("PowerMonitor: Stopped")

    async def monitor_task(self):
        """Power monitoring task"""
        last_update = 0

        while self.running:
            try:
                now = utime.ticks_ms()

                # Update battery status periodically
                if utime.ticks_diff(now, last_update) >= config.BATTERY_UPDATE_INTERVAL:
                    self.update_battery_status()
                    last_update = now

                # Check idle timeout for sleep mode
                if self.power_mode == POWER_MODE_NORMAL:
                    idle_time = utime.ticks_diff(now, self.last_activity)
                    if idle_time >= self.sleep_threshold:
                        self.enter_sleep_mode()

                # Cooperative yield
                await uasyncio.sleep_ms(100)

            except Exception as e:
                print(f"Power monitor error: {e}")

    def update_battery_status(self):
        """Update battery status and trigger events"""
        try:
            soc = self.battery.get_soc()
            voltage = self.battery.get_voltage()
            charging = self.battery.is_charging()

            self.stats["updates"] += 1

            # Check for significant changes
            soc_changed = abs(soc - self.last_soc) >= 1
            voltage_changed = abs(voltage - self.last_voltage) >= 0.05
            charge_changed = charging != self.last_charging

            if soc_changed or voltage_changed or charge_changed:
                self.last_soc = soc
                self.last_voltage = voltage
                self.last_charging = charging

                # Trigger update callback
                if self.on_battery_update:
                    self.on_battery_update(soc, voltage, charging)

                # Check thresholds
                if not charging:
                    if soc <= self.critical_battery_threshold:
                        self.stats["critical_battery_events"] += 1
                        if self.on_critical_battery:
                            self.on_critical_battery(soc)
                        # Enter shutdown mode on critical battery
                        if soc <= 2:
                            self.enter_shutdown_mode()
                    elif soc <= self.low_battery_threshold:
                        self.stats["low_battery_events"] += 1
                        if self.on_low_battery:
                            self.on_low_battery(soc)

        except Exception as e:
            print(f"Battery update error: {e}")

    def set_power_mode(self, mode):
        """Set system power mode"""
        if mode == self.power_mode:
            return

        old_mode = self.power_mode
        self.power_mode = mode

        if mode == POWER_MODE_NORMAL:
            self.exit_low_power()
        elif mode == POWER_MODE_LOW:
            self.enter_low_power()
        elif mode == POWER_MODE_SLEEP:
            self.enter_sleep_mode()
        elif mode == POWER_MODE_SHUTDOWN:
            self.enter_shutdown_mode()

        print(f"Power mode: {old_mode} -> {mode}")

    def enter_low_power(self):
        """Enter low power mode"""
        # TODO: Reduce CPU frequency
        # machine.freq(62500000)  # Reduce to 62.5MHz
        print("PowerMonitor: Low power mode")

    def exit_low_power(self):
        """Exit low power mode"""
        # TODO: Restore CPU frequency
        # machine.freq(125000000)  # Restore to 125MHz
        self.update_activity()
        print("PowerMonitor: Normal power mode")

    def enter_sleep_mode(self):
        """Enter sleep mode"""
        self.power_mode = POWER_MODE_SLEEP
        self.idle_start = utime.ticks_ms()
        self.stats["sleep_count"] += 1

        # TODO: Implement actual sleep mode
        # - Disable unnecessary peripherals
        # - Configure wake sources (UART RX, buttons)
        # - Enter lightsleep or deepsleep

        print("PowerMonitor: Sleep mode")

    def exit_sleep_mode(self):
        """Exit sleep mode on activity"""
        if self.power_mode == POWER_MODE_SLEEP:
            idle_duration = 0
            if self.idle_start:
                idle_duration = utime.ticks_diff(utime.ticks_ms(), self.idle_start)
                self.idle_start = None

            self.power_mode = POWER_MODE_NORMAL
            self.update_activity()
            print(f"PowerMonitor: Wake from sleep (idle {idle_duration}ms)")

    def enter_shutdown_mode(self):
        """Enter shutdown mode (critical battery)"""
        self.power_mode = POWER_MODE_SHUTDOWN
        print("PowerMonitor: Shutdown mode - critical battery")

        # TODO: Save state to flash
        # TODO: Notify host of shutdown
        # TODO: Enter deep sleep or power off

        # For now, just reset
        utime.sleep_ms(1000)
        machine.reset()

    def update_activity(self):
        """Update last activity timestamp"""
        self.last_activity = utime.ticks_ms()
        # Wake from sleep if needed
        if self.power_mode == POWER_MODE_SLEEP:
            self.exit_sleep_mode()

    def get_battery_stats(self):
        """Get current battery statistics"""
        if (
            self.battery
            and hasattr(self.battery, "available")
            and self.battery.available
        ):
            return self.battery.get_stats()
        return {
            "soc": self.last_soc,
            "voltage": self.last_voltage,
            "charging": self.last_charging,
            "available": False,
        }

    def get_power_stats(self):
        """Get power management statistics"""
        stats = self.stats.copy()
        stats["mode"] = self.power_mode
        stats["battery"] = self.get_battery_stats()
        return stats

    def force_battery_update(self):
        """Force immediate battery status update"""
        self.update_battery_status()


class PowerSimulator:
    """Power monitor simulator for testing"""

    def __init__(self):
        self.battery = BatterySimulator()
        self.power_mode = POWER_MODE_NORMAL
        self.on_battery_update = None
        self.on_low_battery = None
        self.on_critical_battery = None
        self.stats = {
            "updates": 0,
            "low_battery_events": 0,
            "critical_battery_events": 0,
            "sleep_count": 0,
        }

    def start(self):
        print("PowerSimulator: Started")

    def stop(self):
        print("PowerSimulator: Stopped")

    def set_power_mode(self, mode):
        self.power_mode = mode
        print(f"PowerSimulator: Mode set to {mode}")

    def update_activity(self):
        pass

    def get_battery_stats(self):
        return self.battery.get_stats()

    def get_power_stats(self):
        stats = self.stats.copy()
        stats["mode"] = self.power_mode
        stats["battery"] = self.get_battery_stats()
        return stats

    def simulate_low_battery(self):
        """Simulate low battery condition"""
        self.battery.soc = 10
        self.battery.voltage = 3.2
        self.stats["low_battery_events"] += 1
        if self.on_low_battery:
            self.on_low_battery(10)

    def simulate_critical_battery(self):
        """Simulate critical battery condition"""
        self.battery.soc = 3
        self.battery.voltage = 3.0
        self.stats["critical_battery_events"] += 1
        if self.on_critical_battery:
            self.on_critical_battery(3)
