import machine
import time

class MP2760:
    """MP2760 Battery Management System class for MPS's MP2760 battery charger IC."""
    
    # Known register values for validation
    DEVICE_ID_REG = 0x05  # Device ID register
    EXPECTED_DEVICE_ID = 0x5C  # Expected value for MP2760
    
    # Important registers
    REG07H = 0x07  # System voltage configuration
    REG08H = 0x08  # Input current limit configuration
    REG0AH = 0x0A  # Output current configuration
    REG12H = 0x12  # Mode control register (charging/source mode)
    
    def __init__(self, i2c=None, address=0x5C):  # Default I2C address for MP2760
        """Initialize the MP2760 with the specified I2C bus and address."""
        self.i2c = i2c or machine.I2C(0, sda=machine.Pin(24), scl=machine.Pin(25), freq=100000)
        self.addr = address
        
    # ---------- low-level I2C helpers ----------
    def _rd(self, reg, n=1):
        """Read n bytes from register reg."""
        return self.i2c.readfrom_mem(self.addr, reg, n)
    
    def _wr(self, reg, data):
        """Write data to register reg."""
        self.i2c.writeto_mem(self.addr, reg, data)
    
    def _rd_byte(self, reg):
        """Read a single byte from register reg."""
        return self._rd(reg, 1)[0]
    
    def _wr_byte(self, reg, val):
        """Write a single byte to register reg."""
        self._wr(reg, bytes([val]))
    
    def _rd_word(self, reg):
        """Read a 16-bit word from register reg."""
        data = self._rd(reg, 2)
        return (data[1] << 8) | data[0]  # Little-endian format
    
    def _wr_word(self, reg, val):
        """Write a 16-bit word to register reg."""
        self._wr(reg, bytes([val & 0xFF, (val >> 8) & 0xFF]))  # Little-endian format
    
    # ---------- Validation functions ----------
    def detect(self):
        """Verify that the MP2760 is connected and responding."""
        try:
            device_id = self._rd_byte(self.DEVICE_ID_REG)
            if device_id == self.EXPECTED_DEVICE_ID:
                print(f"MP2760 detected! Device ID: 0x{device_id:02X}")
                return True
            else:
                print(f"Unexpected device ID: 0x{device_id:02X}, expected: 0x{self.EXPECTED_DEVICE_ID:02X}")
                return False
        except Exception as e:
            print(f"Error detecting MP2760: {e}")
            return False
    
    def scan_i2c_bus(self):
        """Scan the I2C bus for all connected devices."""
        devices = self.i2c.scan()
        if devices:
            print(f"Found {len(devices)} I2C devices:")
            for device in devices:
                print(f"  Device at address: 0x{device:02X}")
            return devices
        else:
            print("No I2C devices found")
            return []
    
    def read_all_registers(self, start_reg=0, end_reg=0x20):
        """Read and print all registers in the specified range."""
        print("Register dump:")
        for reg in range(start_reg, end_reg + 1):
            try:
                value = self._rd_byte(reg)
                print(f"  REG{reg:02X}h: 0x{value:02X}")
            except Exception as e:
                print(f"  REG{reg:02X}h: Error: {e}")
    
    # ---------- Basic configuration functions ----------
    def get_system_voltage(self):
        """Read the system voltage configuration."""
        reg_value = self._rd_byte(self.REG07H)
        vsys_min = (reg_value & 0x7F) * 0.2  # 7 bits, 200mV per bit
        return vsys_min
    
    def get_input_current_limit(self):
        """Read the input current limit configuration."""
        reg_value = self._rd_byte(self.REG08H)
        current_limit = (reg_value & 0x7F) * 50  # 7 bits, 50mA per bit
        return current_limit
    
    def disable_charging(self):
        """Disable charging functionality."""
        reg_value = self._rd_byte(self.REG12H)
        # Clear bit 0 (CHG_EN) to disable charging
        reg_value &= ~0x01
        self._wr_byte(self.REG12H, reg_value)
        print("Charging disabled")
    
    def enable_charging(self):
        """Enable charging functionality."""
        reg_value = self._rd_byte(self.REG12H)
        # Set bit 0 (CHG_EN) to enable charging
        reg_value |= 0x01
        self._wr_byte(self.REG12H, reg_value)
        print("Charging enabled")
    
    def get_charging_status(self):
        """Read the charging status."""
        reg_value = self._rd_byte(self.REG12H)
        charging_enabled = bool(reg_value & 0x01)
        source_mode_enabled = bool(reg_value & 0x08)
        return {
            "charging_enabled": charging_enabled,
            "source_mode_enabled": source_mode_enabled
        }


# Example usage:
"""
import machine
import time
from mp2760 import MP2760

# Initialize the MP2760
mp2760 = MP2760()

# Scan the I2C bus for devices
devices = mp2760.scan_i2c_bus()

# Check if the MP2760 is detected
if mp2760.detect():
    print("MP2760 validation successful!")
    
    # Read all registers for debugging
    mp2760.read_all_registers()
    
    # Get current configuration
    system_voltage = mp2760.get_system_voltage()
    input_current = mp2760.get_input_current_limit()
    charging_status = mp2760.get_charging_status()
    
    print(f"System voltage: {system_voltage}V")
    print(f"Input current limit: {input_current}mA")
    print(f"Charging enabled: {charging_status['charging_enabled']}")
    print(f"Source mode enabled: {charging_status['source_mode_enabled']}")
    
    # Disable charging if needed
    # mp2760.disable_charging()
else:
    print("MP2760 validation failed!")
"""