"""Battery Management System for TI's BQ27441-G1A fuel gauge IC."""

# pylint: disable=import-error,invalid-name
import struct
import time
import machine


class BQ27441:
    """BQ27441 Battery Management System class for TI's BQ27441-G1A fuel gauge IC."""

    def __init__(self, i2c=None, address=0x55):
        self.i2c = i2c or machine.I2C(0, sda=machine.Pin(24), scl=machine.Pin(25))
        self.addr = address
        self.initialise()

    # ---------- low-level helpers ----------
    def _rd(self, reg, n=2):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _wr(self, reg, data):
        self.i2c.writeto_mem(self.addr, reg, data)

    def _rd_word(self, reg):
        return struct.unpack("<H", self._rd(reg, 2))[0]

    def _wr_word(self, reg, val):
        self._wr(reg, struct.pack("<H", val))

    # ---------- standard-command wrapper ----------
    def control(self, subcmd):
        """Send a standard command to the BQ27441."""
        self._wr_word(0x00, subcmd)  # send
        return self._rd_word(0x00)  # read back

    # ---------- sealing helpers ----------
    def _unseal(self):
        # default TI keys 0x8000 twice – change if you use a custom seal key
        self.control(0x8000)
        self.control(0x8000)

    # ---------- CONFIG-UPDATE helpers ----------
    def _enter_cfgupdate(self):
        self.control(0x0013)  # SET_CFGUPDATE
        while not self._rd_word(0x06) & 0x0010:  # Flags()[CFGUPMODE]
            time.sleep_ms(10)  # pylint: disable=no-member

    def _exit_cfgupdate(self):
        self.control(0x0042)  # SOFT_RESET; exits + resimulates

    def _extended_block_write(self, subclass_id, offset, payload):
        """Write <=32 bytes to one Extended-Data block, auto-checksum."""
        # 1 enable block access
        self._wr(0x61, b"\x00")  # BlockDataControl = 0
        self._wr(0x3E, bytes([subclass_id]))  # DataClass
        self._wr(0x3F, bytes([offset // 32]))  # Block offset 0-7

        # 2 read current 32-byte buffer
        buf = bytearray(self._rd(0x40, 32))
        # 3 modify
        buf[offset % 32 : offset % 32 + len(payload)] = payload
        # 4 write back whole buffer (only touched bytes actually needed)
        self._wr(0x40, buf)

        # 5 checksum
        csum = (0xFF - (sum(buf) & 0xFF)) & 0xFF
        self._wr(0x60, bytes([csum]))

    # ---------- public one-shot initialiser ----------
    def initialise(
        self,
        *,
        design_capacity_mAh=3000,
        terminate_voltage_mV=3200,
        CALIBRATION=True,
        golden_image=None
    ):
        """
        If CALIBRATION=True  → write DesignCap/TerminateV and clear BIE.
        If golden_image dict given → flash that instead (skips learning).
        Keys of golden_image must be {subclass_id : bytes-like-object}.
        """
        self._unseal()
        self._enter_cfgupdate()

        # --- path A – first-time board set-up ---------------------------
        if CALIBRATION:
            # 1 Design Capacity & Terminate Voltage (State 0x52, block 0)
            self._extended_block_write(
                0x52, 0x0A, struct.pack("<H", design_capacity_mAh)
            )
            self._extended_block_write(
                0x52, 0x10, struct.pack("<H", terminate_voltage_mV)
            )

            # 2 clear OpConfig BIE (Registers 0x40, byte 0x40)
            regs = bytearray(self._rd(0x40, 2))
            regs[0] &= 0x7F  # clear bit7
            self._extended_block_write(0x40, 0x00, regs)

        # --- path B – copy a golden image -------------------------------
        if golden_image:
            for subclass, blob in golden_image.items():
                for ofs in range(0, len(blob), 32):
                    self._extended_block_write(subclass, ofs, blob[ofs : ofs + 32])

        self._exit_cfgupdate()

        # host-side battery insertion kick-start
        self.control(0x000C)  # BAT_INSERT

    def remain_capacity(self):
        """Get the remaining capacity in mAh."""
        return self._rd_word(0x1C)

    def voltage_V(self):
        """Get the battery voltage in volts."""
        return self._rd_word(0x04) / 1000

    def temp_C(self):
        """Get the battery temperature in degrees Celsius."""
        return self._rd_word(0x02) * 0.1 - 273.15

    def avg_current_mA(self):
        """Get the average current in milliamperes."""
        raw = self._rd_word(0x10)
        return raw - 0x10000 if raw & 0x8000 else raw

    # Add additional methods here for other registers you need to interface with


# Usage example
# bms = BatteryManagementSystem()
# bms.get_control()
# bms.get_temperature()
# bms.get_voltage()
# bms.get_remaining_capacity()
