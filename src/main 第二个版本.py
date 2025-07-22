import time
from power_manager import PowerManager

EXPECTED_DESIGN_CAP = 3000
EXPECTED_TERM_VOLT = 3200
DO_DEEP_VERIFY = True

def deep_verify_state(pm):
    bq = pm.bq27441
    print("\n[DEEP] Enter CFGUPDATE to read State subclass...")
    bq._unseal()
    bq._enter_cfgupdate()

    # Select State subclass (0x52), block 0
    bq._wr(0x61, b"\x00")
    bq._wr(0x3E, b"\x52")
    bq._wr(0x3F, b"\x00")
    block = bytearray(bq._rd(0x40, 32))

    # Dump block (避免嵌套 f-string)
    hex_list = []
    for b in block:
        hex_list.append("{:02X}".format(b))
    print("[DEEP] Raw State block (0x52,0): " + " ".join(hex_list))
    design_cap_be = (block[0x09] << 8) | block[0x0A]
    term_volt_be  = (block[0x0f] << 8) | block[0x10]

    print("[DEEP] DesignCapacity(offset0x0A) = {} mAh {}".format(
        design_cap_be,
        "OK" if design_cap_be == EXPECTED_DESIGN_CAP else "MISMATCH"
    ))
    print("[DEEP] TerminateVoltage(offset0x10) = {} mV {}".format(
        term_volt_be,
        "OK" if term_volt_be == EXPECTED_TERM_VOLT else "MISMATCH"
    ))

    bq._exit_cfgupdate()
    bq.control(0x000C)  # BAT_INSERT

def main():
    print("=== Power / Fuel Gauge Quick Test ===")
    pm = PowerManager(design_capacity_mah=EXPECTED_DESIGN_CAP, debug_enabled=True)
    if pm.bq27441 is None:
        print("ERROR: BQ27441 init failed.")
        return

    time.sleep(0.5)
    bq = pm.bq27441

    def rd(reg):
        return bq._rd_word(reg)

    design_cap = rd(0x3C)
    remain_cap = rd(0x0C)
    full_cap   = rd(0x0E)
    soc        = rd(0x1C)
    voltage    = rd(0x04)
    temp_01K   = rd(0x02)
    avg_curr   = bq.avg_current_mA()
    flags      = rd(0x06)
    #ctrl_stat  = bq.control_status()
    temp_C     = temp_01K * 0.1 - 273.15

    print("\n--- Standard Readouts ---")
    print("DesignCapacity(): {} mAh {}".format(
        design_cap, "OK" if design_cap == EXPECTED_DESIGN_CAP else "MISMATCH"))
    print("RemainingCapacity(): {} mAh".format(remain_cap))
    print("FullChargeCapacity(): {} mAh".format(full_cap))
    print("SOC(): {} %".format(soc))
    print("Voltage(): {} mV".format(voltage))
    print("Temperature(): {:.2f} °C".format(temp_C))
    print("AvgCurrent(): {} mA".format(avg_curr))
    print("Flags(): 0x{:04X}".format(flags))
    #print("CONTROL_STATUS: 0x{:04X}".format(ctrl_stat))

    if DO_DEEP_VERIFY:
        deep_verify_state(pm)

if __name__ == "__main__":
    main()
