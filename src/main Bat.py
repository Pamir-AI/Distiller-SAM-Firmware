import time
from power_manager import PowerManager

EXPECTED_DESIGN_CAP = 3000
EXPECTED_TERM_VOLT = 3200
DO_DEEP_VERIFY = True
DO_POLL = True          # 新增：是否开启5秒轮询
POLL_INTERVAL_S = 5     # 新增：轮询间隔

def u16_be(buf, ofs):
    return (buf[ofs] << 8) | buf[ofs+1]

def deep_verify_state(pm):
    bq = pm.bq27441
    print("\n[DEEP] Enter CFGUPDATE to read State subclass...")
    bq._unseal()
    bq._enter_cfgupdate()

    # Select State subclass (0x52), block 0
    bq._wr(0x61, b"\x00")
    bq._wr(0x3E, b"\x52")
    bq._wr(0x3F, b"\x00")
    time.sleep_ms(2)  # 保守延时
    block = bytearray(bq._rd(0x40, 32))

    # Dump block
    print("[DEEP] Raw State block (0x52,0): " + " ".join(f"{b:02X}" for b in block))

    design_cap_be = u16_be(block, 0x09)
    term_volt_be  = u16_be(block, 0x0F)   # 注意这里是 0x0F/0x10

    print("[DEEP] DesignCapacity(offset0x0A) = {} mAh {}".format(
        design_cap_be, "OK" if design_cap_be == EXPECTED_DESIGN_CAP else "MISMATCH"))
    print("[DEEP] TerminateVoltage(offset0x10) = {} mV {}".format(
        term_volt_be, "OK" if term_volt_be == EXPECTED_TERM_VOLT else "MISMATCH"))

    bq._exit_cfgupdate()
    bq.control(0x000C)  # BAT_INSERT

def poll_every(bq, interval_s=5):
    """每 interval_s 秒打印一次关键参数，Ctrl-C 退出。"""
    try:
        while True:
            v_mv   = bq._rd_word(0x04)                      # mV
            soc    = bq._rd_word(0x1C)                      # %
            t_01k  = bq._rd_word(0x02)                      # 0.1K
            temp_c = t_01k * 0.1 - 273.15
            raw_i  = bq._rd_word(0x10)                      # signed
            curr_mA = raw_i - 0x10000 if raw_i & 0x8000 else raw_i

            # MicroPython 推荐用 ticks_ms 防止溢出问题
            ts = time.ticks_ms() / 1000
            print(f"[{ts:8.1f}s] V={v_mv}mV  I={curr_mA}mA  SOC={soc}%  T={temp_c:.2f}°C")
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\n[Poll] Stopped by user.")

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

    if DO_DEEP_VERIFY:
        deep_verify_state(pm)

    if DO_POLL:
        print(f"\n[Poll] Start polling every {POLL_INTERVAL_S}s. Press Ctrl-C to stop.")
        poll_every(bq, POLL_INTERVAL_S)

if __name__ == "__main__":
    main()
