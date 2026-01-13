#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版 RP2040 烧录脚本 for Window
Rongbo Hu 7 22
1.运行本脚本（注意：记得在本文件夹中放一个ULP文件夹）
2.按住BOOT键插入RP2040
3.等待脚本自动烧录
"""

import os
import time
import shutil
import subprocess

UPLOAD_FILES = [
    "bin/loading1.bin",
    "bin/loading2.bin",
    "eink_driver_sam.py",
    "pamir_uart_protocols.py",
    "neopixel_controller.py",
    "power_manager.py",
    "battery.py",
    "debug_handler.py",
    "uart_handler.py",
    "threaded_task_manager.py",
    "main.py"
]

def find_rp2040_drive():
    """查找包含 INFO_UF2.txt 的盘符"""
    if os.name == 'nt':
        drives = [f"{chr(i)}:\\" for i in range(65, 91) if os.path.exists(f"{chr(i)}:\\")]
    else:
        drives = [d.path for d in os.scandir('/mnt') if d.is_dir()]
    for drive in drives:
        if os.path.exists(os.path.join(drive, "INFO_UF2.txt")):
            return drive
    return None

def wait_for_rp2040(message, timeout=30):
    """等待RP2040插入"""
    print(message)
    start_time = time.time()
    while time.time() - start_time < timeout:
        drive = find_rp2040_drive()
        if drive:
            print(f"[OK] RP2040已连接：{drive}")
            return drive
        time.sleep(1)
    print("[ERROR] 超时未检测到RP2040")
    return None

def copy_uf2(drive, uf2_path):
    """复制UF2文件到RP2040"""
    try:
        shutil.copy(uf2_path, os.path.join(drive, os.path.basename(uf2_path)))
        print(f"[OK] 成功复制 {os.path.basename(uf2_path)} 到 {drive}")
        return True
    except Exception as e:
        print(f"[ERROR] UF2复制失败: {e}")
        return False

def wait_for_mpremote(port="COM162", timeout=60):
    """验证mpremote是否能连接到指定端口"""
    print(f"尝试通过 mpremote 连接到 {port} ...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            result = subprocess.run(["mpremote", "connect", port, "exec", "print('connected')"],
                                    capture_output=True, text=True, timeout=5)
            if "connected" in result.stdout:
                print(f"[OK] 成功连接到 {port}")
                return True
        except Exception as e:
            print(f"[WARNING] 连接失败: {e}")
        time.sleep(1)
    print(f"[ERROR] 无法连接到 {port}")
    return False


PORT = "COM162"  # 你从 Thonny 得到的端口号

def upload_files(port="COM162"):
    """通过mpremote上传文件到指定端口"""
    for f in UPLOAD_FILES:
        if not os.path.exists(f):
            print(f"[ERROR] 文件不存在: {f}")
            return False
        cmd = ["mpremote", "connect", port, "fs", "cp", f, f":/{os.path.basename(f)}"]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if result.returncode == 0:
                print(f"[OK] 上传成功: {os.path.basename(f)}")
            else:
                print(f"[ERROR] 上传失败: {result.stderr.strip()}")
                return False
        except Exception as e:
            print(f"[ERROR] 上传出错: {e}")
            return False
    return True


def main():
    print("========== RP2040 烧录工具 ==========")

    # 1. 等待第一次插入（BOOT模式）
    drive = wait_for_rp2040("请按住BOOT插入RP2040...", timeout=30)
    if not drive:
        return

    # 2. 烧录flash_nuke.uf2
    if not copy_uf2(drive, "ULP/flash_nuke.uf2"):
        return

    print("等待RP2040重新挂载...")
    time.sleep(5)

    # 3. 等待再次出现盘符
    drive = wait_for_rp2040("请等待RP2040重新连接...", timeout=30)
    if not drive:
        return

    # 4. 烧录RPI_PICO-20240222-v1.22.2.uf2
    if not copy_uf2(drive, "ULP/RPI_PICO-20240222-v1.22.2.uf2"):
        return

    print("等待RP2040进入Micropython模式...")
    time.sleep(8)

    # 5. 等待mpremote设备连接
    if not wait_for_mpremote(PORT):
        return

    # 6. 上传文件
    if not upload_files(PORT):
        return

    print("========== 烧录完成 ==========")

if __name__ == "__main__":
    main()
