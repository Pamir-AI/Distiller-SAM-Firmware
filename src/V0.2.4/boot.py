"""
RP2040 SAM Protocol v2.0.0 Boot Configuration
Minimal boot setup for MicroPython on RP2040
"""

import gc
import machine

# Set CPU frequency to 125MHz for stable operation
machine.freq(125000000)

# Enable garbage collection
gc.enable()
gc.threshold(8192)

# Disable REPL on UART0 (needed for SAM protocol)
import os

os.dupterm(None)

print("SAM Protocol v2.0.0 Boot Complete")
