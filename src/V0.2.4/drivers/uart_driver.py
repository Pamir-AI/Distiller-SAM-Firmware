"""
UART Driver with Interrupt Support
Handles SAM protocol UART communication
"""

import machine
import uasyncio
from utils.circular_buffer import CircularBuffer
import config


class UARTDriver:
    """Interrupt-driven UART driver for SAM protocol"""

    def __init__(self, uart_id=0, baudrate=115200, tx_pin=0, rx_pin=1):
        # Configure UART
        self.uart = machine.UART(
            uart_id,
            baudrate=baudrate,
            tx=machine.Pin(tx_pin),
            rx=machine.Pin(rx_pin),
            bits=8,
            parity=None,
            stop=1,
            timeout=0,
            rxbuf=config.UART_FIFO_SIZE,
        )

        # Buffers
        self.rx_buffer = CircularBuffer(config.RX_BUFFER_SIZE)
        self.tx_buffer = CircularBuffer(config.TX_BUFFER_SIZE)

        # Callbacks
        self.on_data_received = None

        # Statistics
        self.stats = {
            "bytes_sent": 0,
            "bytes_received": 0,
            "rx_overflows": 0,
            "tx_overflows": 0,
        }

        # Task control
        self.running = False
        self.rx_task_handle = None

    def start(self):
        """Start UART driver"""
        if not self.running:
            self.running = True
            self.rx_task_handle = uasyncio.create_task(self.rx_task())
            print(
                f"UART: Started on pins TX={config.UART_TX_PIN}, RX={config.UART_RX_PIN}"
            )

    def stop(self):
        """Stop UART driver"""
        self.running = False
        if self.rx_task_handle:
            self.rx_task_handle.cancel()
            self.rx_task_handle = None
        print("UART: Stopped")

    async def rx_task(self):
        """RX task - polls UART for incoming data"""
        while self.running:
            try:
                # Check for available data
                if self.uart.any():
                    # Read available bytes
                    data = self.uart.read(min(self.uart.any(), 64))
                    if data:
                        # Add to RX buffer
                        added = self.rx_buffer.put(data)
                        self.stats["bytes_received"] += added
                        if added < len(data):
                            self.stats["rx_overflows"] += 1

                        # Notify callback
                        if self.on_data_received:
                            self.on_data_received(data)

                # Cooperative yield
                await uasyncio.sleep_ms(1)

            except Exception as e:
                print(f"UART RX error: {e}")

    def write(self, data):
        """Write data to UART"""
        if not self.running:
            return 0

        try:
            # Try to send immediately if possible
            if not self.tx_buffer.available() and self.uart.txdone():
                written = self.uart.write(data)
                self.stats["bytes_sent"] += written
                if written < len(data):
                    # Buffer remaining data
                    remaining = data[written:]
                    buffered = self.tx_buffer.put(remaining)
                    if buffered < len(remaining):
                        self.stats["tx_overflows"] += 1
                    written += buffered
                return written
            else:
                # Buffer all data
                buffered = self.tx_buffer.put(data)
                if buffered < len(data):
                    self.stats["tx_overflows"] += 1
                return buffered
        except Exception as e:
            print(f"UART write error: {e}")
            return 0

    async def process_tx_buffer(self):
        """Process pending TX data"""
        if not self.running:
            return

        if self.tx_buffer.available() > 0 and self.uart.txdone():
            # Send buffered data
            chunk_size = min(64, self.tx_buffer.available())
            data = self.tx_buffer.get(chunk_size)
            if data:
                written = self.uart.write(data)
                self.stats["bytes_sent"] += written

    def read(self, max_bytes=None):
        """Read data from RX buffer"""
        return self.rx_buffer.get(max_bytes)

    def available(self):
        """Get number of bytes available to read"""
        return self.rx_buffer.available()

    def flush_rx(self):
        """Flush RX buffer"""
        self.rx_buffer.clear()

    async def flush_tx(self):
        """Flush TX buffer and wait for transmission"""
        self.tx_buffer.clear()
        # Wait for UART to finish
        timeout = 100  # 100ms timeout
        while not self.uart.txdone() and timeout > 0:
            await uasyncio.sleep_ms(1)
            timeout -= 1

    def get_stats(self):
        """Get UART statistics"""
        stats = self.stats.copy()
        stats["rx_buffer_used"] = self.rx_buffer.available()
        stats["tx_buffer_used"] = self.tx_buffer.available()
        stats["rx_buffer_size"] = self.rx_buffer.size
        stats["tx_buffer_size"] = self.tx_buffer.size
        return stats

    def reset_stats(self):
        """Reset statistics"""
        self.stats = {
            "bytes_sent": 0,
            "bytes_received": 0,
            "rx_overflows": 0,
            "tx_overflows": 0,
        }


class LoopbackUART:
    """Loopback UART for testing"""

    def __init__(self):
        self.rx_buffer = CircularBuffer(1024)
        self.on_data_received = None
        self.running = False

    def start(self):
        self.running = True
        print("LoopbackUART: Started")

    def stop(self):
        self.running = False
        print("LoopbackUART: Stopped")

    def write(self, data):
        """Write data - loops back to RX"""
        if self.running:
            self.rx_buffer.put(data)
            if self.on_data_received:
                self.on_data_received(data)
            return len(data)
        return 0

    def read(self, max_bytes=None):
        """Read looped back data"""
        return self.rx_buffer.get(max_bytes)

    def available(self):
        """Get bytes available"""
        return self.rx_buffer.available()

    def flush_rx(self):
        """Flush RX buffer"""
        self.rx_buffer.clear()

    def flush_tx(self):
        """No-op for loopback"""
        pass

    async def process_tx_buffer(self):
        """No-op for loopback"""
        pass

    def get_stats(self):
        """Get statistics"""
        return {
            "bytes_sent": 0,
            "bytes_received": 0,
            "rx_overflows": 0,
            "tx_overflows": 0,
            "rx_buffer_used": self.rx_buffer.available(),
        }
