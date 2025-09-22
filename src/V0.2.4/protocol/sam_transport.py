"""
SAM Protocol Layer 2.5 - Transport Layer
Sliding window protocol with flow control and retransmission
"""

import utime
import uasyncio
from protocol.sam_defs import *
from protocol.sam_l2 import Layer2
from utils.timer_manager import ProtocolTimers
from utils.circular_buffer import MessageBuffer
import config


class SlidingWindow:
    """Sliding window for reliable transmission"""

    def __init__(self, window_size=4):
        self.window_size = window_size
        self.tx_seq = 0  # Next sequence number to send
        self.tx_ack = 0  # Last acknowledged sequence
        self.rx_seq = 0  # Next expected sequence number
        self.rx_ack = 0  # Last sequence to acknowledge

        # Transmit window
        self.tx_window = {}  # seq_num -> (frame, timestamp, retries)

    def get_next_tx_seq(self):
        """Get next transmit sequence number if window allows"""
        # Check if window is full
        window_used = (self.tx_seq - self.tx_ack) & 0x0F
        if window_used >= self.window_size:
            return None
        seq = self.tx_seq
        self.tx_seq = (self.tx_seq + 1) & 0x0F
        return seq

    def add_to_tx_window(self, seq_num, frame):
        """Add frame to transmit window"""
        self.tx_window[seq_num] = (frame, utime.ticks_ms(), 0)

    def remove_from_tx_window(self, seq_num):
        """Remove acknowledged frame from window"""
        if seq_num in self.tx_window:
            del self.tx_window[seq_num]

    def get_frame_for_retransmit(self, seq_num):
        """Get frame for retransmission"""
        if seq_num in self.tx_window:
            frame, _, retries = self.tx_window[seq_num]
            self.tx_window[seq_num] = (frame, utime.ticks_ms(), retries + 1)
            return frame, retries + 1
        return None, 0

    def process_ack(self, ack_num):
        """Process ACK, removing acknowledged frames"""
        # ACK(n) acknowledges all frames up to n-1
        while self.tx_ack != ack_num:
            if self.tx_ack in self.tx_window:
                del self.tx_window[self.tx_ack]
            self.tx_ack = (self.tx_ack + 1) & 0x0F

    def is_valid_rx_seq(self, seq_num):
        """Check if received sequence number is valid"""
        # Check if in receive window
        distance = (seq_num - self.rx_seq) & 0x0F
        return distance < self.window_size

    def update_rx_seq(self, seq_num):
        """Update receive sequence for in-order frame"""
        if seq_num == self.rx_seq:
            self.rx_seq = (self.rx_seq + 1) & 0x0F
            self.rx_ack = self.rx_seq
            return True
        return False

    def get_pending_count(self):
        """Get number of unacknowledged frames"""
        return len(self.tx_window)

    def reset(self):
        """Reset window state"""
        self.tx_seq = 0
        self.tx_ack = 0
        self.rx_seq = 0
        self.rx_ack = 0
        self.tx_window.clear()


class TransportLayer:
    """SAM Protocol Transport Layer"""

    def __init__(self, uart_driver=None):
        self.uart = uart_driver
        self.state = STATE_DISCONNECTED
        self.window = SlidingWindow(config.DEFAULT_WINDOW_SIZE)

        # Layer 2
        self.l2 = Layer2(self.on_frame_received)

        # Timers
        self.timers = ProtocolTimers(
            config.T1_ACK_TIMEOUT, config.T2_IDLE_TIMEOUT, config.T3_KEEPALIVE
        )
        self.timers.on_ack_timeout = self.handle_ack_timeout
        self.timers.on_idle_timeout = self.handle_idle_timeout
        self.timers.on_keepalive = self.handle_keepalive

        # Message buffers
        self.tx_queue = MessageBuffer(config.MAX_PENDING_MESSAGES)
        self.rx_queue = MessageBuffer(config.MAX_PENDING_MESSAGES)

        # Callbacks
        self.on_connected = None
        self.on_disconnected = None
        self.on_message_received = None

        # Statistics
        self.stats = {
            "frames_sent": 0,
            "frames_received": 0,
            "acks_sent": 0,
            "acks_received": 0,
            "naks_sent": 0,
            "naks_received": 0,
            "retransmissions": 0,
            "timeouts": 0,
        }

        # SYNC negotiation
        self.sync_attempts = 0
        self.remote_window_size = config.DEFAULT_WINDOW_SIZE

    def start(self):
        """Start transport layer"""
        self.state = STATE_SYNCING
        self.send_sync()
        self.timers.create_timer(
            "sync_retry",
            config.SYNC_RETRY_INTERVAL,
            self.handle_sync_timeout,
            periodic=True,
        )
        self.timers.start_timer("sync_retry")

    def stop(self):
        """Stop transport layer"""
        self.timers.stop_all()
        self.state = STATE_DISCONNECTED
        self.window.reset()
        if self.on_disconnected:
            self.on_disconnected()

    def send_sync(self):
        """Send SYNC frame to establish connection"""
        # SYNC payload per spec: VERSION_MAJOR, VERSION_MINOR, WINDOW_SIZE, MAX_PAYLOAD, RX_BUFFER_SIZE(2), FLAGS(2)
        import struct

        payload = struct.pack(
            ">BBBBHH",
            0x02,  # VERSION_MAJOR (v2.0.0)
            0x00,  # VERSION_MINOR
            config.DEFAULT_WINDOW_SIZE,
            249,  # MAX_PAYLOAD
            getattr(config, "RX_BUFFER_SIZE", 2048),  # RX buffer size
            0x0000,  # FLAGS
        )
        ctrl = TYPE_SYNC  # No sequence number for SYNC
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, payload)
        self.send_frame_raw(frame)
        self.sync_attempts += 1

    def send_sync_ack(self):
        """Send SYNC_ACK to confirm connection"""
        # SYNC_ACK uses same format as SYNC
        import struct

        payload = struct.pack(
            ">BBBBHH",
            0x02,  # VERSION_MAJOR
            0x00,  # VERSION_MINOR
            self.window.window_size,
            249,  # MAX_PAYLOAD
            getattr(config, "RX_BUFFER_SIZE", 2048),
            0x0000,  # FLAGS
        )
        ctrl = TYPE_SYNC  # SYNC_ACK uses same type as SYNC
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, payload)
        self.send_frame_raw(frame)

    def send_message(self, message, priority=0):
        """Queue message for transmission"""
        if self.state != STATE_CONNECTED:
            return False
        return self.tx_queue.put(message, priority)

    async def process_tx_queue(self):
        """Process pending transmit messages"""
        if self.state != STATE_CONNECTED:
            return

        while self.tx_queue.available() > 0:
            # Check if window has space
            seq = self.window.get_next_tx_seq()
            if seq is None:
                break  # Window full

            # Get next message
            message = self.tx_queue.get()
            if message is None:
                break

            # Build info frame - sequence in bits 7-4, type in bits 3-0
            ctrl = (seq << 4) | TYPE_INFO
            frame = self.l2.encode_frame(ADDR_SAM, ctrl, message)

            # Add to window and send
            self.window.add_to_tx_window(seq, frame)
            self.send_frame_raw(frame)
            self.stats["frames_sent"] += 1

            # Start ACK timer
            self.timers.start_ack_timer(seq)

    def on_frame_received(self, addr, ctrl, payload):
        """Handle received frame from Layer 2"""
        self.timers.reset_idle_timer()

        frame_type = ctrl & TYPE_MASK
        seq_num = (ctrl & CTRL_SEQ_MASK) >> 4

        if frame_type == TYPE_INFO:
            # Information frame
            self.handle_info_frame(seq_num, payload)
        elif frame_type == TYPE_ACK:
            # ACK frame
            self.handle_ack(seq_num)
        elif frame_type == TYPE_NAK:
            # NAK frame
            self.handle_nak(seq_num)
        elif frame_type == TYPE_SYNC:
            # SYNC frame (or SYNC_ACK - same type)
            self.handle_sync(payload)
        elif frame_type == TYPE_PING:
            # Ping - send pong
            self.send_pong()
        elif frame_type == TYPE_PONG:
            # Pong received
            pass
        elif frame_type == TYPE_RESET:
            # Reset sequence numbers
            self.handle_reset()
        elif frame_type == TYPE_FLOW:
            # Flow control
            self.handle_flow(payload)

    def handle_info_frame(self, seq_num, payload):
        """Handle received information frame"""
        self.stats["frames_received"] += 1

        # Check sequence number
        if self.window.is_valid_rx_seq(seq_num):
            if self.window.update_rx_seq(seq_num):
                # In-order frame - deliver and ACK
                self.rx_queue.put(payload)
                if self.on_message_received:
                    self.on_message_received(payload)
                self.send_ack(self.window.rx_ack)
            else:
                # Out of order - send NAK for expected
                self.send_nak(self.window.rx_seq)
        else:
            # Outside window - ignore
            pass

    def handle_ack(self, ack_num):
        """Handle received ACK"""
        self.stats["acks_received"] += 1
        # Process acknowledgment
        old_ack = self.window.tx_ack
        self.window.process_ack(ack_num)
        # Stop timers for acknowledged frames
        seq = old_ack
        while seq != ack_num:
            self.timers.stop_ack_timer(seq)
            seq = (seq + 1) & 0x0F

    def handle_nak(self, nak_num):
        """Handle received NAK"""
        self.stats["naks_received"] += 1
        # Retransmit requested frame
        frame, retries = self.window.get_frame_for_retransmit(nak_num)
        if frame and retries <= config.MAX_RETRANSMISSIONS:
            self.send_frame_raw(frame)
            self.stats["retransmissions"] += 1
            self.timers.reset_timer(f"ack_{nak_num}")

    def handle_sync(self, payload):
        """Handle received SYNC"""
        import struct

        # SYNC payload: VERSION_MAJOR, VERSION_MINOR, WINDOW_SIZE, MAX_PAYLOAD, RX_BUFFER_SIZE(2), FLAGS(2)
        if len(payload) >= 8:
            major, minor, window_size, max_payload, rx_buffer_size, flags = (
                struct.unpack(">BBBBHH", payload[:8])
            )

            # Validate parameters
            if major == 2 and 1 <= window_size <= config.MAX_WINDOW_SIZE:
                self.remote_window_size = window_size
                self.window.window_size = min(window_size, config.DEFAULT_WINDOW_SIZE)
                # Check if this is initial SYNC or SYNC_ACK response
                if self.state == STATE_SYNCING:
                    # This is SYNC_ACK response to our SYNC
                    self.handle_sync_ack(payload)
                else:
                    # This is initial SYNC from host
                    self.send_sync_ack()
                    self.enter_connected_state()

    def handle_sync_ack(self, payload):
        """Handle received SYNC_ACK (same as SYNC in v2.0.0)"""
        import struct

        if self.state == STATE_SYNCING and len(payload) >= 8:
            major, minor, window_size, max_payload, rx_buffer_size, flags = (
                struct.unpack(">BBBBHH", payload[:8])
            )

            if major == 2 and 1 <= window_size <= config.MAX_WINDOW_SIZE:
                self.timers.stop_timer("sync_retry")
                self.timers.remove_timer("sync_retry")
                self.remote_window_size = window_size
                self.window.window_size = min(window_size, config.DEFAULT_WINDOW_SIZE)
                self.enter_connected_state()

    def handle_sync_timeout(self):
        """Handle SYNC timeout"""
        if self.state == STATE_SYNCING:
            if self.sync_attempts < config.MAX_SYNC_ATTEMPTS:
                self.send_sync()
            else:
                self.state = STATE_ERROR
                self.sync_attempts = 0

    def handle_ack_timeout(self, seq_num):
        """Handle ACK timeout for a frame"""
        self.stats["timeouts"] += 1
        frame, retries = self.window.get_frame_for_retransmit(seq_num)
        if frame:
            if retries <= config.MAX_RETRANSMISSIONS:
                self.send_frame_raw(frame)
                self.stats["retransmissions"] += 1
                self.timers.start_ack_timer(seq_num)
            else:
                # Max retries exceeded - reset connection
                self.reset_connection()

    def handle_idle_timeout(self):
        """Handle idle timeout"""
        if self.state == STATE_CONNECTED:
            self.send_heartbeat()

    def handle_keepalive(self):
        """Send keepalive message"""
        if self.state == STATE_CONNECTED:
            self.send_heartbeat()

    def send_ack(self, ack_num):
        """Send ACK frame"""
        ctrl = (ack_num << 4) | TYPE_ACK
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, bytes())
        self.send_frame_raw(frame)
        self.stats["acks_sent"] += 1

    def send_nak(self, nak_num):
        """Send NAK frame"""
        ctrl = (nak_num << 4) | TYPE_NAK
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, bytes())
        self.send_frame_raw(frame)
        self.stats["naks_sent"] += 1

    def send_heartbeat(self):
        """Send heartbeat using PING frame"""
        ctrl = TYPE_PING  # No sequence for PING
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, bytes())
        self.send_frame_raw(frame)

    def send_pong(self):
        """Send PONG in response to PING"""
        ctrl = TYPE_PONG
        frame = self.l2.encode_frame(ADDR_SAM, ctrl, bytes())
        self.send_frame_raw(frame)

    def handle_reset(self):
        """Handle RESET frame - reset sequence numbers"""
        self.window.reset()
        # Send ACK with seq=0 to confirm reset
        self.send_ack(0)

    def handle_flow(self, payload):
        """Handle FLOW control frame"""
        if len(payload) >= 1:
            credits = payload[0]
            # Update transmit credits
            self.tx_credits = credits

    def send_frame_raw(self, frame):
        """Send raw frame via UART"""
        if self.uart:
            self.uart.write(frame)

    def enter_connected_state(self):
        """Enter connected state"""
        self.state = STATE_CONNECTED
        self.sync_attempts = 0
        self.window.reset()
        self.timers.start_idle_timer()
        self.timers.start_keepalive_timer()
        if self.on_connected:
            self.on_connected()
        print("Transport: Connected")

    def reset_connection(self):
        """Reset connection after errors"""
        print("Transport: Resetting connection")
        self.state = STATE_SYNCING
        self.window.reset()
        self.sync_attempts = 0
        self.send_sync()

    def process_rx_data(self, data):
        """Process received UART data"""
        self.l2.process_rx_data(data)

    async def process_task(self):
        """Main processing task"""
        while True:
            # Process timers
            self.timers.process_timers()
            # Process transmit queue
            await self.process_tx_queue()
            # Cooperative yield
            await uasyncio.sleep_ms(1)

    def get_stats(self):
        """Get transport statistics"""
        stats = self.stats.copy()
        stats["state"] = self.state
        stats["window_size"] = self.window.window_size
        stats["pending_tx"] = self.window.get_pending_count()
        stats["l2"] = self.l2.get_stats()
        return stats
