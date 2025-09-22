"""
SAM Protocol Layer 2 - Data Link Layer
HDLC-inspired framing with byte stuffing and CRC-16-CCITT
"""

from protocol.sam_defs import *
from utils.crc16_ccitt import CRC16CCITT
from utils.circular_buffer import CircularBuffer
import config


class FrameEncoder:
    """Encode frames with byte stuffing and CRC"""

    def __init__(self):
        self.crc = CRC16CCITT()

    def encode_frame(self, addr, ctrl, payload):
        """Encode a complete frame with CRC and byte stuffing"""
        # Build raw frame (without flags and stuffing)
        frame = bytearray()
        frame.append(addr)
        frame.append(ctrl)
        frame.append(len(payload))
        frame.extend(payload)

        # Calculate CRC over raw frame
        self.crc.reset()
        self.crc.update(frame)
        crc_h, crc_l = self.crc.get_bytes()

        # Build complete frame with byte stuffing
        stuffed = bytearray()
        stuffed.append(FLAG)

        # Stuff frame bytes
        for byte in frame:
            if byte == FLAG:
                stuffed.append(ESC)
                stuffed.append(FLAG_ESC)
            elif byte == ESC:
                stuffed.append(ESC)
                stuffed.append(ESC_ESC)
            else:
                stuffed.append(byte)

        # Stuff CRC bytes
        if crc_h == FLAG:
            stuffed.append(ESC)
            stuffed.append(FLAG_ESC)
        elif crc_h == ESC:
            stuffed.append(ESC)
            stuffed.append(ESC_ESC)
        else:
            stuffed.append(crc_h)

        if crc_l == FLAG:
            stuffed.append(ESC)
            stuffed.append(FLAG_ESC)
        elif crc_l == ESC:
            stuffed.append(ESC)
            stuffed.append(ESC_ESC)
        else:
            stuffed.append(crc_l)

        # End flag
        stuffed.append(FLAG)

        return bytes(stuffed)


class FrameDecoder:
    """Decode frames with byte unstuffing and CRC verification"""

    def __init__(self, on_frame_received):
        self.on_frame_received = on_frame_received
        self.state = FRAME_STATE_IDLE
        self.frame_buffer = bytearray()
        self.escape_next = False
        self.crc = CRC16CCITT()
        self.stats = {
            "frames_received": 0,
            "crc_errors": 0,
            "framing_errors": 0,
            "overflow_errors": 0,
        }
        self.consecutive_errors = 0
        self.resync_mode = False

    def process_byte(self, byte):
        """Process a single received byte"""
        # Handle resync mode - look for FLAG to restart
        if self.resync_mode:
            if byte == FLAG:
                self.resync_mode = False
                self.reset_frame()
                self.state = FRAME_STATE_ADDR
            return

        # Check for FLAG at any time (frame delimiter)
        if not self.escape_next and byte == FLAG:
            if self.state == FRAME_STATE_IDLE:
                # Start of new frame
                self.reset_frame()
                self.state = FRAME_STATE_ADDR
            elif self.state == FRAME_STATE_END:
                # End of frame - validate and deliver
                self.validate_frame()
                self.state = FRAME_STATE_IDLE
            else:
                # Unexpected FLAG - restart frame
                self.stats["framing_errors"] += 1
                self.reset_frame()
                self.state = FRAME_STATE_ADDR
            return

        # Handle escape sequence
        if not self.escape_next and byte == ESC:
            self.escape_next = True
            return

        # Apply escape transformation if needed
        if self.escape_next:
            if byte == FLAG_ESC:
                byte = FLAG
            elif byte == ESC_ESC:
                byte = ESC
            else:
                # Invalid escape sequence
                self.stats["framing_errors"] += 1
                self.consecutive_errors += 1
                self.enter_resync_mode()
                return
            self.escape_next = False

        # Process byte based on current state
        if self.state == FRAME_STATE_ADDR:
            self.frame_buffer.append(byte)
            self.state = FRAME_STATE_CTRL

        elif self.state == FRAME_STATE_CTRL:
            self.frame_buffer.append(byte)
            self.state = FRAME_STATE_LEN

        elif self.state == FRAME_STATE_LEN:
            self.frame_buffer.append(byte)
            self.expected_len = byte
            if self.expected_len > 0:
                self.state = FRAME_STATE_DATA
                self.data_count = 0
            else:
                self.state = FRAME_STATE_CRC_H

        elif self.state == FRAME_STATE_DATA:
            if len(self.frame_buffer) >= config.MAX_FRAME_SIZE:
                self.stats["overflow_errors"] += 1
                self.enter_resync_mode()
                return
            self.frame_buffer.append(byte)
            self.data_count += 1
            if self.data_count >= self.expected_len:
                self.state = FRAME_STATE_CRC_H

        elif self.state == FRAME_STATE_CRC_H:
            self.crc_h = byte
            self.state = FRAME_STATE_CRC_L

        elif self.state == FRAME_STATE_CRC_L:
            self.crc_l = byte
            self.state = FRAME_STATE_END

    def reset_frame(self):
        """Reset frame decoder state"""
        self.frame_buffer = bytearray()
        self.escape_next = False
        self.data_count = 0
        self.expected_len = 0

    def validate_frame(self):
        """Validate completed frame"""
        if len(self.frame_buffer) < 3:
            self.stats["framing_errors"] += 1
            return

        # Calculate CRC over frame data
        self.crc.reset()
        self.crc.update(self.frame_buffer)
        calc_crc_h, calc_crc_l = self.crc.get_bytes()

        # Verify CRC
        if calc_crc_h != self.crc_h or calc_crc_l != self.crc_l:
            self.stats["crc_errors"] += 1
            self.consecutive_errors += 1
            if self.consecutive_errors > config.MAX_CONSECUTIVE_ERRORS:
                self.enter_resync_mode()
            return

        # Extract frame components
        addr = self.frame_buffer[0]
        ctrl = self.frame_buffer[1]
        length = self.frame_buffer[2]
        payload = bytes(self.frame_buffer[3 : 3 + length]) if length > 0 else bytes()

        # Deliver valid frame
        self.stats["frames_received"] += 1
        self.consecutive_errors = 0
        if self.on_frame_received:
            self.on_frame_received(addr, ctrl, payload)

    def enter_resync_mode(self):
        """Enter resynchronization mode after errors"""
        self.resync_mode = True
        self.reset_frame()
        self.state = FRAME_STATE_IDLE
        print("L2: Entering resync mode")

    def get_stats(self):
        """Get decoder statistics"""
        return self.stats.copy()


class Layer2:
    """SAM Protocol Layer 2 - Data Link Layer"""

    def __init__(self, on_frame_received=None):
        self.encoder = FrameEncoder()
        self.decoder = FrameDecoder(on_frame_received)
        self.rx_buffer = CircularBuffer(config.RX_BUFFER_SIZE)
        self.tx_buffer = CircularBuffer(config.TX_BUFFER_SIZE)

    def encode_frame(self, addr, ctrl, payload):
        """Encode a frame for transmission"""
        return self.encoder.encode_frame(addr, ctrl, payload)

    def process_rx_data(self, data):
        """Process received data bytes"""
        if isinstance(data, int):
            self.decoder.process_byte(data)
        else:
            for byte in data:
                self.decoder.process_byte(byte)

    def get_stats(self):
        """Get layer 2 statistics"""
        return {
            "decoder": self.decoder.get_stats(),
            "rx_buffer": self.rx_buffer.get_stats(),
            "tx_buffer": self.tx_buffer.get_stats(),
        }
