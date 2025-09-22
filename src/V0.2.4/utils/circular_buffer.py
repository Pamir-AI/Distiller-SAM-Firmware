"""
Circular Buffer Implementation for UART and Message Queues
Ring buffer with overflow protection
"""


class CircularBuffer:
    """Circular buffer implementation"""

    def __init__(self, size):
        self.size = size
        self.buffer = bytearray(size)
        self.head = 0
        self.tail = 0
        self.count = 0
        self.overflow_count = 0

    def put(self, data):
        """Add data to buffer, returns number of bytes added"""
        if isinstance(data, int):
            # Single byte
            if self.count >= self.size:
                self.overflow_count += 1
                return 0
            self.buffer[self.head] = data
            self.head = (self.head + 1) % self.size
            self.count += 1
            return 1
        else:
            # Multiple bytes
            bytes_added = 0
            for byte in data:
                if self.count >= self.size:
                    self.overflow_count += len(data) - bytes_added
                    break
                self.buffer[self.head] = byte
                self.head = (self.head + 1) % self.size
                self.count += 1
                bytes_added += 1
            return bytes_added

    def get(self, max_bytes=None):
        """Get data from buffer, returns bytes"""
        if self.count == 0:
            return bytes()

        if max_bytes is None or max_bytes > self.count:
            max_bytes = self.count

        result = bytearray(max_bytes)
        for i in range(max_bytes):
            result[i] = self.buffer[self.tail]
            self.tail = (self.tail + 1) % self.size
            self.count -= 1

        return bytes(result)

    def peek(self, max_bytes=None):
        """Peek at data without removing it"""
        if self.count == 0:
            return bytes()

        if max_bytes is None or max_bytes > self.count:
            max_bytes = self.count

        result = bytearray(max_bytes)
        idx = self.tail
        for i in range(max_bytes):
            result[i] = self.buffer[idx]
            idx = (idx + 1) % self.size

        return bytes(result)

    def available(self):
        """Return number of bytes available to read"""
        return self.count

    def free_space(self):
        """Return number of bytes available for writing"""
        return self.size - self.count

    def is_empty(self):
        """Check if buffer is empty"""
        return self.count == 0

    def is_full(self):
        """Check if buffer is full"""
        return self.count >= self.size

    def clear(self):
        """Clear all data from buffer"""
        self.head = 0
        self.tail = 0
        self.count = 0

    def get_stats(self):
        """Get buffer statistics"""
        return {
            "size": self.size,
            "count": self.count,
            "free": self.size - self.count,
            "overflow": self.overflow_count,
        }


class MessageBuffer:
    """Buffer for complete protocol messages"""

    def __init__(self, max_messages=32):
        self.max_messages = max_messages
        self.messages = []
        self.dropped_count = 0

    def put(self, message, priority=0):
        """Add message with priority (higher = more important)"""
        if len(self.messages) >= self.max_messages:
            # Drop lowest priority message
            if not self.messages or priority <= min(msg[1] for msg in self.messages):
                self.dropped_count += 1
                return False
            # Remove lowest priority message
            min_idx = 0
            min_priority = self.messages[0][1]
            for i, (_, p) in enumerate(self.messages):
                if p < min_priority:
                    min_idx = i
                    min_priority = p
            self.messages.pop(min_idx)
            self.dropped_count += 1

        # Insert by priority
        insert_pos = len(self.messages)
        for i, (_, p) in enumerate(self.messages):
            if priority > p:
                insert_pos = i
                break
        self.messages.insert(insert_pos, (message, priority))
        return True

    def get(self):
        """Get highest priority message"""
        if self.messages:
            msg, _ = self.messages.pop(0)
            return msg
        return None

    def peek(self):
        """Peek at highest priority message"""
        if self.messages:
            return self.messages[0][0]
        return None

    def available(self):
        """Return number of messages available"""
        return len(self.messages)

    def clear(self):
        """Clear all messages"""
        self.messages.clear()

    def get_stats(self):
        """Get buffer statistics"""
        return {
            "count": len(self.messages),
            "max": self.max_messages,
            "dropped": self.dropped_count,
        }
