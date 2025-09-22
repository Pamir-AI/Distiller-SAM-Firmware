"""
Priority Queue for Message Handling
Ensures high-priority messages are processed first
"""


class PriorityQueue:
    """Priority queue implementation"""

    def __init__(self, max_size=32):
        self.max_size = max_size
        self.queue = []
        self.dropped = 0

    def put(self, item, priority):
        """Add item with priority (higher value = higher priority)"""
        # Check if queue is full
        if len(self.queue) >= self.max_size:
            # Check if new item has higher priority than lowest
            if self.queue and priority <= self.queue[-1][0]:
                self.dropped += 1
                return False
            # Remove lowest priority item
            self.queue.pop()
            self.dropped += 1

        # Binary search for insertion position
        left, right = 0, len(self.queue)
        while left < right:
            mid = (left + right) // 2
            if self.queue[mid][0] > priority:
                left = mid + 1
            else:
                right = mid

        # Insert item at correct position
        self.queue.insert(left, (priority, item))
        return True

    def get(self):
        """Get highest priority item"""
        if self.queue:
            _, item = self.queue.pop(0)
            return item
        return None

    def peek(self):
        """Peek at highest priority item without removing"""
        if self.queue:
            return self.queue[0][1]
        return None

    def remove(self, item):
        """Remove specific item from queue"""
        for i, (_, q_item) in enumerate(self.queue):
            if q_item == item:
                self.queue.pop(i)
                return True
        return False

    def clear(self):
        """Clear all items"""
        self.queue.clear()

    def size(self):
        """Get current queue size"""
        return len(self.queue)

    def is_empty(self):
        """Check if queue is empty"""
        return len(self.queue) == 0

    def is_full(self):
        """Check if queue is full"""
        return len(self.queue) >= self.max_size

    def get_all_priorities(self):
        """Get list of all priorities in queue"""
        return [p for p, _ in self.queue]

    def get_stats(self):
        """Get queue statistics"""
        priorities = [p for p, _ in self.queue]
        return {
            "size": len(self.queue),
            "max_size": self.max_size,
            "dropped": self.dropped,
            "min_priority": min(priorities) if priorities else 0,
            "max_priority": max(priorities) if priorities else 0,
        }


class MessagePriorityQueue(PriorityQueue):
    """Priority queue specialized for SAM protocol messages"""

    def __init__(self, max_size=32):
        super().__init__(max_size)
        # Priority mappings based on message class
        self.priority_map = {
            0x30: 4,  # POWER messages - highest priority
            0x40: 3,  # DISPLAY messages
            0x20: 2,  # OUTPUT messages (LED)
            0x10: 1,  # INPUT messages - lowest priority
            0x00: 2,  # SYSTEM messages - medium priority
            0x50: 1,  # DEBUG messages - low priority
        }

    def put_message(self, message, override_priority=None):
        """Add message with automatic priority based on class"""
        if override_priority is not None:
            priority = override_priority
        else:
            # Extract message class (high nibble of message type)
            if isinstance(message, (bytes, bytearray)) and len(message) > 0:
                msg_class = message[0] & 0xF0
                priority = self.priority_map.get(msg_class, 0)
            else:
                priority = 0

        return self.put(message, priority)
