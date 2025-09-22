"""
Timer Manager for Protocol Timeouts
Handles ACK timeouts, keepalives, and retransmissions
"""

import utime


class Timer:
    """Individual timer instance"""

    def __init__(self, timeout_ms, callback, args=None, periodic=False):
        self.timeout_ms = timeout_ms
        self.callback = callback
        self.args = args if args is not None else []
        self.periodic = periodic
        self.start_time = None
        self.active = False
        self.expired = False

    def start(self):
        """Start or restart timer"""
        self.start_time = utime.ticks_ms()
        self.active = True
        self.expired = False

    def stop(self):
        """Stop timer"""
        self.active = False

    def is_expired(self):
        """Check if timer has expired"""
        if not self.active:
            return False
        if self.expired:
            return True
        if utime.ticks_diff(utime.ticks_ms(), self.start_time) >= self.timeout_ms:
            self.expired = True
            if not self.periodic:
                self.active = False
            return True
        return False

    def remaining_ms(self):
        """Get remaining time in milliseconds"""
        if not self.active:
            return 0
        elapsed = utime.ticks_diff(utime.ticks_ms(), self.start_time)
        remaining = self.timeout_ms - elapsed
        return max(0, remaining)

    def reset(self):
        """Reset timer to full duration"""
        if self.active:
            self.start()


class TimerManager:
    """Manages multiple protocol timers"""

    def __init__(self):
        self.timers = {}
        self.timer_id_counter = 0

    def create_timer(self, name, timeout_ms, callback, args=None, periodic=False):
        """Create a new timer"""
        timer = Timer(timeout_ms, callback, args, periodic)
        self.timers[name] = timer
        return timer

    def start_timer(self, name):
        """Start a timer by name"""
        if name in self.timers:
            self.timers[name].start()
            return True
        return False

    def stop_timer(self, name):
        """Stop a timer by name"""
        if name in self.timers:
            self.timers[name].stop()
            return True
        return False

    def reset_timer(self, name):
        """Reset a timer to full duration"""
        if name in self.timers:
            self.timers[name].reset()
            return True
        return False

    def is_timer_expired(self, name):
        """Check if a timer has expired"""
        if name in self.timers:
            return self.timers[name].is_expired()
        return False

    def get_timer_remaining(self, name):
        """Get remaining time for a timer"""
        if name in self.timers:
            return self.timers[name].remaining_ms()
        return 0

    def remove_timer(self, name):
        """Remove a timer"""
        if name in self.timers:
            del self.timers[name]
            return True
        return False

    def process_timers(self):
        """Process all timers and call callbacks for expired ones"""
        expired_timers = []

        for name, timer in self.timers.items():
            if timer.is_expired():
                expired_timers.append((name, timer))
                if timer.periodic:
                    timer.start()  # Restart periodic timers

        # Call callbacks
        for name, timer in expired_timers:
            try:
                timer.callback(*timer.args)
            except Exception as e:
                print(f"Timer {name} callback error: {e}")

    def get_next_timeout(self):
        """Get milliseconds until next timer expires"""
        next_timeout = None
        for timer in self.timers.values():
            if timer.active:
                remaining = timer.remaining_ms()
                if next_timeout is None or remaining < next_timeout:
                    next_timeout = remaining
        return next_timeout if next_timeout is not None else 1000

    def stop_all(self):
        """Stop all timers"""
        for timer in self.timers.values():
            timer.stop()

    def clear_all(self):
        """Clear all timers"""
        self.timers.clear()

    def get_active_timers(self):
        """Get list of active timer names"""
        return [name for name, timer in self.timers.items() if timer.active]

    def get_stats(self):
        """Get timer statistics"""
        active_count = sum(1 for t in self.timers.values() if t.active)
        return {
            "total": len(self.timers),
            "active": active_count,
            "timers": list(self.timers.keys()),
        }


class ProtocolTimers(TimerManager):
    """Protocol-specific timer manager"""

    def __init__(self, ack_timeout_ms, idle_timeout_ms, keepalive_ms):
        super().__init__()
        self.ack_timeout_ms = ack_timeout_ms
        self.idle_timeout_ms = idle_timeout_ms
        self.keepalive_ms = keepalive_ms

        # Callbacks
        self.on_ack_timeout = None
        self.on_idle_timeout = None
        self.on_keepalive = None

    def start_ack_timer(self, seq_num):
        """Start ACK timer for a sequence number"""
        timer_name = f"ack_{seq_num}"
        if self.on_ack_timeout:
            self.create_timer(
                timer_name, self.ack_timeout_ms, self.on_ack_timeout, [seq_num]
            )
            self.start_timer(timer_name)

    def stop_ack_timer(self, seq_num):
        """Stop ACK timer for a sequence number"""
        timer_name = f"ack_{seq_num}"
        self.stop_timer(timer_name)
        self.remove_timer(timer_name)

    def start_idle_timer(self):
        """Start idle timer"""
        if self.on_idle_timeout:
            self.create_timer("idle", self.idle_timeout_ms, self.on_idle_timeout)
            self.start_timer("idle")

    def reset_idle_timer(self):
        """Reset idle timer on activity"""
        self.reset_timer("idle")

    def start_keepalive_timer(self):
        """Start keepalive timer"""
        if self.on_keepalive:
            self.create_timer(
                "keepalive", self.keepalive_ms, self.on_keepalive, periodic=True
            )
            self.start_timer("keepalive")

    def stop_keepalive_timer(self):
        """Stop keepalive timer"""
        self.stop_timer("keepalive")
        self.remove_timer("keepalive")
