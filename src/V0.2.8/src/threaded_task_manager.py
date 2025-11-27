"""Non-blocking task manager to prevent UART interference from long-running operations"""

# pylint: disable=import-error,broad-exception-caught,consider-using-f-string
from collections import deque
import _thread
import utime


class ThreadedTaskManager:
    """Task manager that prevents blocking operations from interfering with UART communication"""

    # Task priorities
    PRIORITY_CRITICAL = 0  # UART communication (Core 1)
    PRIORITY_HIGH = 1  # Button handling, power management
    PRIORITY_NORMAL = 2  # LED animations
    PRIORITY_LOW = 3  # E-ink display operations

    # Task states
    STATE_PENDING = "PENDING"
    STATE_RUNNING = "RUNNING"
    STATE_COMPLETED = "COMPLETED"
    STATE_FAILED = "FAILED"
    STATE_CANCELLED = "CANCELLED"

    def __init__(self, debug_handler=None):
        """Initialize task manager

        Args:
            debug_handler: Optional debug handler for logging
        """
        self.debug = debug_handler

        # Task queues by priority
        self.task_queues = {
            self.PRIORITY_CRITICAL: deque((), 100),
            self.PRIORITY_HIGH: deque((), 100),
            self.PRIORITY_NORMAL: deque((), 100),
            self.PRIORITY_LOW: deque((), 100),
        }

        # Task tracking
        self.active_tasks = {}
        self.completed_tasks = deque((), 50)  # Keep last 50 completed tasks
        self.completed_tasks_maxlen = 50
        self.next_task_id = 1

        # Thread control
        self.worker_threads = {}
        self.shutdown_requested = False
        self.thread_locks = {
            "task_queues": _thread.allocate_lock(),
            "active_tasks": _thread.allocate_lock(),
            "completed_tasks": _thread.allocate_lock(),
        }

        # Core assignment strategy
        self.core_assignments = {
            self.PRIORITY_CRITICAL: 1,  # UART on Core 1 (dedicated)
            self.PRIORITY_HIGH: 0,  # High priority on Core 0
            self.PRIORITY_NORMAL: 0,  # Normal priority on Core 0
            self.PRIORITY_LOW: 0,  # Low priority on Core 0 (can be interrupted)
        }

        # Statistics
        self.stats = {
            "tasks_created": 0,
            "tasks_completed": 0,
            "tasks_failed": 0,
            "tasks_cancelled": 0,
            "core0_busy_time": 0,
            "core1_busy_time": 0,
            "last_reset": utime.ticks_ms(),
        }

        # Start worker threads
        self._start_worker_threads()

    def _debug_log(self, level, message):
        """Log debug message if debug handler available"""
        if self.debug:
            self.debug.log(level, "SYS", f"[TaskMgr] {message}")

    def _start_worker_threads(self):
        """Start worker threads for different priority levels"""
        try:
            # Core 1 worker for critical tasks (UART)
            _thread.start_new_thread(self._core1_worker, ())
            self._debug_log(2, "Core 1 worker started (CRITICAL priority)")

            # Core 0 worker for other tasks
            _thread.start_new_thread(self._core0_worker, ())
            self._debug_log(2, "Core 0 worker started (HIGH/NORMAL/LOW priority)")

        except Exception as e:
            self._debug_log(1, f"Failed to start worker threads: {e}")

    def _core1_worker(self):
        """Core 1 worker - handles only critical UART tasks"""
        self._debug_log(2, "Core 1 worker thread started")

        while not self.shutdown_requested:
            try:
                # Only process critical tasks on Core 1
                task = self._get_next_task(self.PRIORITY_CRITICAL)

                if task:
                    self._execute_task(task, core_id=1)
                else:
                    # Brief sleep when no critical tasks
                    utime.sleep_ms(1)

            except Exception as e:
                self._debug_log(1, f"Core 1 worker error: {e}")
                utime.sleep_ms(10)

    def _core0_worker(self):
        """Core 0 worker - handles all non-critical tasks with priority ordering"""
        self._debug_log(2, "Core 0 worker thread started")

        while not self.shutdown_requested:
            try:
                # Process tasks by priority (HIGH -> NORMAL -> LOW)
                task = None

                # Check high priority first
                task = self._get_next_task(self.PRIORITY_HIGH)
                if not task:
                    task = self._get_next_task(self.PRIORITY_NORMAL)
                if not task:
                    task = self._get_next_task(self.PRIORITY_LOW)

                if task:
                    self._execute_task(task, core_id=0)
                else:
                    # Brief sleep when no tasks
                    utime.sleep_ms(10)

            except Exception as e:
                self._debug_log(1, f"Core 0 worker error: {e}")
                utime.sleep_ms(10)

    def _get_next_task(self, priority):
        """Get next task from priority queue

        Args:
            priority: Task priority level

        Returns:
            Task object or None
        """
        with self.thread_locks["task_queues"]:
            queue = self.task_queues.get(priority)
            if queue and len(queue) > 0:
                return queue.popleft()
        return None

    def _execute_task(self, task, core_id):
        """Execute a task and handle results

        Args:
            task: Task object to execute
            core_id: Core ID where task is running
        """
        start_time = utime.ticks_ms()

        try:
            # Mark task as running
            with self.thread_locks["active_tasks"]:
                task["state"] = self.STATE_RUNNING
                task["start_time"] = start_time
                task["core_id"] = core_id
                self.active_tasks[task["id"]] = task

            exec_msg = "Executing task {} ({}) on Core {}".format(
                task["id"], task["name"], core_id
            )
            self._debug_log(3, exec_msg)

            # Execute the task function
            result = None
            if task["function"]:
                if task["args"]:
                    result = task["function"](*task["args"])
                else:
                    result = task["function"]()

            # Task completed successfully
            end_time = utime.ticks_ms()
            duration = utime.ticks_diff(end_time, start_time)

            with self.thread_locks["active_tasks"]:
                task["state"] = self.STATE_COMPLETED
                task["end_time"] = end_time
                task["duration_ms"] = duration
                task["result"] = result
                del self.active_tasks[task["id"]]

            # Move to completed tasks
            with self.thread_locks["completed_tasks"]:
                self.completed_tasks.append(task)
                # Manual maxlen enforcement for MicroPython compatibility
                while len(self.completed_tasks) > self.completed_tasks_maxlen:
                    self.completed_tasks.popleft()

            # Update statistics
            self.stats["tasks_completed"] += 1
            if core_id == 0:
                self.stats["core0_busy_time"] += duration
            else:
                self.stats["core1_busy_time"] += duration

            complete_msg = "Task {} completed in {}ms".format(task["id"], duration)
            self._debug_log(3, complete_msg)

            # Call completion callback if provided
            if task.get("completion_callback"):
                try:
                    task["completion_callback"](task["id"], result)
                except Exception as e:
                    callback_msg = "Task {} callback failed: {}".format(task["id"], e)
                    self._debug_log(2, callback_msg)

        except Exception as e:
            # Task failed
            end_time = utime.ticks_ms()
            duration = utime.ticks_diff(end_time, start_time)

            with self.thread_locks["active_tasks"]:
                task["state"] = self.STATE_FAILED
                task["end_time"] = end_time
                task["duration_ms"] = duration
                task["error"] = str(e)
                if task["id"] in self.active_tasks:
                    del self.active_tasks[task["id"]]

            # Move to completed tasks
            with self.thread_locks["completed_tasks"]:
                self.completed_tasks.append(task)
                # Manual maxlen enforcement for MicroPython compatibility
                while len(self.completed_tasks) > self.completed_tasks_maxlen:
                    self.completed_tasks.popleft()

            self.stats["tasks_failed"] += 1
            error_msg = "Task {} ({}) failed: {}".format(task["id"], task["name"], e)
            self._debug_log(1, error_msg)

            # Call error callback if provided
            if task.get("error_callback"):
                try:
                    task["error_callback"](task["id"], str(e))
                except Exception as callback_error:
                    err_callback_msg = "Task {} error callback failed: {}".format(
                        task["id"], callback_error
                    )
                    self._debug_log(1, err_callback_msg)

    def submit_task(
        self,
        name,
        function,
        args=None,
        priority=PRIORITY_NORMAL,
        completion_callback=None,
        error_callback=None,
    ):
        """Submit a task for execution

        Args:
            name: Human-readable task name
            function: Function to execute
            args: Function arguments (tuple)
            priority: Task priority level
            completion_callback: Function to call on completion (task_id, result)
            error_callback: Function to call on error (task_id, error_msg)

        Returns:
            int: Task ID
        """
        task_id = self.next_task_id
        self.next_task_id += 1

        task = {
            "id": task_id,
            "name": name,
            "function": function,
            "args": args,
            "priority": priority,
            "state": self.STATE_PENDING,
            "created_time": utime.ticks_ms(),
            "completion_callback": completion_callback,
            "error_callback": error_callback,
            "start_time": None,
            "end_time": None,
            "duration_ms": None,
            "core_id": None,
            "result": None,
            "error": None,
        }

        # Add to appropriate priority queue
        with self.thread_locks["task_queues"]:
            if priority in self.task_queues:
                self.task_queues[priority].append(task)
            else:
                priority_msg = "Invalid priority {}, using NORMAL".format(priority)
                self._debug_log(2, priority_msg)
                self.task_queues[self.PRIORITY_NORMAL].append(task)

        self.stats["tasks_created"] += 1
        submit_msg = "Task {} ({}) submitted with priority {}".format(
            task_id, name, priority
        )
        self._debug_log(3, submit_msg)

        return task_id

    def submit_uart_task(self, name, function, args=None, completion_callback=None):
        """Submit a critical UART task (runs on Core 1)

        Args:
            name: Task name
            function: Function to execute
            args: Function arguments
            completion_callback: Completion callback

        Returns:
            int: Task ID
        """
        return self.submit_task(
            name, function, args, self.PRIORITY_CRITICAL, completion_callback
        )

    def submit_led_task(self, name, function, args=None, completion_callback=None):
        """Submit an LED animation task (normal priority)

        Args:
            name: Task name
            function: Function to execute
            args: Function arguments
            completion_callback: Completion callback

        Returns:
            int: Task ID
        """
        return self.submit_task(
            name, function, args, self.PRIORITY_NORMAL, completion_callback
        )

    def submit_display_task(self, name, function, args=None, completion_callback=None):
        """Submit a display task (low priority, can be interrupted)

        Args:
            name: Task name
            function: Function to execute
            args: Function arguments
            completion_callback: Completion callback

        Returns:
            int: Task ID
        """
        return self.submit_task(
            name, function, args, self.PRIORITY_LOW, completion_callback
        )

    def cancel_task(self, task_id):
        """Cancel a pending task

        Args:
            task_id: ID of task to cancel

        Returns:
            bool: True if cancelled, False if not found or already running
        """
        # Check if task is in any queue
        with self.thread_locks["task_queues"]:
            for _, queue in self.task_queues.items():
                for i, task in enumerate(queue):
                    if task["id"] == task_id:
                        # Remove from queue
                        del queue[i]
                        task["state"] = self.STATE_CANCELLED
                        task["end_time"] = utime.ticks_ms()

                        # Move to completed tasks
                        with self.thread_locks["completed_tasks"]:
                            self.completed_tasks.append(task)
                            # Manual maxlen enforcement for MicroPython compatibility
                            while (
                                len(self.completed_tasks) > self.completed_tasks_maxlen
                            ):
                                self.completed_tasks.popleft()

                        self.stats["tasks_cancelled"] += 1
                        cancel_msg = f"Task {task_id} cancelled"
                        self._debug_log(2, cancel_msg)
                        return True

        return False

    def get_task_status(self, task_id):
        """Get status of a specific task

        Args:
            task_id: Task ID to check

        Returns:
            dict: Task status information or None if not found
        """
        # Check active tasks
        with self.thread_locks["active_tasks"]:
            if task_id in self.active_tasks:
                return self.active_tasks[task_id].copy()

        # Check completed tasks
        with self.thread_locks["completed_tasks"]:
            for task in self.completed_tasks:
                if task["id"] == task_id:
                    return task.copy()

        # Check pending tasks
        with self.thread_locks["task_queues"]:
            for queue in self.task_queues.values():
                for task in queue:
                    if task["id"] == task_id:
                        return task.copy()

        return None

    def get_queue_status(self):
        """Get status of all task queues

        Returns:
            dict: Queue status information
        """
        with self.thread_locks["task_queues"]:
            queue_status = {}
            for priority, queue in self.task_queues.items():
                priority_names = {
                    self.PRIORITY_CRITICAL: "CRITICAL",
                    self.PRIORITY_HIGH: "HIGH",
                    self.PRIORITY_NORMAL: "NORMAL",
                    self.PRIORITY_LOW: "LOW",
                }
                priority_name = priority_names.get(priority, f"UNKNOWN_{priority}")
                queue_status[priority_name] = len(queue)

        with self.thread_locks["active_tasks"]:
            queue_status["ACTIVE"] = len(self.active_tasks)

        return queue_status

    def get_statistics(self):
        """Get task manager statistics

        Returns:
            dict: Statistics information
        """
        current_time = utime.ticks_ms()
        uptime = utime.ticks_diff(current_time, self.stats["last_reset"])

        stats = self.stats.copy()
        stats.update({"uptime_ms": uptime, "queue_status": self.get_queue_status()})

        # Calculate core utilization
        if uptime > 0:
            stats["core0_utilization_percent"] = min(
                100, (self.stats["core0_busy_time"] * 100) // uptime
            )
            stats["core1_utilization_percent"] = min(
                100, (self.stats["core1_busy_time"] * 100) // uptime
            )
        else:
            stats["core0_utilization_percent"] = 0
            stats["core1_utilization_percent"] = 0

        return stats

    def shutdown(self):
        """Shutdown task manager and worker threads"""
        self._debug_log(2, "Shutting down task manager...")
        self.shutdown_requested = True

        # Wait a bit for threads to finish current tasks
        utime.sleep_ms(100)

        # Cancel all pending tasks
        with self.thread_locks["task_queues"]:
            for queue in self.task_queues.values():
                while queue:
                    task = queue.popleft()
                    task["state"] = self.STATE_CANCELLED
                    task["end_time"] = utime.ticks_ms()
                    with self.thread_locks["completed_tasks"]:
                        self.completed_tasks.append(task)
                    self.stats["tasks_cancelled"] += 1

        self._debug_log(2, "Task manager shutdown complete")
