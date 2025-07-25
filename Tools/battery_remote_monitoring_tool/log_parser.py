import re
from datetime import datetime
from typing import Dict, Optional


class LogParser:
    def __init__(self):
        self.metrics_pattern = re.compile(
            r"PWR Metrics sent: \{\'battery_percent\': (\d+), "
            r"\'temperature_0_1c\': (\d+), "
            r"\'voltage_mv\': (\d+), "
            r"\'current_ma\': (-?\d+)\}"
        )
        self.timestamp_pattern = re.compile(r"^\[(\d+)\]")
        
    def parse_line(self, line: str) -> Optional[Dict]:
        match = self.metrics_pattern.search(line)
        if match:
            timestamp_match = self.timestamp_pattern.search(line)
            timestamp = int(timestamp_match.group(1)) if timestamp_match else None
            
            metrics = {
                'timestamp': timestamp,
                'system_time': datetime.now(),
                'battery_percent': int(match.group(1)),
                'temperature_0_1c': int(match.group(2)),
                'voltage_mv': int(match.group(3)),
                'current_ma': int(match.group(4)),
                'temperature_c': int(match.group(2)) / 10.0,
                'voltage_v': int(match.group(3)) / 1000.0,
                'current_a': int(match.group(4)) / 1000.0
            }
            
            return metrics
        
        return None
    
    def parse_lines(self, lines: list) -> list:
        metrics_list = []
        for line in lines:
            metrics = self.parse_line(line)
            if metrics:
                metrics_list.append(metrics)
        return metrics_list