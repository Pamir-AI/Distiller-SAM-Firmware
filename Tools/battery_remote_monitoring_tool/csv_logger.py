import csv
import os
from datetime import datetime
from pathlib import Path


class CSVLogger:
    def __init__(self, base_dir="logs"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True)
        
        self.session_start = datetime.now()
        self.filename = self.base_dir / f"battery_log_{self.session_start.strftime('%Y%m%d_%H%M%S')}.csv"
        self.file = None
        self.writer = None
        self.row_count = 0
        
        self._init_file()
    
    def _init_file(self):
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        headers = [
            'timestamp',
            'system_time',
            'elapsed_seconds',
            'battery_percent',
            'temperature_c',
            'voltage_v',
            'current_a',
            'power_w',
            'temperature_0_1c',
            'voltage_mv',
            'current_ma'
        ]
        
        self.writer.writerow(headers)
        self.file.flush()
    
    def log_metrics(self, metrics):
        if not self.writer:
            return
        
        elapsed = (metrics['system_time'] - self.session_start).total_seconds()
        power_w = metrics['voltage_v'] * metrics['current_a']
        
        row = [
            metrics.get('timestamp', ''),
            metrics['system_time'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f"{elapsed:.3f}",
            metrics['battery_percent'],
            f"{metrics['temperature_c']:.1f}",
            f"{metrics['voltage_v']:.3f}",
            f"{metrics['current_a']:.3f}",
            f"{power_w:.3f}",
            metrics['temperature_0_1c'],
            metrics['voltage_mv'],
            metrics['current_ma']
        ]
        
        self.writer.writerow(row)
        self.row_count += 1
        
        if self.row_count % 10 == 0:
            self.file.flush()
    
    def get_log_info(self):
        return {
            'filename': str(self.filename),
            'rows_logged': self.row_count,
            'session_duration': (datetime.now() - self.session_start).total_seconds()
        }
    
    def close(self):
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
            self.writer = None