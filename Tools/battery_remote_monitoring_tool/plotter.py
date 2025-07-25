import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import CheckButtons
from collections import deque
from datetime import datetime
import numpy as np


class RealTimePlotter:
    def __init__(self, max_points=300, update_interval=100):
        self.max_points = max_points
        self.update_interval = update_interval
        
        # Use lists instead of deque for unlimited history
        self.times = []
        self.battery_percent = []
        self.temperature_c = []
        self.voltage_v = []
        self.current_a = []
        
        # Keep recent data for rolling window mode
        self.rolling_window_size = max_points
        
        self.start_time = datetime.now()
        self.last_update_time = None
        self.connection_status = "Disconnected"
        
        plt.style.use('dark_background')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('Battery Remote Monitoring Dashboard', fontsize=16)
        
        self.axes = self.axes.flatten()
        
        self.lines = []
        self.setup_plots()
        
        self.status_text = self.fig.text(0.02, 0.02, '', fontsize=10, 
                                        color='white', backgroundcolor='black')
        
        # Add checkbox for view mode
        self.show_full_history = False
        ax_checkbox = plt.axes([0.92, 0.02, 0.06, 0.05])
        self.checkbox = CheckButtons(ax_checkbox, ['Full History'], [False])
        self.checkbox.on_clicked(self.toggle_view_mode)
        
        self.animation = None
        
    def setup_plots(self):
        titles = ['Battery Percentage', 'Temperature', 'Voltage', 'Current']
        ylabels = ['%', '°C', 'V', 'A']
        colors = ['#00ff00', '#ff9900', '#00ccff', '#ff00ff']
        
        for i, (ax, title, ylabel, color) in enumerate(zip(self.axes, titles, ylabels, colors)):
            ax.set_title(title, fontsize=12)
            ax.set_xlabel('Time (seconds)', fontsize=10)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.grid(True, alpha=0.3)
            
            line, = ax.plot([], [], color=color, linewidth=2)
            self.lines.append(line)
            
            ax.set_xlim(0, 60)
            
            if i == 0:  # Battery percentage
                ax.set_ylim(0, 105)
            elif i == 1:  # Temperature
                ax.set_ylim(10, 60)
            elif i == 2:  # Voltage
                ax.set_ylim(3.0, 4.5)
            elif i == 3:  # Current
                ax.set_ylim(-3.0, 3.0)
                ax.axhline(y=0, color='white', linestyle='--', alpha=0.5)
    
    def add_data(self, metrics):
        current_time = (datetime.now() - self.start_time).total_seconds()
        self.times.append(current_time)
        self.battery_percent.append(metrics['battery_percent'])
        self.temperature_c.append(metrics['temperature_c'])
        self.voltage_v.append(metrics['voltage_v'])
        self.current_a.append(metrics['current_a'])
        self.last_update_time = datetime.now()
    
    def update_plot(self, frame):
        if not self.times:
            return self.lines
        
        times_array = np.array(self.times)
        data_arrays = [
            np.array(self.battery_percent),
            np.array(self.temperature_c),
            np.array(self.voltage_v),
            np.array(self.current_a)
        ]
        
        for i, (line, data) in enumerate(zip(self.lines, data_arrays)):
            if self.show_full_history:
                # Show all data
                line.set_data(times_array, data)
            else:
                # Show only recent data for rolling window
                if len(times_array) > self.rolling_window_size:
                    start_idx = len(times_array) - self.rolling_window_size
                    line.set_data(times_array[start_idx:], data[start_idx:])
                else:
                    line.set_data(times_array, data)
            
            if len(times_array) > 0:
                if self.show_full_history:
                    # Show full history from beginning
                    self.axes[i].set_xlim(0, times_array[-1] + 2)
                else:
                    # Show rolling 60-second window
                    self.axes[i].set_xlim(max(0, times_array[-1] - 60), times_array[-1] + 2)
                
                if i == 1:  # Temperature - adjust y-axis dynamically
                    if len(data) > 0:
                        ymin = max(10, min(data) - 2)
                        ymax = min(60, max(data) + 2)
                        self.axes[i].set_ylim(ymin, ymax)
        
        if self.last_update_time:
            time_since_update = (datetime.now() - self.last_update_time).total_seconds()
            if time_since_update > 2:
                self.connection_status = "No Data"
                status_color = 'red'
            else:
                self.connection_status = "Connected"
                status_color = 'green'
        else:
            status_color = 'gray'
        
        if len(data_arrays[0]) > 0:
            status_text = (f'Status: {self.connection_status} | '
                          f'Battery: {data_arrays[0][-1]:.0f}% | '
                          f'Temp: {data_arrays[1][-1]:.1f}°C | '
                          f'Voltage: {data_arrays[2][-1]:.3f}V | '
                          f'Current: {data_arrays[3][-1]:.3f}A')
        else:
            status_text = f'Status: {self.connection_status} | Waiting for data...'
        
        self.status_text.set_text(status_text)
        self.status_text.set_color(status_color)
        
        return self.lines + [self.status_text]
    
    def start(self):
        self.animation = FuncAnimation(
            self.fig, self.update_plot, interval=self.update_interval,
            blit=False, cache_frame_data=False
        )
        plt.tight_layout()
        return self.fig, self.animation
    
    def set_connection_status(self, status):
        self.connection_status = status
    
    def toggle_view_mode(self, label):
        self.show_full_history = self.checkbox.get_status()[0]