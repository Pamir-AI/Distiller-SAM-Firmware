# Battery Remote Monitoring Tool

A real-time battery monitoring dashboard that connects via SSH to a remote device and displays battery metrics with live plotting and CSV logging.

## Features

- Real-time monitoring of battery metrics via SSH
- Live plotting of 4 key metrics:
  - Battery percentage
  - Temperature (°C)
  - Voltage (V)
  - Current (A)
- Automatic CSV logging of all data
- Connection status indicator
- Automatic reconnection on connection loss
- Dark theme optimized for battery testing

## Installation

1. Install Python 3.7 or higher

2. Install required packages:
```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

Run with default settings (connects to distiller@192.168.0.105):
```bash
python main.py
```

### Custom Connection

Specify custom SSH credentials:
```bash
python main.py --host 192.168.0.100 --user myuser --password mypass --port 22
```

### Command Line Options

- `--host`: SSH hostname or IP address (default: 192.168.0.105)
- `--user`: SSH username (default: distiller)
- `--password`: SSH password (default: one)
- `--port`: SSH port (default: 22)

## Output

### Real-time Dashboard
- 4 live plots showing battery metrics
- Connection status indicator at the bottom
- Auto-scaling axes for optimal viewing

### CSV Logs
- Logs are saved in the `logs/` directory
- Filename format: `battery_log_YYYYMMDD_HHMMSS.csv`
- Contains all raw and calculated values with timestamps

## Log Format

The CSV files contain the following columns:
- `timestamp`: Device timestamp from log
- `system_time`: Local system time
- `elapsed_seconds`: Seconds since monitoring started
- `battery_percent`: Battery percentage (0-100)
- `temperature_c`: Temperature in Celsius
- `voltage_v`: Voltage in Volts
- `current_a`: Current in Amperes (negative = discharging)
- `power_w`: Calculated power in Watts
- `temperature_0_1c`: Raw temperature value
- `voltage_mv`: Raw voltage in millivolts
- `current_ma`: Raw current in milliamperes

## Troubleshooting

### Connection Issues
- Ensure the target device is reachable via SSH
- Verify SSH credentials are correct
- Check that `mpremote` command is available on the target device

### No Data Appearing
- The tool parses lines containing "PWR Metrics sent:"
- Ensure the remote device is outputting the expected log format

### Performance
- Adjust `max_plot_points` in config to reduce memory usage
- Increase `update_interval_ms` to reduce CPU usage