#!/usr/bin/env python3

import sys
import time
import threading
import argparse
import matplotlib.pyplot as plt
from datetime import datetime

from ssh_manager import SSHManager
from log_parser import LogParser
from plotter import RealTimePlotter
from csv_logger import CSVLogger


class BatteryMonitor:
    def __init__(self, hostname, username, password, port=22):
        self.ssh_manager = SSHManager(hostname, username, password, port)
        self.log_parser = LogParser()
        self.plotter = RealTimePlotter()
        self.csv_logger = CSVLogger()
        
        self.running = False
        self.monitor_thread = None
        
    def connect(self):
        print(f"Connecting to {self.ssh_manager.hostname}...")
        if self.ssh_manager.connect():
            print("SSH connection established!")
            self.plotter.set_connection_status("Connected")
            return True
        else:
            print("Failed to establish SSH connection")
            self.plotter.set_connection_status("Connection Failed")
            return False
    
    def monitor_loop(self):
        consecutive_errors = 0
        
        while self.running:
            try:
                output_lines = self.ssh_manager.get_output()
                
                if output_lines:
                    metrics_list = self.log_parser.parse_lines(output_lines)
                    
                    for metrics in metrics_list:
                        self.plotter.add_data(metrics)
                        self.csv_logger.log_metrics(metrics)
                    
                    if metrics_list:
                        consecutive_errors = 0
                
                if not self.ssh_manager.is_alive():
                    self.plotter.set_connection_status("Connection Lost")
                    print("\nConnection lost. Attempting to reconnect...")
                    
                    self.ssh_manager.disconnect()
                    time.sleep(2)
                    
                    if self.connect():
                        print("Reconnection successful!")
                    else:
                        print("Reconnection failed. Will retry...")
                        time.sleep(5)
                else:
                    self.plotter.set_connection_status("Connected")
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                self.running = False
                break
            except Exception as e:
                consecutive_errors += 1
                print(f"Monitor error: {str(e)}")
                
                if consecutive_errors > 10:
                    print("Too many consecutive errors. Stopping monitor.")
                    self.running = False
                    break
                
                time.sleep(1)
    
    def start(self):
        if not self.connect():
            return False
        
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        try:
            fig, ani = self.plotter.start()
            plt.show()
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
        finally:
            self.stop()
        
        return True
    
    def stop(self):
        print("\nStopping monitor...")
        self.running = False
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        
        self.ssh_manager.disconnect()
        self.csv_logger.close()
        
        log_info = self.csv_logger.get_log_info()
        print(f"\nSession Summary:")
        print(f"  Log file: {log_info['filename']}")
        print(f"  Total rows logged: {log_info['rows_logged']}")
        print(f"  Session duration: {log_info['session_duration']:.1f} seconds")


def main():
    parser = argparse.ArgumentParser(description='Battery Remote Monitoring Tool')
    parser.add_argument('--host', default='192.168.0.105', help='SSH hostname/IP')
    parser.add_argument('--user', default='distiller', help='SSH username')
    parser.add_argument('--password', default='one', help='SSH password')
    parser.add_argument('--port', type=int, default=22, help='SSH port')
    
    args = parser.parse_args()
    
    print("Battery Remote Monitoring Tool")
    print("==============================")
    print(f"Target: {args.user}@{args.host}:{args.port}")
    print("Press Ctrl+C to stop monitoring\n")
    
    monitor = BatteryMonitor(args.host, args.user, args.password, args.port)
    
    try:
        if not monitor.start():
            print("Failed to start monitoring")
            sys.exit(1)
    except Exception as e:
        print(f"Error: {str(e)}")
        monitor.stop()
        sys.exit(1)


if __name__ == '__main__':
    main()