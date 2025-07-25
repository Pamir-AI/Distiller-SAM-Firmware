import paramiko
import threading
import queue
import time
from datetime import datetime


class SSHManager:
    def __init__(self, hostname, username, password, port=22):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.port = port
        self.client = None
        self.channel = None
        self.connected = False
        self.output_queue = queue.Queue()
        self.reader_thread = None
        self.last_data_time = time.time()
        
    def connect(self):
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(
                hostname=self.hostname,
                username=self.username,
                password=self.password,
                port=self.port,
                timeout=10
            )
            
            self.channel = self.client.invoke_shell()
            self.channel.settimeout(0.1)
            self.connected = True
            
            self.reader_thread = threading.Thread(target=self._read_output, daemon=True)
            self.reader_thread.start()
            
            time.sleep(1)
            self.send_command("mpremote\n")
            
            return True
            
        except Exception as e:
            self.connected = False
            print(f"SSH Connection failed: {str(e)}")
            return False
    
    def _read_output(self):
        buffer = ""
        while self.connected:
            try:
                if self.channel.recv_ready():
                    data = self.channel.recv(4096).decode('utf-8', errors='ignore')
                    buffer += data
                    self.last_data_time = time.time()
                    
                    lines = buffer.split('\n')
                    for line in lines[:-1]:
                        self.output_queue.put(line.strip())
                    buffer = lines[-1]
                    
                time.sleep(0.01)
                
            except Exception as e:
                if self.connected:
                    print(f"Read error: {str(e)}")
                time.sleep(0.1)
    
    def send_command(self, command):
        if self.connected and self.channel:
            try:
                self.channel.send(command)
                return True
            except Exception as e:
                print(f"Send command error: {str(e)}")
                return False
        return False
    
    def get_output(self):
        output = []
        while not self.output_queue.empty():
            try:
                output.append(self.output_queue.get_nowait())
            except queue.Empty:
                break
        return output
    
    def is_alive(self, timeout=5):
        return self.connected and (time.time() - self.last_data_time) < timeout
    
    def disconnect(self):
        self.connected = False
        if self.reader_thread:
            self.reader_thread.join(timeout=2)
        if self.channel:
            self.channel.close()
        if self.client:
            self.client.close()