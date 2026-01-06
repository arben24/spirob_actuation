import tkinter as tk
from tkinter import ttk
import serial
import threading
import time

class ServoControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Servo Controller GUI")
        self.serial_conn = None
        self.connected = False

        # Serial Port Selection
        ttk.Label(root, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(root, width=20)
        self.port_entry.insert(0, "/dev/ttyUSB0")  # Default ESP32 port
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)

        # Connect Button
        self.connect_btn = ttk.Button(root, text="Connect", command=self.connect_serial)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=5)

        # Status Label
        self.status_label = ttk.Label(root, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=3, padx=5, pady=5)

        # Mode Buttons
        ttk.Label(root, text="Modes:").grid(row=2, column=0, padx=5, pady=5)
        self.manual_btn = ttk.Button(root, text="Set Manual Mode", command=self.set_manual_mode, state=tk.DISABLED)
        self.manual_btn.grid(row=2, column=1, padx=5, pady=5)
        self.pid_btn = ttk.Button(root, text="Set PID Mode", command=self.set_pid_mode, state=tk.DISABLED)
        self.pid_btn.grid(row=2, column=2, padx=5, pady=5)

        # Setpoint Sliders
        ttk.Label(root, text="Setpoints (mm):").grid(row=3, column=0, padx=5, pady=5)
        
        # Motor 0 Setpoint
        ttk.Label(root, text="Motor 0:").grid(row=4, column=0, padx=5, pady=5)
        self.setpoint0_scale = tk.Scale(root, from_=20, to=65, orient=tk.HORIZONTAL, command=self.update_setpoint0)
        self.setpoint0_scale.set(40)  # Default
        self.setpoint0_scale.grid(row=4, column=1, columnspan=2, padx=5, pady=5, sticky="ew")

        # Motor 1 Setpoint
        ttk.Label(root, text="Motor 1:").grid(row=5, column=0, padx=5, pady=5)
        self.setpoint1_scale = tk.Scale(root, from_=20, to=65, orient=tk.HORIZONTAL, command=self.update_setpoint1)
        self.setpoint1_scale.set(40)  # Default
        self.setpoint1_scale.grid(row=5, column=1, columnspan=2, padx=5, pady=5, sticky="ew")

        # Log Text
        self.log_text = tk.Text(root, height=10, width=50, state=tk.DISABLED)
        self.log_text.grid(row=6, column=0, columnspan=3, padx=5, pady=5)

        # Start serial reader thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()

    def connect_serial(self):
        if self.connected:
            self.disconnect_serial()
            return

        port = self.port_entry.get()
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
            self.connected = True
            self.status_label.config(text=f"Connected to {port}", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.manual_btn.config(state=tk.NORMAL)
            self.pid_btn.config(state=tk.NORMAL)
            self.log("Connected to serial port")
        except Exception as e:
            self.log(f"Failed to connect: {e}")
            self.status_label.config(text="Connection Failed", foreground="red")

    def disconnect_serial(self):
        if self.serial_conn:
            self.serial_conn.close()
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.manual_btn.config(state=tk.DISABLED)
        self.pid_btn.config(state=tk.DISABLED)
        self.log("Disconnected from serial port")

    def send_command(self, cmd):
        if self.connected and self.serial_conn:
            try:
                self.serial_conn.write((cmd + '\n').encode())
                self.log(f"Sent: {cmd}")
            except Exception as e:
                self.log(f"Send error: {e}")

    def set_manual_mode(self):
        self.send_command("manual")

    def set_pid_mode(self):
        self.send_command("pid")

    def update_setpoint0(self, value):
        if self.connected:
            self.send_command(f"setpoint:0:{value}")

    def update_setpoint1(self, value):
        if self.connected:
            self.send_command(f"setpoint:1:{value}")

    def serial_reader(self):
        while self.running:
            if self.connected and self.serial_conn:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode().strip()
                        if line:
                            self.log(f"Received: {line}")
                except Exception as e:
                    self.log(f"Read error: {e}")
                    time.sleep(0.1)
            time.sleep(0.01)

    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + '\n')
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def on_closing(self):
        self.running = False
        self.disconnect_serial()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
