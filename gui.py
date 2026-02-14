import sys
import serial
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

# Configuration - Match the Arduino Struct
# Format: < (little-endian), H (uint16), I (uint32), f (float), d (double), h (int16)
# H (sync), I (time), fff (att), fff (accel), dd (gps), h (alt), fff (flight), HHH (raw)
STRUCT_FORMAT = "<HI fff fff dd h fff HHH"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)
SYNC_HEADER = 0xAA55

class TelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa Flight Telemetry - Ground Station")
        self.root.geometry("600x500")
        self.root.configure(bg="#1e1e1e")

        self.data_store = {}
        self.setup_ui()
        
        # Serial Setup (Change 'COM3' or '/dev/ttyUSB0' as needed)
        try:
            self.ser = serial.Serial('COM3', 115200, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self.read_serial, daemon=True)
            self.thread.start()
        except Exception as e:
            print(f"Serial Error: {e}. GUI running in demo mode.")
            self.ser = None

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TLabel", foreground="white", background="#1e1e1e", font=("Consolas", 10))
        style.configure("Header.TLabel", font=("Consolas", 12, "bold"), foreground="#00ff00")

        frames = {
            "Attitude": ["Pitch", "Roll", "Yaw"],
            "Motion": ["AccX", "AccY", "AccZ"],
            "Navigation": ["Lat", "Lon", "Alt"],
            "Flight Data": ["Airspeed", "Voltage", "Current"],
            "Raw ADC": ["Raw_Air", "Raw_Volt", "Raw_Curr"]
        }

        row = 0
        for section, fields in frames.items():
            f = ttk.Frame(self.root, padding=10, style="TFrame")
            f.grid(row=row//2, column=row%2, sticky="nsew")
            ttk.Label(f, text=section, style="Header.TLabel").pack(anchor="w")
            
            for field in fields:
                lbl_frame = ttk.Frame(f)
                lbl_frame.pack(fill="x")
                ttk.Label(lbl_frame, text=f"{field}:").pack(side="left")
                val_var = tk.StringVar(value="0.00")
                self.data_store[field] = val_var
                ttk.Label(lbl_frame, textvariable=val_var, foreground="#00d4ff").pack(side="right")
            row += 1

        self.status_var = tk.StringVar(value="Waiting for data...")
        ttk.Label(self.root, textvariable=self.status_var, style="TLabel").grid(row=3, column=0, columnspan=2)

    def read_serial(self):
        buffer = bytearray()
        while self.running:
            if self.ser and self.ser.in_available:
                buffer.extend(self.ser.read(self.ser.in_available))
                
                while len(buffer) >= STRUCT_SIZE:
                    # Look for sync header
                    sync = struct.unpack("<H", buffer[:2])[0]
                    if sync == SYNC_HEADER:
                        packet = buffer[:STRUCT_SIZE]
                        self.parse_packet(packet)
                        del buffer[:STRUCT_SIZE]
                    else:
                        del buffer[0] # Slide window
            time.sleep(0.01)

    def parse_packet(self, packet):
        try:
            data = struct.unpack(STRUCT_FORMAT, packet)
            # data indices mapping
            self.data_store["Pitch"].set(f"{data[2]:.2f}°")
            self.data_store["Roll"].set(f"{data[3]:.2f}°")
            self.data_store["Yaw"].set(f"{data[4]:.2f}°")
            self.data_store["AccX"].set(f"{data[5]:.2f}g")
            self.data_store["AccY"].set(f"{data[6]:.2f}g")
            self.data_store["AccZ"].set(f"{data[7]:.2f}g")
            self.data_store["Lat"].set(f"{data[8]:.6f}")
            self.data_store["Lon"].set(f"{data[9]:.6f}")
            self.data_store["Alt"].set(f"{data[10]}m")
            self.data_store["Airspeed"].set(f"{data[11]:.2f}m/s")
            self.data_store["Voltage"].set(f"{data[12]:.2f}V")
            self.data_store["Current"].set(f"{data[13]:.2f}A")
            self.data_store["Raw_Air"].set(data[14])
            self.data_store["Raw_Volt"].set(data[15])
            self.data_store["Raw_Curr"].set(data[16])
            
            self.status_var.set(f"Connected | Packet Age: {time.time()*1000 - data[1]:.0f}ms")
        except Exception as e:
            print(f"Parse Error: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = TelemetryGUI(root)
    root.mainloop()