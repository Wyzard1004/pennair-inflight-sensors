# LoRa Flight Telemetry System

A complete wireless telemetry solution for real-time aircraft monitoring using LoRa radio modules and Arduino microcontrollers.

## Overview

This system enables live telemetry streaming from an aircraft (transmitter) to a ground station (receiver) via LoRa communication. Flight data including attitude, motion, navigation, airspeed, and power metrics are displayed in real-time on a Python GUI.

## Features

- **Real-time Data Transmission**: 10Hz update rate over LoRa radio
- **Comprehensive Sensor Data**:
  - Attitude: Pitch, Roll, Yaw
  - Motion: AccX, AccY, AccZ (g-forces)
  - Navigation: Latitude, Longitude, Altitude (m)
  - Flight Data: Airspeed (m/s), Voltage (V), Current (A)
  - Raw ADC Values: For airspeed, voltage, and current sensors
- **Live Ground Station GUI**: Dark-themed tkinter interface with real-time updates
- **Modular Design**: Single codebase configurable for transmitter or receiver mode

## Hardware Requirements

### Common to Both Nodes
- Arduino Uno/Mega or compatible microcontroller
- LoRa Module (DX-LR02-900T22D or Ebyte E22)
- USB cable for programming and serial communication

### Aircraft (Transmitter)
- IMU Sensor (6-DOF accelerometer/gyroscope)
- GPS Module
- Airspeed Sensor (analog)
- Voltage Divider (for battery monitoring)
- Current Sensor (analog)

### Ground Station (Receiver)
- LoRa module connected to receiving Arduino
- USB connection to ground station computer running Python GUI

## Software Setup

### Arduino Setup

1. **Install Arduino IDE**: Download from [arduino.cc](https://arduino.cc)
2. **Configure LoRa Module**:
   - Connect LoRa RX pin to Arduino pin 10
   - Connect LoRa TX pin to Arduino pin 11
   - Connect GND and 5V/3.3V appropriately
3. **Upload Firmware**:
   - For Aircraft: Use `transmitter/transmitter.ino` with `isTransmitter = true`
   - For Ground Station: Use `receiver/receiver.ino` with `isTransmitter = false`

#### Communication Pins
```cpp
const int LORA_RX = 10;  // LoRa module RX
const int LORA_TX = 11;  // LoRa module TX
Serial begin 115200      // USB/Serial baud rate
```

### Python GUI Setup

1. **Install Python 3.7+** and required packages:
   ```bash
   pip install pyserial
   ```

2. **Run the Ground Station GUI**:
   ```bash
   python gui.py
   ```

3. **Configure Serial Port** (in `gui.py`):
   - Change `'COM3'` to your ground station Arduino's COM port
   - On Linux/Mac: Use `/dev/ttyUSB0` or `/dev/ttyACM0`
   - Baud rate: 115200

## Data Format

All telemetry data uses a standardized binary packet structure (64 bytes total):

```c
struct SensorData {
  uint16_t sync;          // 0xAA55 header
  uint32_t timestamp;     // milliseconds
  
  float pitch, roll, yaw;           // degrees
  float accX, accY, accZ;           // g-forces
  
  double latitude, longitude;       // decimal degrees
  int16_t altitude;                 // meters
  
  float airSpeed;                   // m/s
  float voltage;                    // volts
  float current;                    // amps
  
  uint16_t rawAirspeed, rawVoltage, rawCurrent;  // ADC values
};
```

The Python GUI uses this struct to parse incoming telemetry packets:
```python
STRUCT_FORMAT = "<HI fff fff dd h fff HHH"
```

## Operation

1. **Power on the aircraft** with transmitter Arduino and sensors connected
2. **Power on the ground station** with receiver Arduino connected via USB
3. **Launch the Python GUI**: `python gui.py`
4. **Monitor telemetry** in the live dashboard showing all sensor values

The system will automatically:
- Detect sync headers (0xAA55) in incoming data
- Parse binary packets into usable values
- Update the UI at 10Hz
- Display connection status and packet age

## File Structure

```
ifs/
├── transmitter/
│   └── transmitter.ino     # Aircraft flight data sender
├── receiver/
│   └── receiver.ino        # Ground station receiver
├── gui.py                  # Ground station telemetry display
├── readme.md               # This file
└── .vscode/
    └── c_cpp_properties.json  # Arduino include paths
```

## Troubleshooting

### Arduino Compilation Errors
- **"SoftwareSerial.h not found"**: Ensure Arduino IDE is installed and `.vscode/c_cpp_properties.json` is properly configured
- **"COM port not available"**: Check USB connection and select correct port in Arduino IDE Tools menu

### GUI Issues
- **"Serial Error"**: Verify COM port in `gui.py` matches device manager
- **No data displayed**: Ensure LoRa modules are powered and properly connected
- **Garbled data**: Check baud rate (should be 115200) on both Arduinos

### LoRa Module Communication
- Verify module is powered with 5V/3.3V (check datasheet)
- Confirm TX and RX pins are correct (10 and 11)
- Ensure both modules are configured with matching frequency bands

## Future Enhancements

- Real-time data logging to CSV
- Multiple aircraft tracking
- Advanced alarm thresholds
- Wireless firmware updates via LoRa
- Integration with flight control systems
- Web-based dashboard

## License

Open source - feel free to modify and distribute