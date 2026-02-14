/*
 * LoRa Telemetry System for DX-LR02-900T22D / Ebyte E22
 * Expanded for Full Flight Sensors: Attitude, IMU, GPS, Airspeed, and Power.
 * Includes raw ADC values for Airspeed, Current, and Voltage.
 */

#include <SoftwareSerial.h>

// 1. DATA STRUCTURE (The "Contract" between Plane, Ground Arduino, and Python GUI)
struct __attribute__((__packed__)) SensorData {
  uint16_t sync;        // 2 bytes: 0xAA55 (Header)
  uint32_t timestamp;   // 4 bytes: millis()
  
  // Attitude & Motion
  float pitch;          // 4 bytes
  float roll;           // 4 bytes
  float yaw;            // 4 bytes
  float accX;           // 4 bytes
  float accY;           // 4 bytes
  float accZ;           // 4 bytes
  
  // Navigation
  double latitude;      // 8 bytes 
  double longitude;     // 8 bytes
  int16_t altitude;     // 2 bytes
  
  // Flight Data
  float airSpeed;       // 4 bytes
  float voltage;        // 4 bytes
  float current;        // 4 bytes
  
  // Raw Sensor Values (ADC)
  uint16_t rawAirspeed; // 2 bytes
  uint16_t rawVoltage;  // 2 bytes
  uint16_t rawCurrent;  // 2 bytes
}; // Total: 64 bytes

const int LORA_RX = 10;
const int LORA_TX = 11;
SoftwareSerial loraSerial(LORA_RX, LORA_TX);

SensorData myData;
bool isTransmitter = true; // TOGGLE THIS: true for Plane, false for Ground Station

void setup() {
  Serial.begin(115200);
  loraSerial.begin(115200); 
  
  if(isTransmitter) Serial.println("MODE: PLANE (TX)");
  else Serial.println("MODE: GROUND (RX)");
}

void loop() {
  if (isTransmitter) {
    handleTransmitter();
  } else {
    handleReceiver();
  }
}

void handleTransmitter() {
  static uint32_t lastSend = 0;
  if (millis() - lastSend > 100) { // 10Hz Update Rate
    lastSend = millis();
    
    myData.sync = 0xAA55;
    myData.timestamp = millis();

    // --- SENSOR READING SECTION ---
    // (Replace placeholders with actual sensor library calls)
    myData.pitch = 5.2; 
    myData.roll = -1.1;
    myData.yaw = 180.5;
    myData.accX = 0.01;
    myData.accY = 0.02;
    myData.accZ = 1.0; 
    myData.latitude = 34.052235;
    myData.longitude = -118.243683;
    myData.altitude = 350;
    myData.airSpeed = 22.5; 
    myData.voltage = 11.1;  
    myData.current = 15.4;  
    
    // Raw ADC
    myData.rawAirspeed = analogRead(A0);
    myData.rawVoltage  = analogRead(A1);
    myData.rawCurrent  = analogRead(A2);

    // Send binary packet to LoRa
    loraSerial.write((uint8_t*)&myData, sizeof(SensorData));
  }
}

void handleReceiver() {
  // Wait for at least one full packet
  if (loraSerial.available() >= sizeof(SensorData)) {
    // Check for Sync Header low byte
    if (loraSerial.peek() != 0x55) { 
       loraSerial.read(); // Discard and slide window
       return;
    }

    uint8_t buffer[sizeof(SensorData)];
    loraSerial.readBytes(buffer, sizeof(SensorData));
    memcpy(&myData, buffer, sizeof(SensorData));

    if (myData.sync == 0xAA55) {
      // Pipe raw binary to USB Serial for the Python GUI to read
      Serial.write((uint8_t*)&myData, sizeof(SensorData));
    }
  }
}