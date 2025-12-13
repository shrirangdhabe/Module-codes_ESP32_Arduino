/*
  ESP32 Slave: Sensor Hub + UART Communication with Master
  - Reads sensors: IMU (BNO055), wheel encoder, ultrasonic, GPS
  - Computes robot pose (posX, posY, heading) using odometry
  - Sends binary status packets to master every 200ms
  - Receives text commands from master for robot control
  - Status packet: 0xAA, length(18), type(0x01), payload(16 bytes), checksum
*/

#include <HardwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Encoder.h>
#include <Ultrasonic.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// slave

// Pin definitions for sensors
#define TRIG_PIN 12
#define ECHO_PIN 14
#define ENCODER_A_PIN 25
#define ENCODER_B_PIN 26

// UART pins for communication with master ESP32
#define UART_RX_PIN 16  // Slave RX from Master TX
#define UART_TX_PIN 17  // Slave TX to Master RX

// Hardware serial interfaces
HardwareSerial SerialMaster(2);  // UART2 to master ESP32
HardwareSerial SerialGPS(1);     // UART1 for GPS module (RX=4, TX=15)

// Sensor objects
TinyGPSPlus gps;
Encoder wheelEncoder(ENCODER_A_PIN, ENCODER_B_PIN);
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Robot state variables
volatile long lastEncoderCount = 0;
float posX = 0.0f;
float posY = 0.0f;
float heading = 0.0f;
float imuYaw = 0.0f;
float rearObstacleDistance = 0.0f;

// Timing control
unsigned long lastSensorUpdate = 0;
unsigned long lastSendTime = 0;
const unsigned long sensorIntervalMs = 100;   // Read sensors every 100ms
const unsigned long sendIntervalMs = 200;     // Send status every 200ms

// Forward declarations
void updatePose();
void readSensors();
void sendStatusPacket();
void processTextCommand(String cmd);
uint8_t computeChecksum(uint8_t type, uint8_t *payload, uint8_t length);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("ESP32 Slave starting...");

  // Initialize UART to master ESP32
  SerialMaster.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Initialize GPS UART
  SerialGPS.begin(9600, SERIAL_8N1, 4, 15);  // Adjust pins if needed

  // Initialize I2C for IMU
  Wire.begin();

  // Initialize IMU sensor
  if (!bno.begin()) {
    Serial.println("ERROR: IMU (BNO055) init failed! Check wiring.");
  } else {
    Serial.println("IMU initialized successfully");
    bno.setExtCrystalUse(true);
  }

  // Initialize wheel encoder
  wheelEncoder.write(0);

  Serial.println("Slave ESP32 initialized and ready.");
}

void loop() {
  unsigned long now = millis();

  // Read and parse GPS data continuously
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // Read and process text commands from master ESP32
  while (SerialMaster.available()) {
    String cmd = SerialMaster.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      processTextCommand(cmd);
    }
  }

  // Periodic sensor read and pose update (every 100ms)
  if (now - lastSensorUpdate >= sensorIntervalMs) {
    readSensors();
    updatePose();
    lastSensorUpdate = now;
  }

  // Periodic status packet send to master (every 200ms)
  if (now - lastSendTime >= sendIntervalMs) {
    sendStatusPacket();
    lastSendTime = now;
  }
}

// Read all sensors and update sensor values
void readSensors() {
  // Read ultrasonic distance sensor (in cm)
  unsigned long distance = ultrasonic.read();
  if (distance > 0 && distance < 400) {  // Valid range check
    rearObstacleDistance = (float)distance;
  } else {
    // Keep last valid reading if current reading is invalid
  }

  // Read IMU orientation (yaw/heading)
  sensors_event_t event;
  if (bno.getEvent(&event)) {
    imuYaw = event.orientation.x;  // Euler angle X = yaw (adjust axis if needed)
    heading = imuYaw;
  } else {
    Serial.println("WARNING: IMU read failed");
  }

  // Optionally use GPS position if valid
  if (gps.location.isValid()) {
    // Can update global position or use for calibration
    float gpsLat = gps.location.lat();
    float gpsLng = gps.location.lng();
    // Serial.printf("GPS: %.6f, %.6f\n", gpsLat, gpsLng);
  }
}

// Update robot pose using wheel encoder and IMU heading (dead reckoning)
void updatePose() {
  long encoderCount = wheelEncoder.read();
  long deltaCount = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;

  // Convert encoder ticks to distance traveled (adjust calibration factor)
  // Example: 0.01 meters per tick (tune this based on your wheel/encoder setup)
  float distanceMoved = deltaCount * 0.01f;

  // Update pose using current heading (convert degrees to radians)
  float headingRad = heading * DEG_TO_RAD;
  posX += distanceMoved * cos(headingRad);
  posY += distanceMoved * sin(headingRad);
}

// Send binary status packet to master ESP32
void sendStatusPacket() {
  // Prepare payload: 4 floats = 16 bytes (posX, posY, heading, obstacle distance)
  uint8_t payload[16];
  memcpy(payload, &posX, 4);
  memcpy(payload + 4, &posY, 4);
  memcpy(payload + 8, &heading, 4);
  memcpy(payload + 12, &rearObstacleDistance, 4);

  // Calculate checksum
  uint8_t checksum = computeChecksum(0x01, payload, 16);

  // Build and send packet
  SerialMaster.write(0xAA);        // Start byte
  SerialMaster.write(18);          // Length: type(1) + payload(16) + checksum(1) = 18
  SerialMaster.write(0x01);        // Packet type 0x01 = status
  SerialMaster.write(payload, 16); // Payload (4 floats)
  SerialMaster.write(checksum);    // XOR checksum

  // Debug output (optional - comment out for production)
  // Serial.printf("Sent status: X=%.2f Y=%.2f H=%.1f Obs=%.1f\n", posX, posY, heading, rearObstacleDistance);
}

// Compute XOR checksum over type byte and payload
uint8_t computeChecksum(uint8_t type, uint8_t *payload, uint8_t length) {
  uint8_t cs = type;
  for (uint8_t i = 0; i < length; i++) {
    cs ^= payload[i];
  }
  return cs;
}

// Process text commands received from master ESP32
void processTextCommand(String cmd) {
  Serial.printf("Received command: %s\n", cmd.c_str());

  // Parse joint angle commands: "B90 S45 E135 W90"
  if (cmd.startsWith("B")) {
    int base, shoulder, elbow, wrist;
    if (sscanf(cmd.c_str(), "B%d S%d E%d W%d", &base, &shoulder, &elbow, &wrist) == 4) {
      Serial.printf("Joint angles: Base=%d Shoulder=%d Elbow=%d Wrist=%d\n", 
                    base, shoulder, elbow, wrist);
      
      // TODO: Forward to Arduino Mega via Serial or I2C
      // Example: Serial.printf("ARM:%d,%d,%d,%d\n", base, shoulder, elbow, wrist);
      
      // Or control servos directly if connected to this ESP32
    } else {
      Serial.println("ERROR: Failed to parse joint command");
    }
  }
  // Parse vegetable picking command: "PICK tomato 5"
  else if (cmd.startsWith("PICK")) {
    char veggieName[32];
    int count;
    if (sscanf(cmd.c_str(), "PICK %31s %d", veggieName, &count) == 2) {
      Serial.printf("Pick command: vegetable=%s count=%d\n", veggieName, count);
      
      // TODO: Trigger computer vision detection and picking routine
      // Example: Start detection algorithm, move arm to pick
      
    } else {
      Serial.println("ERROR: Failed to parse PICK command");
    }
  }
  // Add more command types as needed
  else if (cmd.startsWith("STOP")) {
    Serial.println("Emergency stop command received");
    // TODO: Stop all motors immediately
  }
  else if (cmd.startsWith("HOME")) {
    Serial.println("Homing command received");
    // TODO: Move robot/arm to home position
  }
  else {
    Serial.printf("WARNING: Unknown command: %s\n", cmd.c_str());
  }
}
