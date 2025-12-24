#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// Wi-Fi Credentials
const char* ssid = "Gurukul";
const char* password = "Gurukul@123";
const char* flaskServer = "http://<flask-server-ip>:5000"; // Replace with Flask server IP

WebServer server(80);
HardwareSerial SerialSlave(2);
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Global variables for latest slave data
volatile float slavePosX = 0.0f;
volatile float slavePosY = 0.0f;
volatile float slaveHeading = 0.0f;
volatile float slaveObstacle = 0.0f;

// UART parsing state
enum RecvState { WAIT_START, WAIT_LENGTH, WAIT_TYPE, WAIT_PAYLOAD, WAIT_CHECKSUM };
RecvState recvState = WAIT_START;
uint8_t recvLength = 0, recvType = 0, recvPayloadPos = 0, recvChecksum = 0;
uint8_t recvPayload[32];

// Flask sync
bool triggerReceived = false;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  SerialSlave.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Add endpoint for Flask commands
  server.on("/command", HTTP_POST, []() {
    String command = server.arg("plain");
    Serial.printf("Received command from Flask: %s\n", command.c_str());

    // Forward command to Slave ESP32
    SerialSlave.println(command);

    // Optionally read response from Slave ESP32
    delay(30);
    String response = "";
    while (SerialSlave.available()) {
      response += (char)SerialSlave.read();
    }

    if (response.length() > 0) {
      server.send(200, "text/plain", "Command sent. Slave response: " + response);
    } else {
      server.send(200, "text/plain", "Command sent: " + command);
    }
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();  // Handle Flask commands

  // Read and parse Slave ESP32 status packets
  while (SerialSlave.available()) {
    uint8_t b = (uint8_t)SerialSlave.read();
    parseIncomingByte(b);
  }

  // Send metadata to Flask every 1 second if trigger received
  if (triggerReceived) {
    unsigned long now = millis();
    if (now - lastSendTime >= sendInterval) {
      sendMetadataToFlask();
      lastSendTime = now;
    }
  }
}

void sendMetadataToFlask() {
  HTTPClient http;
  String url = String(flaskServer) + "/metadata";
  http.begin(url.c_str());
  http.addHeader("Content-Type", "application/json");

  String payload = "{\"timestamp\": " + String(millis()) + 
                   ", \"posX\": " + String(slavePosX, 2) + 
                   ", \"posY\": " + String(slavePosY, 2) + 
                   ", \"heading\": " + String(slaveHeading, 1) + 
                   ", \"obstacle\": " + String(slaveObstacle, 2) + "}";

  int httpResponseCode = http.POST(payload);
  http.end();
}

// UART parsing and packet handling functions
void parseIncomingByte(uint8_t b) {
  switch (recvState) {
    case WAIT_START:
      if (b == 0xAA) {
        recvPayloadPos = 0;
        recvChecksum = 0;
        recvState = WAIT_LENGTH;
      }
      break;
    case WAIT_LENGTH:
      recvLength = b;
      if (recvLength < 2 || recvLength > (sizeof(recvPayload) + 1)) {
        Serial.println(F("UART length error"));
        recvState = WAIT_START;
      } else {
        recvPayloadPos = 0;
        recvChecksum = 0;
        recvState = WAIT_TYPE;
      }
      break;
    case WAIT_TYPE:
      recvType = b;
      recvChecksum = b;
      if (recvLength <= 2) {
        recvState = WAIT_CHECKSUM;
      } else {
        recvState = WAIT_PAYLOAD;
      }
      break;
    case WAIT_PAYLOAD:
      if (recvPayloadPos < sizeof(recvPayload)) {
        recvPayload[recvPayloadPos++] = b;
        recvChecksum ^= b;
      } else {
        Serial.println(F("UART payload overflow"));
        recvState = WAIT_START;
        break;
      }
      if (recvPayloadPos >= (uint8_t)(recvLength - 2)) {
        recvState = WAIT_CHECKSUM;
      }
      break;
    case WAIT_CHECKSUM:
      if (recvChecksum == b) {
        handlePacket(recvType, recvPayload, recvPayloadPos);
      } else {
        Serial.println(F("UART checksum error"));
      }
      recvState = WAIT_START;
      break;
    default:
      recvState = WAIT_START;
      break;
  }
}

void handlePacket(uint8_t type, uint8_t* payload, uint8_t length) {
  if (type == 0x01 && length >= 16) {
    float x, y, h, o;
    memcpy(&x, payload, 4);
    memcpy(&y, payload + 4, 4);
    memcpy(&h, payload + 8, 4);
    memcpy(&o, payload + 12, 4);

    noInterrupts();
    slavePosX = x;
    slavePosY = y;
    slaveHeading = h;
    slaveObstacle = o;
    interrupts();

    Serial.printf("Status packet received: X=%.2f Y=%.2f H=%.1f O=%.2f\n", x, y, h, o);
  } else {
    Serial.printf("Unknown packet type 0x%02X length %d\n", type, length);
  }
}
