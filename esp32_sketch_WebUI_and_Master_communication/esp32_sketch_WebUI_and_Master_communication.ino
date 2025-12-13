/*
  ESP32 Master: Web UI + UART master for slave ESP32
  - Serves a control page with stereo camera streams, arm controls, and map visualization
  - Forwards UI commands to slave over UART2 (GPIO16 RX, GPIO17 TX)
  - Receives binary status packets from slave: start(0xAA), length, type, payload..., checksum (XOR)
  - Status packet type 0x01 payload: float posX, float posY, float heading, float obstacle (16 bytes)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiMulti.h>

// Wi-Fi Credentials (multiple networks)
const char* ssids[] = { "Gurukul", "dlink-4BB0" };
const char* passwords[] = { "Gurukul@123", "Gurukul@123" };
const int wifiCount = sizeof(ssids) / sizeof(ssids[0]);

WiFiMulti wifiMulti;          // Manage multi-network Wi-Fi connection
WebServer server(80);         // Web server on port 80

// UART2 interface for communication with slave ESP32 on GPIO16 (RX) and GPIO17 (TX)
HardwareSerial SerialSlave(2);
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Global variables for latest slave data
volatile float slavePosX = 0.0f;
volatile float slavePosY = 0.0f;
volatile float slaveHeading = 0.0f;
volatile float slaveObstacle = 0.0f;

// UART parsing state variables
enum RecvState { WAIT_START, WAIT_LENGTH, WAIT_TYPE, WAIT_PAYLOAD, WAIT_CHECKSUM };
RecvState recvState = WAIT_START;
uint8_t recvLength = 0, recvType = 0, recvPayloadPos = 0, recvChecksum = 0;
uint8_t recvPayload[32];

// Forward declarations
void handleRoot();
void handleCommand();
void handleStatus();
void parseIncomingByte(uint8_t b);
void handlePacket(uint8_t type, uint8_t* payload, uint8_t length);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 Master starting...");

  // Initialize UART2 for slave communication
  SerialSlave.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  for (int i = 0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passwords[i]);
  }
  Serial.print("Connecting to WiFi");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("Connected! IP address: %s\n", WiFi.localIP().toString().c_str());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.on("/status", handleStatus);

  server.begin();
  Serial.println("Web server started on port 80.");
}

void loop() {
  // Keep WiFi alive
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Attempting reconnect...");
  }

  server.handleClient();

  // Read bytes from slave and parse
  while (SerialSlave.available()) {
    uint8_t b = (uint8_t)SerialSlave.read();
    parseIncomingByte(b);
  }
}

// HTML page with embedded JavaScript - all inside this string literal
const char* controlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Garden Robot Control & Vision Hub</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: Arial; margin: 12px; background-color: #000; color: #fff; }
  h2 { color: #4CAF50; margin-bottom: 6px; }
  h3 { color: #8BC34A; margin: 8px 0; }
  label, button, input[type=range], input[type=text], input[type=number] {
    margin: 6px 0; width: 100%;
  }
  button { background-color: #4CAF50; color: white; border: none; padding: 10px; cursor: pointer; font-size: 16px; }
  button:hover { background-color: #45a049; }
  .video-stream { width: 320px; height: 240px; border: 1px solid #444; margin:5px; background-color: #000; object-fit: cover; }
  #streams { display: flex; flex-wrap: wrap; gap: 8px; }
  #arm-svg { border: 1px solid #333; width: 320px; height: 300px; margin-top: 12px; background-color: #000; }
  #robotMap { background-color: #111; border: 1px solid #333; margin-top: 12px; width: 100%; max-width: 420px; }
  .controls { max-width: 420px; }
</style>
</head>
<body>
<h2>Robot Control & Stereo Vision Hub</h2>

<div id="streams">
  <div>
    <h3>Stereo Camera #1</h3>
    <img class="video-stream" src="http://192.168.4.2:81/stream" alt="Stereo Cam 1 Stream" />
  </div>
  <div>
    <h3>Stereo Camera #2</h3>
    <img class="video-stream" src="http://192.168.4.3:81/stream" alt="Stereo Cam 2 Stream" />
  </div>
  <div>
    <h3>Gripper Camera</h3>
    <img class="video-stream" src="http://192.168.4.4:81/stream" alt="Gripper Cam Stream" />
  </div>
</div>

<div class="controls">
  <h3>Arm Joint Controls</h3>
  <label>Base: <span id="baseLabel">90</span></label>
  <input type="range" min="0" max="180" value="90" id="base" oninput="updateAngle('base')">
  <label>Shoulder: <span id="shoulderLabel">90</span></label>
  <input type="range" min="0" max="180" value="90" id="shoulder" oninput="updateAngle('shoulder')">
  <label>Elbow: <span id="elbowLabel">90</span></label>
  <input type="range" min="0" max="180" value="90" id="elbow" oninput="updateAngle('elbow')">
  <label>Wrist: <span id="wristLabel">90</span></label>
  <input type="range" min="0" max="180" value="90" id="wrist" oninput="updateAngle('wrist')">
  <button onclick="sendJointCommand()">Send Joint Angles</button>

  <h3>Vegetable Detection & Picking</h3>
  <label for="veggieName">Vegetable Name</label>
  <input type="text" id="veggieName" placeholder="e.g., tomato" />
  <label for="veggieCount">Count to Pick</label>
  <input type="number" id="veggieCount" min="1" value="1" />
  <button onclick="sendVeggieCommand()">Start Detect & Pick</button>

  <h3>Robotic Arm Position Visualization</h3>
  <svg id="arm-svg" viewBox="0 0 200 250">
    <circle cx="100" cy="230" r="5" fill="#4CAF50"></circle>
    <line id="base-shoulder" x1="100" y1="230" x2="100" y2="180" stroke="#2196F3" stroke-width="6"></line>
    <line id="shoulder-elbow" x1="100" y1="180" x2="100" y2="130" stroke="#2196F3" stroke-width="6"></line>
    <line id="elbow-wrist" x1="100" y1="130" x2="100" y2="80" stroke="#2196F3" stroke-width="6"></line>
    <line id="wrist-end" x1="100" y1="80" x2="100" y2="50" stroke="#F44336" stroke-width="8"></line>
    <circle id="shoulder-joint" cx="100" cy="180" r="8" fill="#FFC107"></circle>
    <circle id="elbow-joint" cx="100" cy="130" r="8" fill="#FFC107"></circle>
    <circle id="wrist-joint" cx="100" cy="80" r="8" fill="#FFC107"></circle>
  </svg>

  <h3>Robot Pose & Obstacle Visualization</h3>
  <canvas id="robotMap" width="400" height="400"></canvas>
</div>

<script>
function sendCommand(cmd) {
  // cmd should be already encoded
  fetch('/command?cmd=' + cmd).catch(e => console.error(e));
}

function sendJointCommand() {
  const cmd = 'B' + document.getElementById('base').value + ' ' +
              'S' + document.getElementById('shoulder').value + ' ' +
              'E' + document.getElementById('elbow').value + ' ' +
              'W' + document.getElementById('wrist').value;
  sendCommand(encodeURIComponent(cmd));
}

function sendVeggieCommand() {
  const nameRaw = document.getElementById('veggieName').value.trim();
  const name = encodeURIComponent(nameRaw);
  const count = parseInt(document.getElementById('veggieCount').value);
  if (!nameRaw) { alert('Please enter a vegetable name!'); return; }
  if (isNaN(count) || count < 1) { alert('Count must be at least 1!'); return; }
  sendCommand(encodeURIComponent('PICK ' + nameRaw + ' ' + count));
}

function updateAngle(part) {
  let val = document.getElementById(part).value;
  document.getElementById(part + 'Label').innerText = val;
  updateArmVisualization();
}

function updateArmVisualization() {
  const length1 = 50, length2 = 50, length3 = 50, length4 = 30;
  const baseX = 100, baseY = 230;

  const baseAngle = parseInt(document.getElementById('base').value) * Math.PI / 180;
  const shoulderAngle = parseInt(document.getElementById('shoulder').value) * Math.PI / 180;
  const elbowAngle = parseInt(document.getElementById('elbow').value) * Math.PI / 180;
  const wristAngle = parseInt(document.getElementById('wrist').value) * Math.PI / 180;

  let shoulderX = baseX + length1 * Math.sin(baseAngle);
  let shoulderY = baseY - length1 * Math.cos(baseAngle);
  let elbowX = shoulderX + length2 * Math.sin(baseAngle + shoulderAngle);
  let elbowY = shoulderY - length2 * Math.cos(baseAngle + shoulderAngle);
  let wristX = elbowX + length3 * Math.sin(baseAngle + shoulderAngle + elbowAngle);
  let wristY = elbowY - length3 * Math.cos(baseAngle + shoulderAngle + elbowAngle);
  let endX = wristX + length4 * Math.sin(baseAngle + shoulderAngle + elbowAngle + wristAngle);
  let endY = wristY - length4 * Math.cos(baseAngle + shoulderAngle + elbowAngle + wristAngle);

  document.getElementById('base-shoulder').setAttribute('x2', shoulderX);
  document.getElementById('base-shoulder').setAttribute('y2', shoulderY);
  document.getElementById('shoulder-elbow').setAttribute('x1', shoulderX);
  document.getElementById('shoulder-elbow').setAttribute('y1', shoulderY);
  document.getElementById('shoulder-elbow').setAttribute('x2', elbowX);
  document.getElementById('shoulder-elbow').setAttribute('y2', elbowY);
  document.getElementById('elbow-wrist').setAttribute('x1', elbowX);
  document.getElementById('elbow-wrist').setAttribute('y1', elbowY);
  document.getElementById('elbow-wrist').setAttribute('x2', wristX);
  document.getElementById('elbow-wrist').setAttribute('y2', wristY);
  document.getElementById('wrist-end').setAttribute('x1', wristX);
  document.getElementById('wrist-end').setAttribute('y1', wristY);
  document.getElementById('wrist-end').setAttribute('x2', endX);
  document.getElementById('wrist-end').setAttribute('y2', endY);
  document.getElementById('shoulder-joint').setAttribute('cx', shoulderX);
  document.getElementById('shoulder-joint').setAttribute('cy', shoulderY);
  document.getElementById('elbow-joint').setAttribute('cx', elbowX);
  document.getElementById('elbow-joint').setAttribute('cy', elbowY);
  document.getElementById('wrist-joint').setAttribute('cx', wristX);
  document.getElementById('wrist-joint').setAttribute('cy', wristY);
}

function drawRobotMap(posX, posY, heading, obstacle) {
  const canvas = document.getElementById('robotMap');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const scale = 20; // pixels per meter (adjust to your map scale)
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;

  let x = centerX + posX * scale;
  let y = centerY - posY * scale;

  // Robot body
  ctx.beginPath();
  ctx.arc(x, y, 15, 0, 2 * Math.PI);
  ctx.fillStyle = "#4CAF50";
  ctx.fill();

  // Heading line
  ctx.beginPath();
  let rad = heading * Math.PI / 180;
  ctx.moveTo(x, y);
  ctx.lineTo(x + 25 * Math.cos(rad), y - 25 * Math.sin(rad));
  ctx.strokeStyle = "#FFC107";
  ctx.lineWidth = 4;
  ctx.stroke();

  // Obstacle radius (visual)
  if (!isNaN(obstacle) && obstacle > 0) {
    ctx.beginPath();
    ctx.arc(x, y, obstacle * scale, 0, 2 * Math.PI);
    ctx.strokeStyle = "rgba(255, 0, 0, 0.5)";
    ctx.lineWidth = 2;
    ctx.stroke();
  }
}

// Poll status and update map
setInterval(() => {
  fetch('/status')
    .then(res => res.json())
    .then(data => {
      drawRobotMap(data.posX, data.posY, data.heading, data.obstacle);
    })
    .catch(e => {
      // console.error(e);
    });
}, 250);

// Initialize visualization on load
window.onload = function() {
  updateArmVisualization();
};
</script>
</body>
</html>
)rawliteral";

// Serve the control page
void handleRoot() {
  server.send(200, "text/html", controlPage);
}

// Forward commands from UI to slave ESP32 over UART
void handleCommand() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd"); // server decodes percent-encoding
    Serial.printf("Received command: %s\n", cmd.c_str());

    // Basic sanitization
    if (cmd.length() > 128) {
      server.send(400, "text/plain", "Command too long");
      return;
    }

    // Forward to slave
    SerialSlave.println(cmd);

    // Optionally read immediate response from slave (non-blocking small wait)
    delay(30);
    String response = "";
    while (SerialSlave.available()) {
      response += (char)SerialSlave.read();
    }

    if (response.length() > 0) {
      server.send(200, "text/plain", "Command sent. Slave response: " + response);
    } else {
      server.send(200, "text/plain", "Command sent: " + cmd);
    }
  } else {
    server.send(400, "text/plain", "No command received");
  }
}

// Provide latest slave ESP32 status as JSON for UI updates
void handleStatus() {
  // Use local copies to avoid volatile read issues
  float x = slavePosX;
  float y = slavePosY;
  float h = slaveHeading;
  float o = slaveObstacle;

  String json = "{";
  json += "\"posX\":" + String(x, 2) + ",";
  json += "\"posY\":" + String(y, 2) + ",";
  json += "\"heading\":" + String(h, 1) + ",";
  json += "\"obstacle\":" + String(o, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// Parse bytes from slave ESP32 UART with checksum verification
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
      // recvLength includes type + payload + checksum? In our design: length = type + payload + checksum? 
      // Here we treat recvLength as total bytes after length byte: type + payload + checksum
      // Validate length: must be at least 2 (type + checksum) and not exceed buffer+1
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
      recvChecksum = b; // initialize checksum with type
      // payload length = recvLength - 2 (type + checksum) ? We used recvLength as type+payload+checksum
      // We'll expect payload bytes = recvLength - 2
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
        // overflow protection
        Serial.println(F("UART payload overflow"));
        recvState = WAIT_START;
        break;
      }
      // check if we've read expected payload bytes
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

// Update global variables from valid status packet
void handlePacket(uint8_t type, uint8_t* payload, uint8_t length) {
  // Type 0x01: status packet with 4 floats (16 bytes)
  if (type == 0x01 && length >= 16) {
    // Copy bytes into floats (assumes same endianness)
    float x, y, h, o;
    memcpy(&x, payload, 4);
    memcpy(&y, payload + 4, 4);
    memcpy(&h, payload + 8, 4);
    memcpy(&o, payload + 12, 4);

    // Update shared variables atomically (simple approach)
    noInterrupts();
    slavePosX = x;
    slavePosY = y;
    slaveHeading = h;
    slaveObstacle = o;
    interrupts();

    Serial.printf("Status packet received: X=%.2f Y=%.2f H=%.1f O=%.2f\n", x, y, h, o);
  } else {
    // handle other packet types if needed
    Serial.printf("Unknown packet type 0x%02X length %d\n", type, length);
  }
}