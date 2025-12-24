#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// Master
#define TRIGGER_PIN 13
#define ACK_PIN 12
#define SENSOR_ACK_PIN 14 // GPIO for sensor module acknowledgment
#define TRIGGER_PULSE_MS 2
#define TRIGGER_INTERVAL_MS 200 // ~5 FPS

const char* ssids[] = { "Gurukul", "dlink-4BB0" };
const char* passwords[] = { "Gurukul@123", "Gurukul@123" };
const int wifiCount = sizeof(ssids) / sizeof(ssids[0]);
WiFiMulti wifiMulti;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

WebServer server(80);

volatile bool ackReceived = false;
volatile bool sensorAckReceived = false;
volatile bool newFrameForProcessing = false;
enum State { IDLE, TRIGGER_SENT, WAITING_ACK, PROCESSING };
State state = IDLE;

// Start streaming in QVGA by default
framesize_t currentResolution = FRAMESIZE_QVGA; // Correct type

void IRAM_ATTR onAckSignal() {
  ackReceived = true;
}

void IRAM_ATTR onSensorAckSignal() {
  sensorAckReceived = true;
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = currentResolution;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  if (psramFound()) {
    config.fb_count = 2;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) { delay(1000); }
  }
}

void setupTriggerPin() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  pinMode(ACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACK_PIN), onAckSignal, FALLING);
  pinMode(SENSOR_ACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_ACK_PIN), onSensorAckSignal, FALLING);
}

void sendTriggerPulse() {
  Serial.println("Sending trigger pulse");
  digitalWrite(TRIGGER_PIN, LOW);
  delay(TRIGGER_PULSE_MS);
  digitalWrite(TRIGGER_PIN, HIGH);
}

void streamHandler() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n";
    server.sendContent(header);
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);
    if (!client.connected()) break;
    delay(50); // Control stream FPS
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passwords[i]);
  }
  Serial.print("Connecting to WiFi...");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  timeClient.begin();

  initCamera();
  setupTriggerPin();

  server.on("/stream", HTTP_GET, streamHandler);
  server.on("/", HTTP_GET, []() {
    String html = "<html><body><h1>ESP32-CAM Master Trigger & Stream</h1>";
    html += "<img src=\"/stream\"></body></html>";
    server.send(200, "text/html", html);
  });

  // Endpoint to set resolution
  server.on("/set_resolution", HTTP_POST, []() {
    String resolution = server.arg("resolution");
    if (resolution == "CIF") currentResolution = FRAMESIZE_CIF;
    else if (resolution == "QVGA") currentResolution = FRAMESIZE_QVGA;
    else if (resolution == "VGA") currentResolution = FRAMESIZE_VGA;
    else if (resolution == "XVGA") currentResolution = FRAMESIZE_XGA;
    else if (resolution == "HD") currentResolution = FRAMESIZE_HD;
    else if (resolution == "SXGA") currentResolution = FRAMESIZE_SXGA;
    else if (resolution == "UXGA") currentResolution = FRAMESIZE_UXGA;
    else {
      server.send(400, "text/plain", "Invalid resolution");
      return;
    }
    // Reinitialize camera with new resolution
    esp_camera_deinit();
    initCamera();
    server.send(200, "text/plain", "Resolution set");
  });

  server.begin();
  Serial.println("HTTP server started");
}

unsigned long lastTriggerTime = 0;
unsigned long lastNtpUpdate = 0;

void loop() {
  server.handleClient();

  // Update NTP only every 5 seconds
  unsigned long now = millis();
  if (now - lastNtpUpdate > 5000) {
    timeClient.update();
    lastNtpUpdate = now;
  }

  // State machine for trigger and ACK
  if (state == IDLE && (now - lastTriggerTime) > TRIGGER_INTERVAL_MS) {
    ackReceived = false;
    sensorAckReceived = false;
    sendTriggerPulse();
    state = TRIGGER_SENT;
    lastTriggerTime = now;
  } else if (state == TRIGGER_SENT && ackReceived && sensorAckReceived) {
    newFrameForProcessing = true;
    state = PROCESSING;
    // Log timestamp for sync
    Serial.printf("Sync record: NTP time: %u\n", timeClient.getEpochTime());
  } else if (state == PROCESSING && !newFrameForProcessing) {
    state = IDLE;
  }

  // Process frame if ready
  if (newFrameForProcessing) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // Your processing code here (object detection, etc.)
      // ...
      Serial.printf("Processing frame at NTP time: %u\n", timeClient.getEpochTime());
      esp_camera_fb_return(fb);
      newFrameForProcessing = false;
    }
  }
}
