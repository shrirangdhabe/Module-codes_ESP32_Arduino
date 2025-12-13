#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
// Master
#define TRIGGER_PIN 13       // GPIO to send trigger pulse to slave camera
#define ACK_PIN 12           // GPIO to receive acknowledgment from slave camera
#define TRIGGER_PULSE_MS 10  // Trigger pulse duration
#define TRIGGER_INTERVAL_MS 200  // Time between triggers (~5 FPS)

const char* ssids[] = { "Gurukul", "dlink-4BB0" };
const char* passwords[] = { "Gurukul@123", "Gurukul@123" };
const int wifiCount = sizeof(ssids) / sizeof(ssids[0]);
WiFiMulti wifiMulti;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC time, update every 60s

#include <WebServer.h>
WebServer server(80);

bool ackReceived = false;

// Interrupt handler to detect slave camera acknowledgment signal
void IRAM_ATTR onAckSignal() {
  ackReceived = true;
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
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) { delay(1000); }
  }
}

void setupTriggerPin() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH); // inactive high (assuming active low trigger)
  pinMode(ACK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ACK_PIN), onAckSignal, FALLING);
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
    delay(50);
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
  timeClient.update();

  initCamera();
  setupTriggerPin();

  server.on("/stream", HTTP_GET, streamHandler);
  server.on("/", HTTP_GET, []() {
    String html = "<html><body><h1>ESP32-CAM Master Trigger & Stream</h1>";
    html += "<img src=\"/stream\"></body></html>";
    server.send(200, "text/html", html);
  });
  server.begin();
  Serial.println("HTTP server started");
}

unsigned long lastTriggerTime = 0;

void loop() {
  server.handleClient();
  timeClient.update();

  unsigned long now = millis();
  if (now - lastTriggerTime > TRIGGER_INTERVAL_MS) {
    ackReceived = false;
    sendTriggerPulse();
    unsigned long waitStart = millis();
    // Wait up to 50 ms for acknowledgment from slave camera
    while (!ackReceived && (millis() - waitStart) < 50) {
      delay(1);
    }
    if (ackReceived) {
      // Log current NTP timestamp for sync record
      Serial.printf("Trigger ack received, NTP time: %u\n", timeClient.getEpochTime());
    } else {
      Serial.println("Trigger ack NOT received, retrying next interval");
    }
    lastTriggerTime = now;
  }
}
