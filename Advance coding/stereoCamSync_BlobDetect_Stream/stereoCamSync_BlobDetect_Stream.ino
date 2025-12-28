#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 189);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// SLAVE
#define TRIGGER_PIN 13
#define ACK_PIN 12
#define ACK_PULSE_MS 2

const char* ssids[] = { "Gurukul", "dlink-4BB0" };
const char* passwords[] = { "Gurukul@123", "Gurukul@123" };
const int wifiCount = sizeof(ssids) / sizeof(ssids[0]);
WiFiMulti wifiMulti;

WebServer server(80);

volatile bool triggerReceived = false;
framesize_t currentResolution = FRAMESIZE_QVGA; // Default resolution

void IRAM_ATTR onTrigger() {
  triggerReceived = true;
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

void sendAckPulse() {
  digitalWrite(ACK_PIN, LOW);
  delay(ACK_PULSE_MS);
  digitalWrite(ACK_PIN, HIGH);
  Serial.println("ACK pulse sent");
}

void streamHandler() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  while (client.connected()) {
    if (triggerReceived) {
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
      sendAckPulse();
      triggerReceived = false;
    } else {
      delay(10);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure static IP before connecting to WiFi
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTrigger, FALLING);
  pinMode(ACK_PIN, OUTPUT);
  digitalWrite(ACK_PIN, HIGH);

  for (int i = 0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passwords[i]);
  }
  Serial.print("Connecting to WiFi...");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());

  initCamera();

  server.on("/stream", HTTP_GET, streamHandler);
  server.on("/", HTTP_GET, []() {
    String html = "<html><body><h1>ESP32-CAM Slave Stream with Trigger</h1><img src=\"/stream\"></body></html>";
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

void loop() {
  server.handleClient();
}
