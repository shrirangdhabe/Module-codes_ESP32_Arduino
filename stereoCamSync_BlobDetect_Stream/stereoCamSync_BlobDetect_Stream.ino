#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiMulti.h>
// SLAVE
#define TRIGGER_PIN 13    // GPIO pin connected from master trigger output
#define ACK_PIN 12        // GPIO pin to send acknowledgment pulse back to master
#define ACK_PULSE_MS 10   // Duration of ACK pulse in milliseconds

// WiFi credentials arrays
const char* ssids[] = { "Gurukul", "dlink-4BB0" };
const char* passwords[] = { "Gurukul@123", "Gurukul@123" };
const int wifiCount = sizeof(ssids) / sizeof(ssids[0]);
WiFiMulti wifiMulti;

#include <WebServer.h>
WebServer server(80);

// Flag to indicate a trigger event received from master
volatile bool triggerReceived = false;

// Interrupt handler for trigger input (falling edge)
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

void sendAckPulse() {
  digitalWrite(ACK_PIN, LOW);   // Active low signal
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

      // Send the frame as MJPEG multipart
      String header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n";
      server.sendContent(header);
      client.write(fb->buf, fb->len);
      server.sendContent("\r\n");

      esp_camera_fb_return(fb);

      // Send acknowledgment pulse to master to confirm capture
      sendAckPulse();

      triggerReceived = false;  // Reset trigger flag after processing
    } else {
      delay(10);  // Wait for trigger event
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure trigger pin as input with pullup and interrupt on falling edge
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTrigger, FALLING);

  // Configure ACK pin as output and set high (inactive)
  pinMode(ACK_PIN, OUTPUT);
  digitalWrite(ACK_PIN, HIGH);

  // Connect to multiple WiFi networks with WiFiMulti
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

  initCamera();

  // Setup web server routes
  server.on("/stream", HTTP_GET, streamHandler);
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<html><body><h1>ESP32-CAM Slave Stream with Trigger</h1><img src=\"/stream\"></body></html>");
  });
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
