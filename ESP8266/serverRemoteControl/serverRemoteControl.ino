#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Stepper.h>

// ---- WiFi Configuration ----
const char* ssid = "Gustavo";
const char* password = "Biossistemas123";

// ---- Stepper Motor Setup ----
#define STEPS_PER_REV 2048
Stepper stepper(STEPS_PER_REV, 5, 0, 4, 2);  // GPIO pins for ULN2003

// ---- Web Server ----
ESP8266WebServer server(80);

void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP8266 Motor Control</title>
    <style>
      body { font-family: sans-serif; text-align: center; background: #f2f2f2; }
      button { font-size: 20px; margin: 10px; padding: 15px 30px; border-radius: 10px; border: none; background: #0078D7; color: white; }
      button:hover { background: #005FA3; }
    </style>
  </head>
  <body>
    <h1>Stepper Motor Control</h1>
    <button onclick="fetch('/rotate60')">Rotate 60°</button>
    <button onclick="fetch('/rotate360')">Rotate 360°</button>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleRotate60() {
  int steps = STEPS_PER_REV / 6; // 60°
  stepper.step(steps);
  server.send(200, "text/plain", "Rotated 60 degrees");
}

void handleRotate360() {
  stepper.step(STEPS_PER_REV);
  server.send(200, "text/plain", "Rotated 360 degrees");
}

void setup() {
  Serial.begin(115200);
  stepper.setSpeed(10);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Web routes
  server.on("/", handleRoot);
  server.on("/rotate60", handleRotate60);
  server.on("/rotate360", handleRotate360);

  server.begin();
  Serial.println("Server started!");
}

void loop() {
  server.handleClient();
}
