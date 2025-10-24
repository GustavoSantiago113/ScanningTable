#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Stepper.h>

// ----- Wi-Fi Access Point Configuration -----
const char* ssid = "ESP_Motor_Control";   // Wi-Fi name (you’ll see this on your phone)
const char* password = "12345678";        // minimum 8 characters

// ----- Stepper Motor Setup -----
#define STEPS_PER_REV 2048
// GPIO pins for ULN2003 connected to ESP-12F
Stepper stepper(STEPS_PER_REV, 5, 0, 4, 2);  // GPIO5, GPIO0, GPIO4, GPIO2

// ----- Web Server -----
ESP8266WebServer server(80);

// HTML page served to the user
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP8266 Stepper Control</title>
    <style>
      body { font-family: Arial; text-align: center; background: #f4f4f4; padding: 40px; }
      h1 { color: #333; }
      button {
        font-size: 20px; margin: 10px; padding: 15px 30px;
        border: none; border-radius: 10px;
        background: #0078D7; color: white; cursor: pointer;
      }
      button:hover { background: #005FA3; }
    </style>
  </head>
  <body>
    <h1>Stepper Motor Control</h1>
    <button onclick="fetch('/rotate60')">Rotate 60°</button>
    <button onclick="fetch('/rotate360')">Rotate 360°</button>
    <button onclick="fetch('/reverse')">Reverse Direction</button>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// Stepper actions
bool directionCW = true;

void handleRotate60() {
  int steps = STEPS_PER_REV / 6;  // 60 degrees
  stepper.step(directionCW ? steps : -steps);
  server.send(200, "text/plain", "Rotated 60 degrees");
}

void handleRotate360() {
  stepper.step(directionCW ? STEPS_PER_REV : -STEPS_PER_REV);
  server.send(200, "text/plain", "Rotated 360 degrees");
}

void handleReverse() {
  directionCW = !directionCW;
  server.send(200, "text/plain", directionCW ? "Direction: Clockwise" : "Direction: Counterclockwise");
}

void setup() {
  Serial.begin(115200);
  stepper.setSpeed(10); // 10 RPM

  // ---- Start Access Point ----
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.println();
  Serial.println("Access Point started!");
  Serial.print("Network name (SSID): ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Connect and open in browser: http://");
  Serial.println(IP);

  // ---- Define Web Routes ----
  server.on("/", handleRoot);
  server.on("/rotate60", handleRotate60);
  server.on("/rotate360", handleRotate360);
  server.on("/reverse", handleReverse);

  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  server.handleClient();
}