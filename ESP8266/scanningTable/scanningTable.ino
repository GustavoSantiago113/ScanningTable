#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Stepper.h>

// --- Wi-Fi Access Point ---
const char* ssid = "ESP_Motor_WS";
const char* password = "12345678";

#define IN1 5   // D1
#define IN2 4   // D2
#define IN3 14  // D5
#define IN4 12  // D6

#define STEPS_PER_REV 2048
Stepper stepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);

// --- HTTP Server ---
ESP8266WebServer server(80);

// --- Motor control state ---
bool running = false;
bool waitingForContinue = false;
int totalStops = 0;
int currentStop = 0;
long stepsPerStop = 0;

// --- Helpers ---
void sendJson(const String &json, int code = 200) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(code, "application/json", json);
}

void sendOk(const String &msg = "OK") {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", msg);
}

void moveOneSegment() {
  if (!running) return;

  if (currentStop < totalStops) {
    performSteps(stepsPerStop);
    currentStop++;
    Serial.printf("Reached stop %d/%d\n", currentStop, totalStops);

    // Notify app via status polling (waitingForContinue)
    waitingForContinue = true;
  } else {
    running = false;
    waitingForContinue = false;
    Serial.println("Sequence complete.");
  }
}

void performSteps(long nsteps) {
  if (nsteps == 0) return;

  static int lastDir = 0;
  int dir = (nsteps > 0) ? 1 : -1;
  if (dir != lastDir && lastDir != 0) {
    Serial.println("Direction change cooldown");
    delay(200);           // allow coils to de-energize
  }
  lastDir = dir;

  long todo = abs(nsteps);
  for (long i = 0; i < todo; i++) {
    stepper.step(dir);    // one micro-step
    delayMicroseconds(1000); // slower = lower current spike
    yield();              // keep Wi-Fi / watchdog alive
  }
}

// --- HTTP Handlers ---
void handleStatus() {
  String json = "{";
  json += "\"running\":" + String(running ? "true" : "false") + ",";
  json += "\"waitingForContinue\":" + String(waitingForContinue ? "true" : "false") + ",";
  json += "\"currentStop\":" + String(currentStop) + ",";
  json += "\"totalStops\":" + String(totalStops) + "}";
  sendJson(json);
}

void handleStart() {
  int turns = 0;
  int stops = 0;
  if (server.hasArg("turns")) turns = server.arg("turns").toInt();
  if (server.hasArg("stops")) stops = server.arg("stops").toInt();

  if (turns > 0 && stops > 0) {
    long totalSteps = STEPS_PER_REV * turns;
    stepsPerStop = totalSteps / stops;
    totalStops = stops;
    currentStop = 0;
    running = true;
    waitingForContinue = false;
    Serial.printf("START %d turns, %d stops (stepsPerStop=%ld)\n", turns, stops, stepsPerStop);
    sendOk("STARTED");
    moveOneSegment();
  } else {
    server.send(400, "text/plain", "Invalid args. Use POST /start?turns=<n>&stops=<m>");
  }
}

void handleContinue() {
  if (running && waitingForContinue) {
    waitingForContinue = false;
    sendOk("CONTINUING");
    moveOneSegment();
  } else {
    server.send(409, "text/plain", "Not waiting for continue");
  }
}

void handleStop() {
  running = false;
  waitingForContinue = false;
  Serial.println("Manual stop triggered.");
  sendOk("STOPPED");
}

void setup() {
  Serial.begin(115200);
  stepper.setSpeed(8);

  // Start Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println();
  Serial.print("Access Point: "); Serial.println(ssid);
  Serial.print("Password: "); Serial.println(password);
  Serial.print("Connect via HTTP: http://"); Serial.print(IP); Serial.println("/");

  // Start HTTP server
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/continue", HTTP_POST, handleContinue);
  server.on("/stop", HTTP_POST, handleStop);
  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });
  server.begin();
  Serial.println("HTTP server started!");
}

void loop() {
  server.handleClient();
}
