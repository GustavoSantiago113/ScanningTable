#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>

// --- Wi-Fi Access Point ---
const char* ssid = "ESP_Motor_WS";
const char* password = "12345678";

// NEMA17 driver (A4988/DRV8825) pin definitions for step+dir
#define STEP_PIN 14 // D5
#define DIR_PIN  12 // D6
#define ENABLE_PIN 4 // D2 (LOW = enabled)

// Steps per revolution for the stepper motor (full steps)
#define STEPS_PER_REV 200

// Use DRIVER mode (step + dir)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Motion tuning ---
// AccelStepper motion tuning
const float MAX_SPEED = 600.0;     // steps/sec
const float ACCEL = 175.0;         // steps/sec^2

// --- HTTP Server ---
WebServer server(80);

// --- Motor control state ---
bool running = false;
bool waitingForContinue = false;
int totalStops = 0;
int currentStop = 0;
long stepsPerStop = 0;

// --- Rotation (full turn) state ---
bool performingRotation = false;
long rotationStepsRemaining = 0;
int rotationDir_global = 1;
int lastDir = 0; // global last direction (1 or -1)

// --- Helpers ---
void sendJson(const String &json, int code = 200) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(code, "application/json", json);
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

void sendOk(const String &msg = "OK") {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", msg);
}

// Move one segment (blocking but keeps server responsive)
void moveOneSegment() {
  if (!running) return;

  if (currentStop < totalStops) {
    long target = stepper.currentPosition() + stepsPerStop;
    stepper.moveTo(target);

    while (stepper.distanceToGo() != 0 && running) {
      stepper.run();
      server.handleClient(); // keep server responsive during movement
    }

    if (running) {
      currentStop++;
      Serial.printf("Reached stop %d/%d\n", currentStop, totalStops);
      waitingForContinue = (currentStop < totalStops);
      if (!waitingForContinue) {
        running = false;
        Serial.println("Sequence complete.");
      }
    }
  } else {
    running = false;
    waitingForContinue = false;
    Serial.println("Sequence complete.");
  }
}

// Called repeatedly from loop() to perform an in-progress full rotation
void processRotation() {
  if (!performingRotation || rotationStepsRemaining <= 0) return;
  if (!running) {
    performingRotation = false;
    waitingForContinue = false;
    Serial.println("Rotation interrupted by STOP.");
    return;
  }

  // rotation handled by AccelStepper via moveTo + run in loop()
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  } else {
    performingRotation = false;
    running = false;
    waitingForContinue = false;
    Serial.println("Full rotation complete.");
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
    long totalSteps = (long)STEPS_PER_REV * (long)turns;
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
  // Stop AccelStepper movement
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition());
  sendOk("STOPPED");
}

void handleRotate() {
  if (running || performingRotation) {
    server.send(409, "text/plain", "Already running");
    return;
  }

  int turns = 1;
  if (server.hasArg("turns")) turns = server.arg("turns").toInt();
  if (turns == 0) {
    server.send(400, "text/plain", "Invalid turns. Use POST /rotate?turns=<n>");
    return;
  }

  long totalSteps = (long)STEPS_PER_REV * (long)abs(turns);
  rotationDir_global = (turns > 0) ? 1 : -1;
  long target = stepper.currentPosition() + rotationDir_global * totalSteps;
  stepper.moveTo(target);

  performingRotation = true;
  running = true;
  waitingForContinue = false;

  Serial.printf("ROTATE %d turns (%ld steps)\n", turns, totalSteps);
  sendOk("ROTATING");
}

void setup() {
  Serial.begin(115200);
  // Setup AccelStepper parameters
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // enable driver
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCEL);
  stepper.setCurrentPosition(0);

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
  server.on("/rotate", HTTP_POST, handleRotate);
  server.on("/continue", HTTP_POST, handleContinue);
  server.on("/stop", HTTP_POST, handleStop);
  // OPTIONS preflight handlers for CORS
  server.on("/start", HTTP_OPTIONS, handleOptions);
  server.on("/rotate", HTTP_OPTIONS, handleOptions);
  server.on("/continue", HTTP_OPTIONS, handleOptions);
  server.on("/stop", HTTP_OPTIONS, handleOptions);
  server.on("/status", HTTP_OPTIONS, handleOptions);
  server.onNotFound([](){ server.send(404, "text/plain", "Not Found"); });
  server.begin();
  Serial.println("HTTP server started!");
}

void loop() {
  server.handleClient();
  // If performing a long rotation, progress it non-blocking
  processRotation();
}
