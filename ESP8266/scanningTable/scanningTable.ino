#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
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

// --- WebSocket Server ---
WebSocketsServer webSocket = WebSocketsServer(81);  // Port 81 for WS

// --- Motor control state ---
bool running = false;
bool waitingForContinue = false;
int totalStops = 0;
int currentStop = 0;
long stepsPerStop = 0;

void moveOneSegment() {
  if (!running) return;

  if (currentStop < totalStops) {
    performSteps(stepsPerStop);
    currentStop++;
    Serial.printf("Reached stop %d/%d\n", currentStop, totalStops);

    // Notify app that we stopped
    String msg = "STOP " + String(currentStop) + "/" + String(totalStops);
    webSocket.broadcastTXT(msg);

    // Wait for CONTINUE command from the app
    waitingForContinue = true;
  } else {
    running = false;
    waitingForContinue = false;
    webSocket.broadcastTXT("DONE");
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

// --- Handle incoming WebSocket messages ---
void handleMessage(uint8_t num, String payload) {
  payload.trim();
  Serial.printf("[WS %u] Received: %s\n", num, payload.c_str());

  if (payload.startsWith("START")) {
    // Format: START <turns> <stops>
    int turns, stops;
    int n = sscanf(payload.c_str(), "START %d %d", &turns, &stops);
    if (n == 2 && turns > 0 && stops > 0) {
      long totalSteps = STEPS_PER_REV * turns;
      stepsPerStop = totalSteps / stops;
      totalStops = stops;
      currentStop = 0;
      running = true;
      waitingForContinue = false;

      String startedMsg = "STARTED " + String(turns) + " turns, " + String(stops) + " stops";
      webSocket.broadcastTXT(startedMsg);
      moveOneSegment();
    } else {
      webSocket.sendTXT(num, "ERROR Invalid START command. Use: START <turns> <stops>");
    }
  }

  else if (payload.equalsIgnoreCase("CONTINUE")) {
    if (running && waitingForContinue) {
      waitingForContinue = false;
      moveOneSegment();
    } else {
      webSocket.sendTXT(num, "IGNORED CONTINUE (not waiting)");
    }
  }

  else if (payload.equalsIgnoreCase("STOP")) {
    running = false;
    waitingForContinue = false;
    webSocket.broadcastTXT("STOPPED");
    Serial.println("Manual stop triggered.");
  }

  else {
    webSocket.sendTXT(num, "UNKNOWN COMMAND");
  }
}

// --- WebSocket event callback ---
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("Client %u connected from %s\n", num, ip.toString().c_str());
      webSocket.sendTXT(num, "CONNECTED to ESP_Motor_WS");
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", num);
      break;
    case WStype_TEXT:
      handleMessage(num, String((char*)payload));
      break;
    default:
      break;
  }
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
  Serial.print("Connect via: ws://"); Serial.print(IP); Serial.println(":81");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("WebSocket server started!");
}

void loop() {
  webSocket.loop();
}
