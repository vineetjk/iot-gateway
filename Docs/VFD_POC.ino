#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define BTN_UP    15
#define BTN_DOWN   2
#define BTN_OK     4
#define RXD2      16
#define TXD2      17
#define LED_G     18
#define LED_R     19

// Potentiometer on GPIO34
#define POT_PIN 34
#define POT_CHANGE_THRESHOLD 25
#define POT_DISPLAY_TIMEOUT 3000

// ============================================================
// VFD VARIABLES
// ============================================================
float vfd_freq    = 50.0;
float vfd_bus_v   = 312.0;
bool  vfd_running = true;
int   vfd_rpm     = 1500;
float vfd_amps    = 2.4;

// ============================================================
// STATE MACHINE
// ============================================================
enum State {
  DASHBOARD, MENU, EDIT_FREQ, SIG_VIEW,
  FUN_MENU, SNAKE, BRICKS, SET_INTERVAL, POT_VOLT
};
State currentState = DASHBOARD;

int menuIndex  = 0;
int funIndex   = 0;
const int TOTAL_MENU_ITEMS = 5;
String menuItems[] = { "SET FREQUENCY", "RUN/STOP MODE", "GSM STATUS", "SET INTERVAL", "FUN ZONE" };
String funItems[]  = { "SNAKE GAME", "BREAK BRICKS", "< BACK" };

// ============================================================
// INTERVAL
// ============================================================
const int TOTAL_INTERVALS = 5;
const unsigned long intervalValues[] = { 1000, 5000, 10000, 30000, 60000, 300000 };
const char* intervalLabels[]         = { "1s", "5s", "10s", "30s", "1m", "5m" };
int selectedIntervalIndex = 1;
int intervalMenuIndex     = 1;
unsigned long lastAutoPublish = 0;
bool autoPublishEnabled = true;

// ============================================================
// TIMERS
// ============================================================
unsigned long lastInteraction = 0;
int           dashCycle       = 0;
unsigned long lastDashUpdate  = 0;  
unsigned long prevGameTick    = 0;
unsigned long potLastChanged  = 0;

// Pot median filter
int  potSamples[7]   = {0};
int  potSampleIdx    = 0;
int  lastPotFiltered = 0;
int  potSmoothed     = 0;


// ============================================================
// GSM / MQTT
// ============================================================
const char* mqtt_broker    = "broker.emqx.io";
const char* mqtt_topic     = "delta/vfd";
const char* mqtt_sub_topic = "delta/vfd/set";
String gsm_signal = "0", gsm_oper = "Searching...";
bool mqtt_connected = false;

// Non-blocking GSM receive buffer
String gsmBuffer = "";

// ============================================================
// GAMES
// ============================================================
int   snakeX[30], snakeY[30], snakeLen, snakeDir;
int   foodX, foodY, paddleX = 50;
float ballX, ballY, ballDX, ballDY;
bool  bricks[3][6];

// ============================================================
// LOGO BITMAP 50x46
// ============================================================
static const unsigned char PROGMEM logo_bmp[] = {
  0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
  0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
  0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00,
  0x01, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x01,
  0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x03, 0xFF,
  0x80, 0xF0, 0x00, 0x00, 0x00, 0x03, 0xFE, 0x00,
  0x70, 0x00, 0x00, 0x00, 0x07, 0xF8, 0x3E, 0x38,
  0x00, 0x00, 0x00, 0x07, 0xE1, 0xFF, 0x38, 0x00,
  0x00, 0x00, 0x0F, 0xC3, 0xFF, 0x9C, 0x00, 0x00,
  0x00, 0x0F, 0x8F, 0xFF, 0x9C, 0x00, 0x00, 0x00,
  0x1F, 0x1F, 0xFF, 0x9E, 0x00, 0x00, 0x00, 0x1E,
  0x3F, 0xFF, 0x9F, 0x00, 0x00, 0x00, 0x3E, 0x7F,
  0xFF, 0x9F, 0x00, 0x00, 0x00, 0x7C, 0x7F, 0xE0,
  0x1F, 0x80, 0x00, 0x00, 0x78, 0xFF, 0xC0, 0x3F,
  0x80, 0x00, 0x00, 0xF9, 0xFF, 0x80, 0x1F, 0xC0,
  0x00, 0x00, 0xF1, 0xFF, 0x80, 0x1F, 0xC0, 0x00,
  0x01, 0xF3, 0xFF, 0x00, 0x1F, 0xE0, 0x00, 0x01,
  0xF3, 0xFF, 0x00, 0x0F, 0xE0, 0x00, 0x03, 0xE7,
  0xFF, 0x00, 0x0F, 0xF0, 0x00, 0x03, 0xE7, 0xFF,
  0x00, 0x1F, 0xF0, 0x00, 0x07, 0xE7, 0xFF, 0x80,
  0x1F, 0xF8, 0x00, 0x07, 0xE7, 0xFF, 0x80, 0x1F,
  0xFC, 0x00, 0x0F, 0xE3, 0xFF, 0x00, 0x3F, 0xFC,
  0x00, 0x1F, 0xF3, 0xFE, 0x00, 0x7F, 0xFE, 0x00,
  0x1F, 0xF0, 0xF8, 0x79, 0xFF, 0xFE, 0x00, 0x3F,
  0xF8, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x3F, 0xFE,
  0x03, 0xFF, 0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xC0,
};



// ============================================================
// POT MEDIAN FILTER (only compiled if using pot)
// ============================================================

int medianFilter(int newVal) {
  potSamples[potSampleIdx % 7] = newVal;
  potSampleIdx++;
  // Copy and sort
  int tmp[7];
  for (int i = 0; i < 7; i++) tmp[i] = potSamples[i];
  for (int i = 0; i < 6; i++)
    for (int j = i+1; j < 7; j++)
      if (tmp[j] < tmp[i]) { int t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[3]; // median
}


// ============================================================
// LED
// ============================================================
void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;

  if (!mqtt_connected) {
    // YELLOW blinking
    if (millis() - lastBlink >= 400) {
      lastBlink  = millis();
      blinkState = !blinkState;
    }
    digitalWrite(LED_R, blinkState ? LOW : HIGH);
    digitalWrite(LED_G, blinkState ? LOW : HIGH);
    return;
  }
  if (vfd_running) {
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, HIGH);
  } else {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
  }
}

// ============================================================
// NON-BLOCKING GSM READER
// Accumulates chars into gsmBuffer, processes complete lines
// ============================================================
void checkGSMForCommands() {
  // Drain available bytes into buffer — zero blocking
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      gsmBuffer.trim();
      if (gsmBuffer.length() > 0) {
        processGSMLine(gsmBuffer);
      }
      gsmBuffer = "";
    } else {
      gsmBuffer += c;
      // Safety: prevent buffer overflow
      if (gsmBuffer.length() > 256) gsmBuffer = "";
    }
  }
}

void processGSMLine(String& line) {
  Serial.println("GSM< " + line);

  // MQTT received message
  if (line.indexOf("+QMTRECV:") != -1) {
    Serial.println("Cloud Cmd: " + line);

    if (line.indexOf("F:") != -1) {
      int idx = line.indexOf("F:");
      float newFreq = line.substring(idx + 2).toFloat();
      if (newFreq >= 1.0 && newFreq <= 100.0) vfd_freq = newFreq;
    }
    if (line.indexOf("S:RUN")  != -1) { vfd_running = true;  Serial.println("→ RUN"); }
    if (line.indexOf("S:STOP") != -1) { vfd_running = false; Serial.println("→ STOP"); }
    if (line.indexOf("I:") != -1) {
      int idx = line.indexOf("I:");
      unsigned long req = line.substring(idx + 2).toInt() * 1000UL;
      for (int i = 0; i < TOTAL_INTERVALS; i++)
        if (intervalValues[i] == req) { selectedIntervalIndex = i; break; }
    }
  }

  // Modem reports MQTT connection dropped
  if (line.indexOf("+QMTSTAT:") != -1) {
    mqtt_connected = false;
    Serial.println("MQTT dropped!");
  }

  // Modem confirms publish sent (we don't block waiting for this anymore)
  // Just use it to confirm connection still alive
  if (line.indexOf("+QMTPUB:") != -1) {
    mqtt_connected = true;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(BTN_UP,   INPUT_PULLDOWN);
  pinMode(BTN_DOWN, INPUT_PULLDOWN);
  pinMode(BTN_OK,   INPUT_PULLDOWN);
  pinMode(LED_G,    OUTPUT);
  pinMode(LED_R,    OUTPUT);

  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_R, HIGH);


  pinMode(POT_PIN, INPUT);
  // Warm-up: discard first 50 ADC readings (ESP32 ADC settles slowly)
  for (int i = 0; i < 50; i++) analogRead(POT_PIN);
  // Stable baseline: median of 32 samples
  long sum = 0;
  for (int i = 0; i < 32; i++) { sum += analogRead(POT_PIN); delay(2); }
  int baseline = sum / 32;
  for (int i = 0; i < 7; i++) potSamples[i] = baseline;
  lastPotFiltered = baseline;
  vfd_bus_v = map(baseline, 0, 4095, 280, 420);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
    for (;;);

  showSplashScreen();
  initGSM();
  connectMQTT();

  lastInteraction = millis();
  lastAutoPublish = millis();
}

// ============================================================
// LOOP — kept as lean as possible, zero blocking calls
// ============================================================
void loop() {
    static unsigned long lastPotSample = 0;
    // Sample pot every 20ms (50Hz) — not every loop tick
    if (millis() - lastPotSample >= 20) {
      lastPotSample = millis();
      int filtered = medianFilter(analogRead(POT_PIN));
      if (abs(filtered - lastPotFiltered) > POT_CHANGE_THRESHOLD) {
        lastPotFiltered = filtered;
        vfd_bus_v = map(filtered, 0, 4095, 280, 420);
        potLastChanged = millis();
        if (currentState != SNAKE && currentState != BRICKS)
          currentState = POT_VOLT;
      }
    }

  // Auto-exit POT_VOLT after timeout
  if (currentState == POT_VOLT &&
      millis() - potLastChanged > POT_DISPLAY_TIMEOUT) {
    currentState = DASHBOARD;
  }
  checkGSMForCommands();
  simulateVFD();
  updateStatusLED();
  // ---- Auto-publish (fire and forget, no ACK wait) ----
  if (mqtt_connected && autoPublishEnabled) {
    if (millis() - lastAutoPublish >= intervalValues[selectedIntervalIndex]) {
      lastAutoPublish = millis();
      publishMQTTData();
    }
  }

  // ---- Idle timeout ----
  if (millis() - lastInteraction > 5000
      && currentState != DASHBOARD
      && currentState != SNAKE
      && currentState != BRICKS
      && currentState != POT_VOLT) {
    currentState = DASHBOARD;
  }

  // ---- State router ----
  if (currentState == DASHBOARD) {
    runDashboard();
    if (digitalRead(BTN_UP) || digitalRead(BTN_DOWN) || digitalRead(BTN_OK)) {
      currentState = MENU;
      lastInteraction = millis();
      delay(200);
    }
  } else if (currentState == POT_VOLT) {
    runPotVoltPage();
  } else if (currentState == SNAKE) {
    runSnake();
  } else if (currentState == BRICKS) {
    runBricks();
  } else {
    handleButtons();
  }
}

// ============================================================
// PUBLISH — fire and forget, NO blocking ACK wait
// ============================================================
void publishMQTTData() {
  Serial2.print("AT+QMTPUB=0,0,0,0,\"");
  Serial2.print(mqtt_topic);
  Serial2.println("\"");

  // Wait for '>' prompt — max 500ms (was 2000ms before)
  unsigned long t = millis();
  bool gotPrompt = false;
  while (millis() - t < 500) {
    if (Serial2.available() && Serial2.read() == '>') {
      gotPrompt = true; break;
    }
  }

  if (!gotPrompt) {
    mqtt_connected = false;
    Serial.println("Pub: no prompt, marking disconnected");
    return;
  }

  String payload = "{\"f\":"  + String(vfd_freq, 1)
                 + ",\"s\":" + String(vfd_running ? 1 : 0)
                 + ",\"v\":" + String((int)vfd_bus_v)
                 + ",\"r\":" + String(vfd_rpm)
                 + ",\"a\":" + String(vfd_amps, 1) + "}";

  Serial2.print(payload);
  Serial2.write(0x1A);  // Ctrl+Z — triggers send

  // processGSMLine() will catch "+QMTPUB:" asynchronously next loop
  Serial.println("Pub fired: " + payload);
}

// ============================================================
// INIT GSM — faster, removed unnecessary delays
// ============================================================
void initGSM() {
  sendAT("AT",                          500);
  sendAT("ATE0",                        500);  // turn off echo — reduces noise in buffer
  sendAT("AT+QMTCFG=\"version\",0,4",  500);
}

// ============================================================
// CONNECT MQTT — with proper timeout, no extra delays
// ============================================================
void connectMQTT() {
  mqtt_connected = false;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 16);
  display.println("Connecting 4G...");
  display.setCursor(10, 28);
  display.println("Please wait...");
  display.display();

  // Activate PDP context
  sendAT("AT+QIDEACT=1",                              2000);
  sendAT("AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",0", 500);

  // Wait for network attach before activating
  String creg = sendAT("AT+CREG?", 1000);
  display.setCursor(10, 40);
  display.print("Network: ");
  display.println(creg.indexOf(",1") != -1 || creg.indexOf(",5") != -1 ? "OK" : "Wait..");
  display.display();

  sendAT("AT+QIACT=1", 6000);   // was 8000ms — 6s is enough for most networks

  // Open MQTT broker connection
  Serial2.print("AT+QMTOPEN=0,\"");
  Serial2.print(mqtt_broker);
  Serial2.println("\",1883");

  // Wait for +QMTOPEN: 0,0 — up to 8 seconds
  unsigned long t = millis();
  bool opened = false;
  while (millis() - t < 8000) {
    updateStatusLED();           // keep yellow blinking
    checkGSMForCommands();       // drain buffer
    if (gsmBuffer.indexOf("+QMTOPEN: 0,0") != -1 || Serial2.find("+QMTOPEN: 0,0")) {
      opened = true; break;
    }
  }

  if (!opened) {
    display.clearDisplay();
    display.setCursor(4, 20);  display.println("4G Connect FAILED");
    display.setCursor(4, 34);  display.println("Check SIM/Antenna");
    display.display();
    delay(2000);
    return;
  }

  // MQTT CONNECT
  String r = sendAT("AT+QMTCONN=0,\"VFD_" + String(random(1000, 9999)) + "\"", 4000);

  // Subscribe to command topic
  sendAT("AT+QMTSUB=0,1,\"" + String(mqtt_sub_topic) + "\",0", 2000);

  // Check result — some modems return 0,0,0 others just OK
  if (r.indexOf("0,0,0") != -1 || r.indexOf("OK") != -1) {
    mqtt_connected = true;
    display.clearDisplay();
    display.setCursor(10, 25); display.setTextColor(WHITE);
    display.println("Connected!");
    display.setCursor(10, 38); display.println(mqtt_broker);
    display.display();
    delay(800);
  } else {
    display.clearDisplay();
    display.setCursor(4, 25); display.println("MQTT Auth FAILED");
    display.display();
    delay(2000);
  }
}

// ============================================================
// SEND AT — used only during init/connect, NOT in main loop
// ============================================================
String sendAT(String cmd, int timeout) {
  while (Serial2.available()) Serial2.read();  // flush
  Serial2.println(cmd);
  unsigned long t = millis();
  String res = "";
  while (millis() - t < timeout) {
    if (Serial2.available()) res += (char)Serial2.read();
  }
  Serial.print(">> "); Serial.println(cmd);
  Serial.print("<< "); Serial.println(res);
  return res;
}

// ============================================================
// GSM INFO
// ============================================================
void updateGSMInfo() {
  String cops = sendAT("AT+COPS?", 1000);
  int f = cops.indexOf('"'), s = cops.indexOf('"', f + 1);
  gsm_oper   = (f != -1) ? cops.substring(f + 1, s) : "No Service";
  String csq = sendAT("AT+CSQ", 500);
  int col = csq.indexOf(':'), com = csq.indexOf(',');
  gsm_signal = (col != -1) ? csq.substring(col + 2, com) : "0";
}

// ============================================================
// VFD SIMULATION
// ============================================================
void simulateVFD() {
  if (vfd_running) {
    vfd_rpm  = (int)(vfd_freq * 30);
    vfd_amps = (vfd_freq / 50.0) * 4.2 + (random(-5, 5) / 10.0);
  } else {
    vfd_rpm  = 0;
    vfd_amps = 0.0;
  }
}

// ============================================================
// DASHBOARD
// ============================================================
void runDashboard() {
  if (millis() - lastDashUpdate > 2000) {
    lastDashUpdate = millis();
    dashCycle = (dashCycle + 1) % 5;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("VFD MONITOR");
    display.drawFastHLine(0, 10, 128, WHITE);
    display.setCursor(80, 0);
    display.print("~"); display.print(intervalLabels[selectedIntervalIndex]);
    if (mqtt_connected) display.fillCircle(124, 4, 3, WHITE);
    else                display.drawCircle(124, 4, 3, WHITE);
    display.setCursor(0, 14);
    switch (dashCycle) {
      case 0:
        display.println("FREQUENCY");
        display.setTextSize(2); display.setCursor(10, 36);
        display.print(vfd_freq, 1); display.print(" Hz"); break;
      case 1:
        display.println("BUS VOLTAGE");
        display.setTextSize(2); display.setCursor(10, 36);
        display.print((int)vfd_bus_v); display.print(" V"); break;
      case 2:
        display.println("MOTOR SPEED");
        display.setTextSize(2); display.setCursor(10, 36);
        display.print(vfd_rpm); display.print(" RPM"); break;
      case 3:
        display.println("DRIVE STATUS");
        display.setTextSize(2); display.setCursor(10, 36);
        display.print(vfd_running ? "RUNNING" : "STOPPED"); break;
      case 4:
        display.println("LOAD CURRENT");
        display.setTextSize(2); display.setCursor(10, 36);
        display.print(vfd_amps, 1); display.print(" A"); break;
    }
    display.display();
  }
}

// ============================================================
// POT VOLTAGE PAGE
// ============================================================
void runPotVoltPage() {
  static unsigned long lastRefresh = 0;
  if (millis() - lastRefresh < 80) return;
  lastRefresh = millis();

  // Re-read pot
  int filtered = medianFilter(analogRead(POT_PIN));
  vfd_bus_v = map(filtered, 0, 4095, 280, 420);

  display.clearDisplay();

  // Header
  display.fillRect(0, 0, 128, 13, WHITE);
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(22, 3);
  display.print("BUS VOLTAGE");

  // Large number
  display.setTextColor(WHITE);
  display.setTextSize(3);
  String vStr = String((int)vfd_bus_v);
  int numW = vStr.length() * 18;
  display.setCursor((60 - numW) / 2 + 4, 18);
  display.print(vStr);
  display.setTextSize(2);
  display.setCursor(72, 24);
  display.print("V");

  // Animated label
  static int dotCount = 0;
  static unsigned long lastDot = 0;
  if (millis() - lastDot > 350) { dotCount = (dotCount + 1) % 4; lastDot = millis(); }
  display.setTextSize(1);
  display.setCursor(4, 42);
  display.print("Adjusting");

  for (int i = 0; i < dotCount; i++) display.print(".");

  // Bar graph
  int barMaxW = 120;
  int barW = constrain(map((int)vfd_bus_v, 280, 420, 0, barMaxW), 0, barMaxW);
  display.drawRect(4, 52, barMaxW, 8, WHITE);
  display.fillRect(4, 52, barW,    8, WHITE);
  display.setTextSize(1);
  display.setCursor(4,  61); display.print("280");
  display.setCursor(100, 61); display.print("420");

  // Countdown strip
  unsigned long elapsed  = millis() - potLastChanged;
  int countdownW = constrain(map(elapsed, 0, POT_DISPLAY_TIMEOUT, barMaxW, 0), 0, barMaxW);
  display.fillRect(4, 13, countdownW, 2, WHITE);

  display.drawFastHLine(0, 40, 128, WHITE);
  display.display();

  // Any button → back to dashboard
  if (digitalRead(BTN_UP) || digitalRead(BTN_DOWN) || digitalRead(BTN_OK)) {
    currentState = DASHBOARD;
    lastInteraction = millis();
    delay(200);
  }
}

// ============================================================
// MENU DISPLAY
// ============================================================
void updateDisplay() {
  display.clearDisplay();
  display.fillRect(0, 0, 128, 14, WHITE);
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(4, 3);
  if      (currentState == FUN_MENU)     display.print("FUN ZONE");
  else if (currentState == SET_INTERVAL) display.print("SET INTERVAL");
  else                                   display.print("VFD SETTINGS");
  display.setTextColor(WHITE);

  if (currentState == MENU) {
    for (int i = 0; i < TOTAL_MENU_ITEMS; i++) {
      int yPos = 18 + (i * 9);
      if (i == menuIndex) { display.fillRect(0, yPos-1, 128, 9, WHITE); display.setTextColor(BLACK); }
      else display.setTextColor(WHITE);
      display.setCursor(5, yPos);
      display.print(menuItems[i]);
      if (i == 1) { display.print(": "); display.print(vfd_running ? "RUN" : "STOP"); }
      if (i == 3) { display.print(": "); display.print(intervalLabels[selectedIntervalIndex]); }
    }
  } else if (currentState == SET_INTERVAL) {
    for (int i = 0; i < TOTAL_INTERVALS; i++) {
      int yPos = 18 + (i * 9);
      if (i == intervalMenuIndex) { display.fillRect(0, yPos-1, 128, 9, WHITE); display.setTextColor(BLACK); }
      else display.setTextColor(WHITE);
      display.setCursor(10, yPos); display.print(intervalLabels[i]);
      if (i == selectedIntervalIndex) { display.setCursor(85, yPos); display.print("<ACTIVE"); }
    }
  } else if (currentState == FUN_MENU) {
    for (int i = 0; i < 3; i++) {
      int yPos = 25 + (i * 12);
      if (i == funIndex) { display.fillRect(0, yPos-1, 128, 11, WHITE); display.setTextColor(BLACK); }
      else display.setTextColor(WHITE);
      display.setCursor(10, yPos); display.print(funItems[i]);
    }
  } else if (currentState == EDIT_FREQ) {
    display.setCursor(10, 22); display.print("Adjust Frequency:");
    display.setTextSize(2); display.setCursor(30, 40);
    display.print(vfd_freq, 1); display.print("Hz");
  } else if (currentState == SIG_VIEW) {
    display.setCursor(5, 16); display.print("Operator: "); display.println(gsm_oper);
    display.setCursor(5, 30); display.print("Signal:   "); display.println(gsm_signal);
    display.setCursor(5, 44); display.print("MQTT: ");
    display.print(mqtt_connected ? "CONNECTED" : "OFFLINE");
    int bars = map(gsm_signal.toInt(), 0, 31, 0, 5);
    for (int i = 0; i < 5; i++) {
      if (i < bars) display.fillRect(80+(i*6), 56-(i*3), 4, (i*3)+3, WHITE);
      else          display.drawRect(80+(i*6), 56-(i*3), 4, (i*3)+3, WHITE);
    }
  }
  display.display();
}

// ============================================================
// BUTTONS
// ============================================================
void handleButtons() {
  if (digitalRead(BTN_UP)) {
    lastInteraction = millis();
    if      (currentState == MENU)         menuIndex = (menuIndex - 1 + TOTAL_MENU_ITEMS) % TOTAL_MENU_ITEMS;
    else if (currentState == FUN_MENU)     funIndex  = (funIndex  - 1 + 3) % 3;
    else if (currentState == EDIT_FREQ)    vfd_freq  = constrain(vfd_freq + 0.5, 1.0, 100.0);
    else if (currentState == SET_INTERVAL) intervalMenuIndex = (intervalMenuIndex - 1 + TOTAL_INTERVALS) % TOTAL_INTERVALS;
    updateDisplay();
    while (digitalRead(BTN_UP)); delay(100);
  }
  if (digitalRead(BTN_DOWN)) {
    lastInteraction = millis();
    if      (currentState == MENU)         menuIndex = (menuIndex + 1) % TOTAL_MENU_ITEMS;
    else if (currentState == FUN_MENU)     funIndex  = (funIndex  + 1) % 3;
    else if (currentState == EDIT_FREQ)    vfd_freq  = constrain(vfd_freq - 0.5, 1.0, 100.0);
    else if (currentState == SET_INTERVAL) intervalMenuIndex = (intervalMenuIndex + 1) % TOTAL_INTERVALS;
    updateDisplay();
    while (digitalRead(BTN_DOWN)); delay(100);
  }
  if (digitalRead(BTN_OK)) {
    lastInteraction = millis();
    if (currentState == MENU) {
      if      (menuIndex == 0) currentState = EDIT_FREQ;
      else if (menuIndex == 1) vfd_running = !vfd_running;
      else if (menuIndex == 2) { currentState = SIG_VIEW; updateGSMInfo(); }
      else if (menuIndex == 3) { intervalMenuIndex = selectedIntervalIndex; currentState = SET_INTERVAL; }
      else if (menuIndex == 4) currentState = FUN_MENU;
    } else if (currentState == SET_INTERVAL) {
      selectedIntervalIndex = intervalMenuIndex;
      lastAutoPublish = millis();
      currentState = MENU;
    } else if (currentState == FUN_MENU) {
      if      (funIndex == 0) { initSnake();  currentState = SNAKE; }
      else if (funIndex == 1) { initBricks(); currentState = BRICKS; }
      else                      currentState = MENU;
    } else currentState = MENU;
    updateDisplay();
    while (digitalRead(BTN_OK)); delay(150);
  }
}

// ============================================================
// SPLASH
// ============================================================
void showSplashScreen() {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, WHITE);
  display.drawBitmap(2, 9, logo_bmp, 50, 46, WHITE);
  display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(58, 14); display.println("DELTA");
  display.setTextSize(1);
  display.setCursor(58, 34); display.println("SOLAR VFD 2");
  display.setCursor(58, 46); display.println("v2.0");
  display.drawFastVLine(55, 9, 46, WHITE);
  display.display();
  delay(3000);
}

// ============================================================
// GAMES
// ============================================================
bool checkGameExit() {
  if (digitalRead(BTN_OK) == HIGH) {
    unsigned long st = millis();
    while (digitalRead(BTN_OK) == HIGH)
      if (millis() - st > 1000) { currentState = FUN_MENU; return true; }
  }
  return false;
}

void initSnake() {
  snakeLen = 5; snakeDir = 1;
  for (int i = 0; i < snakeLen; i++) { snakeX[i] = 10-i; snakeY[i] = 10; }
  foodX = random(2, 23); foodY = random(2, 10);
}

void runSnake() {
  if (checkGameExit()) return;
  if (millis() - prevGameTick > 120) {
    prevGameTick = millis();
    if (digitalRead(BTN_UP))   snakeDir = (snakeDir + 1) % 4;
    if (digitalRead(BTN_DOWN)) snakeDir = (snakeDir + 3) % 4;
    for (int i = snakeLen-1; i > 0; i--) { snakeX[i]=snakeX[i-1]; snakeY[i]=snakeY[i-1]; }
    if      (snakeDir == 0) snakeY[0]--;
    else if (snakeDir == 1) snakeX[0]++;
    else if (snakeDir == 2) snakeY[0]++;
    else                    snakeX[0]--;
    if (snakeX[0]==foodX && snakeY[0]==foodY) { snakeLen++; foodX=random(2,23); foodY=random(2,10); }
    if (snakeX[0]<0||snakeX[0]>25||snakeY[0]<0||snakeY[0]>12) initSnake();
    display.clearDisplay();
    display.drawRect(0,0,128,64,WHITE);
    display.fillRect(foodX*5,foodY*5,4,4,WHITE);
    for (int i=0;i<snakeLen;i++) display.fillRect(snakeX[i]*5,snakeY[i]*5,4,4,WHITE);
    display.display();
  }
}

void initBricks() {
  paddleX=50; ballX=64; ballY=50; ballDX=1.8; ballDY=-1.8;
  for (int i=0;i<3;i++) for (int j=0;j<6;j++) bricks[i][j]=true;
}

void runBricks() {
  if (checkGameExit()) return;
  if (millis() - prevGameTick > 25) {
    prevGameTick = millis();
    if (digitalRead(BTN_UP)   && paddleX > 0)  paddleX -= 5;
    if (digitalRead(BTN_DOWN) && paddleX < 100) paddleX += 5;
    ballX+=ballDX; ballY+=ballDY;
    if (ballX<=0||ballX>=127) ballDX*=-1;
    if (ballY<=0) ballDY*=-1;
    if (ballY>=58&&ballX>paddleX&&ballX<paddleX+25) ballDY*=-1;
    if (ballY>64) initBricks();
    display.clearDisplay();
    display.fillRect(paddleX,60,25,4,WHITE);
    display.fillCircle(ballX,ballY,2,WHITE);
    for (int i=0;i<3;i++)
      for (int j=0;j<6;j++)
        if (bricks[i][j]) {
          int bx=j*21,by=i*8+10;
          display.drawRect(bx,by,18,6,WHITE);
          if (ballX>bx&&ballX<bx+18&&ballY>by&&ballY<by+6){bricks[i][j]=false;ballDY*=-1;}
        }
    display.display();
  }
}