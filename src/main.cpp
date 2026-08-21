#include <Arduino.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- BLE UUIDs ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- Hardware & Bluetooth Setup ---
uint8_t     BT_MAC[6] = {0x13, 0xE0, 0x2F, 0x8D, 0x64, 0x43};
const char* BT_PIN    = "1234";

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite tileSpr = TFT_eSprite(&tft);
BluetoothSerial SerialBT;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool bleClientConnected = false;
uint32_t lastBleNotify = 0;

#define SCREEN_W  320
#define SCREEN_H  240
#define HEADER_H   20
#define COLS        3
#define ROWS        2
#define TILE_W    (SCREEN_W / COLS)
#define TILE_H    ((SCREEN_H - HEADER_H) / ROWS)

struct Gauge {
  const char* label;
  const char* unit;
  float  value;
  float  maxValue; 
  float  warnLow;
  float  warnHigh;
  float  minVal;
  float  maxVal;
  bool   hasValue;
  bool   trackMax; 
};

// 7 Gauges: Index 0-5 für TFT Display, Index 6 nur für Öltemperatur (App)
Gauge gauges[7] = {
  {"Kuehlwasser", "C",    0.f, -100.f, -1.0f,  105.0f, 0.0f, 120.0f, false, true},  // 0
  {"Ladedruck",   "bar",  0.f, -1.0f,  -0.9f,    1.5f, -1.0f,   2.0f, false, true},  // 1
  {"Getriebe",    "C",    0.f, -100.f, 20.0f,  110.0f, 0.f,  130.0f, false, true},  // 2
  {"Ansaugtemp",  "C",    0.f, -100.f, -20.f,   60.0f, -20.f,  80.0f, false, true},  // 3
  {"Tankinhalt",  "L",    0.f, -100.f,  5.0f,   60.0f,  0.f,   60.0f, false, false}, // 4
  {"Batterie",    "V",    0.f, -100.f, 11.5f,   15.3f, 10.f,   16.0f, false, true},  // 5
  {"Oeltemp",     "C",    0.f, -100.f, -1.0f,  110.0f, 0.0f, 130.0f, false, true}   // 6 (Nur BLE)
};

struct PIDDef { uint8_t mode; uint16_t pid; uint32_t header; bool isExtended; };
const PIDDef pidDefs[7] = {
  { 1, 0x05,   0x7DF, false}, // 0: Kühlwasser
  { 1, 0x0B,   0x7DF, false}, // 1: Ladedruck
  {22, 0x2104, 0x7E1, false}, // 2: Getriebe
  { 1, 0x0F,   0x7DF, false}, // 3: Ansaugtemp
  { 1, 0x2F,   0x7DF, false}, // 4: Tank
  { 1, 0x42,   0x7DF, false}, // 5: Batterie
  { 1, 0x5C,   0x7DF, false}  // 6: Öltemperatur (Standard OBD2 PID 5C)
};

bool btConnected = false, elmReady = false, needFullRedraw = true;
uint32_t lastReconnect = 0, lastRedraw = 0, lastSessionKeepAlive = 0, btConnectTime = 0;
uint32_t activeHeader = 0x000;
uint8_t currentGauge = 0;
char rawBuf[128];
float smoothedVolt = 0.0f;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
      bleClientConnected = true;
      needFullRedraw = true;
    }
    void onDisconnect(BLEServer* pServer) override {
      bleClientConnected = false;
      needFullRedraw = true;
      BLEDevice::startAdvertising();
    }
};

uint8_t hexByte(const char* s) {
  uint8_t v = 0;
  for (int i = 0; i < 2; i++) {
    v <<= 4; char c = s[i];
    if (c >= '0' && c <= '9') v |= c - '0';
    else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
    else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10;
  }
  return v;
}

bool elmRawCmd(const char* cmd, uint16_t timeoutMs = 600) {
  if (!SerialBT.connected()) return false;
  
  while (SerialBT.available()) SerialBT.read();
  SerialBT.print(cmd); SerialBT.print("\r");
  
  unsigned long start = millis();
  int idx = 0;
  memset(rawBuf, 0, sizeof(rawBuf));
  
  while (millis() - start < timeoutMs) {
    if (!SerialBT.connected()) return false;
    
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') return (idx > 0);
      if (idx < 126 && c >= ' ') rawBuf[idx++] = c;
    }
    yield();
  }
  return false;
}

bool initELM() {
  Serial.println("Initialisiere ELM327...");
  delay(500);
  
  if(!elmRawCmd("ATZ", 1500)) {
    Serial.println("ATZ keine Antwort.");
    return false;
  }
  delay(300);
  
  elmRawCmd("ATE0", 300); 
  elmRawCmd("ATS0", 300);
  elmRawCmd("ATH1", 300); 
  elmRawCmd("ATSP6", 300);
  elmRawCmd("ATCAF1", 300); 
  activeHeader = 0;
  Serial.println("ELM327 Bereit!");
  return true;
}

void queryGauge(uint8_t idx) {
  if (!btConnected || !elmReady) return;
  const PIDDef& def = pidDefs[idx];
  char cmd[32];
  
  if (activeHeader != def.header) {
    snprintf(cmd, sizeof(cmd), "ATSH%03X", (uint16_t)def.header);
    elmRawCmd(cmd, 200); 
    activeHeader = def.header;
    delay(20);
  }

  if (def.mode == 22 && (millis() - lastSessionKeepAlive > 2000)) {
    elmRawCmd("1003", 250); 
    lastSessionKeepAlive = millis();
  }

  if (def.mode == 1) snprintf(cmd, sizeof(cmd), "01%02X1", (uint8_t)def.pid);
  else               snprintf(cmd, sizeof(cmd), "22%04X1", def.pid);

  if (!elmRawCmd(cmd, (idx <= 1 ? 800 : 350))) return;
  
  char search[12];
  if (def.mode == 1) sprintf(search, "41%02X", (uint8_t)def.pid);
  else               sprintf(search, "62%04X", def.pid);
  
  char* p = strstr(rawBuf, search);
  if (!p && def.mode == 22) p = strstr(rawBuf, "62"); 

  if (p) {
    if (strstr(p, search)) p += strlen(search); else p += 2;
    while(*p == ' ') p++;
    if (strlen(p) < 2) return;

    if (idx == 0) { // Kuehlwasser
      gauges[idx].value = (float)hexByte(p) - 40.0f; 
    }
    else if (idx == 1) { // Ladedruck
      float absBar = (float)hexByte(p) / 100.0f;
      gauges[idx].value = absBar - 1.0f; 
    }
    else if (def.pid == 0x42) { // Batterie
      float instantVolt = (float)(hexByte(p) * 256 + hexByte(p + 2)) / 1000.0f;
      smoothedVolt = (smoothedVolt < 5.0f) ? instantVolt : (smoothedVolt * 0.8f) + (instantVolt * 0.2f);
      gauges[idx].value = smoothedVolt;
    }
    else if (def.mode == 1) {
      if (def.pid == 0x2F) gauges[idx].value = ((float)hexByte(p) * 100.0f / 255.0f) * 0.55f;
      else                 gauges[idx].value = (float)hexByte(p) - 40.0f; // Passt für Ansaugtemp (0x0F) und Öltemp (0x5C)
    }
    else if (def.pid == 0x2104) { // Getriebe
      gauges[idx].value = (float)hexByte(p); 
    }
    
    if (gauges[idx].trackMax) {
      if (gauges[idx].value > gauges[idx].maxValue) gauges[idx].maxValue = gauges[idx].value;
    }
    gauges[idx].hasValue = true;
  }
}

// Sendet alle Werte inkl. Öltemperatur per BLE an die App
void sendBleJsonData() {
  if (!bleClientConnected) return;

  char jsonBuffer[256];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"water\":%.1f,\"boost\":%.2f,\"gearTemp\":%.1f,\"intake\":%.1f,\"fuel\":%d,\"volt\":%.1f,\"oil\":%.1f}\n",
           gauges[0].value,        // Kuehlwasser
           gauges[1].value,        // Ladedruck
           gauges[2].value,        // Getriebe
           gauges[3].value,        // Ansaugtemp
           (int)gauges[4].value,   // Tankinhalt
           gauges[5].value,        // Batterie
           gauges[6].value         // Öltemperatur
  );

  pCharacteristic->setValue(jsonBuffer);
  pCharacteristic->notify();
}

void drawTile(uint8_t i) {
  const Gauge& g = gauges[i];
  int16_t tx = (i % COLS) * TILE_W;
  int16_t ty = HEADER_H + (i / COLS) * TILE_H;
  int16_t cx = TILE_W / 2;
  int16_t cy = TILE_H / 2;
  
  tileSpr.fillSprite(TFT_BLACK);
  tileSpr.drawRect(0, 0, TILE_W, TILE_H, TFT_RED);

  tileSpr.setTextColor(TFT_YELLOW); 
  tileSpr.setCursor(5, 7); 
  tileSpr.print(g.label);

  if (g.trackMax && (g.maxValue > -99.f)) {
    tileSpr.setTextDatum(TR_DATUM); 
    tileSpr.setTextColor(TFT_CYAN);
    char mBuf[10];
    if (i == 5 || i == 4 || i == 1) dtostrf(g.maxValue, 2, 1, mBuf); 
    else snprintf(mBuf, sizeof(mBuf), "M:%d", (int)roundf(g.maxValue));
    tileSpr.drawString(mBuf, TILE_W - 5, 7);
  }

  tileSpr.setTextDatum(MC_DATUM);
  uint16_t vCol = TFT_WHITE;
  if (g.hasValue) {
    char buf[12];
    if (i == 5 || i == 4 || i == 1) dtostrf(g.value, 2, 1, buf); 
    else snprintf(buf, sizeof(buf), "%d", (int)roundf(g.value));
    
    if (g.value < g.warnLow || g.value > g.warnHigh) vCol = TFT_RED;
    tileSpr.setTextColor(vCol); 
    tileSpr.setTextSize(3);
    tileSpr.drawString(buf, cx, cy - 5);
  } else {
    tileSpr.setTextColor(TFT_DARKGREY); 
    tileSpr.setTextSize(3); 
    tileSpr.drawString("---", cx, cy - 5);
  }

  tileSpr.setTextSize(1); 
  tileSpr.setTextColor(TFT_CYAN);
  tileSpr.drawString(g.unit, cx, cy + 18);

  int16_t bX = 8, bY = TILE_H - 12, bW = TILE_W - 16, bH = 5;
  tileSpr.drawRect(bX, bY, bW, bH, TFT_DARKGREY);
  if (g.hasValue) {
    float pct = constrain((g.value - g.minVal) / (g.maxVal - g.minVal), 0.0f, 1.0f);
    tileSpr.fillRect(bX + 1, bY + 1, pct * (bW - 2), bH - 2, (vCol == TFT_RED) ? TFT_RED : TFT_GREEN);
  }

  tileSpr.pushSprite(tx, ty);
}

void drawHeader() {
  tft.fillRect(0, 0, SCREEN_W, HEADER_H, TFT_BLACK);
  tft.setTextSize(1); 
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(5, 5); 
  tft.print("RS3 8V DAZA");

  tft.setTextDatum(TR_DATUM);
  int rightX = SCREEN_W - 5;

  tft.setTextColor(bleClientConnected ? TFT_CYAN : TFT_DARKGREY);
  tft.drawString(bleClientConnected ? "APP:ON" : "APP:OFF", rightX, 5);
  rightX -= 55;

  tft.setTextColor(elmReady ? TFT_GREEN : TFT_RED);
  tft.drawString(elmReady ? "ELM:ON" : "ELM:OFF", rightX, 5);
}

void drawDashboard(bool full) {
  if (full) {
    tft.fillScreen(TFT_BLACK);
    drawHeader();
  }
  // Nur die ersten 6 Gauges (Index 0 bis 5) auf das TFT zeichnen
  for (uint8_t i = 0; i < 6; i++) drawTile(i);
  if (!full) drawHeader();
}

void setupBLE() {
  BLEDevice::init("RS3-Dashboard-BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void setup() {
  Serial.begin(115200); 
  delay(1000);
  tft.init(); 
  tft.setRotation(3); 
  tft.invertDisplay(true);

  tileSpr.createSprite(TILE_W, TILE_H);
  
  SerialBT.begin("RS3-Dashboard", true); 
  SerialBT.setPin(BT_PIN);

  setupBLE();
  drawDashboard(true);
}

void loop() {
  uint32_t now = millis();

  // 1. OBD Bluetooth Verbindungsverwaltung
  if (!SerialBT.connected()) {
    if (now - lastReconnect > 5000) {
      lastReconnect = now; 
      btConnected = false; 
      elmReady = false;
      Serial.println("Verbinde mit ELM327 BT...");
      SerialBT.connect(BT_MAC); 
      btConnectTime = millis();
      needFullRedraw = true;
    }
  } else {
    if (!btConnected) {
      btConnected = true;
      btConnectTime = millis();
    }
    // Nach erfolgreicher BT-Kopplung erst 2 Sekunden warten, bevor ATZ gesendet wird
    if (!elmReady && (now - btConnectTime > 2000)) { 
      elmReady = initELM(); 
      needFullRedraw = true; 
    }
  }

  // 2. OBD Werte abfragen (Zyklus geht jetzt von 0 bis 6, inkl. Öltemperatur)
  if (elmReady) { 
    queryGauge(currentGauge); 
    currentGauge = (currentGauge + 1) % 7; 
  }

  // 3. Display Refresh
  if (now - lastRedraw > 300) { 
    lastRedraw = now; 
    drawDashboard(needFullRedraw); 
    needFullRedraw = false; 
  }

  // 4. BLE Live-Daten an App senden
  if (now - lastBleNotify > 200) {
    lastBleNotify = now;
    sendBleJsonData();
  }

  yield();
}
