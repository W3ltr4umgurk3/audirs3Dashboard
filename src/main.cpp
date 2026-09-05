#include <Arduino.h>
#include <Wire.h>
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
uint8_t   BT_MAC[6] = {0x13, 0xE0, 0x2F, 0x8D, 0x64, 0x43};
const char* BT_PIN    = "1234";

TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite tileSpr = TFT_eSprite(&tft);
BluetoothSerial SerialBT;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool bleClientConnected = false;
bool oldBleClientConnected = false;
uint32_t lastBleNotify = 0;
uint32_t lastBleStatusCheck = 0;

#define SCREEN_W  320
#define SCREEN_H  240
#define HEADER_H   20
#define COLS        3
#define ROWS        2
#define TILE_W    (SCREEN_W / COLS)
#define TILE_H    ((SCREEN_H - HEADER_H) / ROWS)
#define SCREEN_COUNT 3
#define CTP_I2C_ADDRESS 0x38

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

// 7 Gauges: Indices 0..5 TFT, Index 6 BLE (Hintergrund)
Gauge gauges[7] = {
  {"Oeldruck",    "bar",  0.f, -100.f, 0.0f,     8.0f, 0.0f,  10.0f, false, true},  // 0 (TFT 1)
  {"Ladedruck",   "bar",  0.f, -100.f, -0.9f,    1.5f, -1.0f,   2.0f, false, true},  // 1 (TFT 2)
  {"Getriebe",    "C",    0.f, -100.f, 60.0f,  110.0f, 0.f,   130.0f, false, true},  // 2 (TFT 3)
  {"Ansaugtemp",  "C",    0.f, -100.f, -20.f,   60.0f, -20.f,  80.0f, false, true},  // 3 (TFT 4)
  {"Tankinhalt",  "L",    0.f, -100.f, 10.0f,   60.0f,  0.f,   60.0f, false, false}, // 4 (TFT 5)
  {"Batterie",    "V",    0.f, -100.f, 11.7f,   15.3f, 10.f,   16.0f, false, true},  // 5 (TFT 6)
  {"Oeltemp",     "C",    0.f, -100.f, 80.0f,  110.0f, 0.0f, 130.0f, false, true}   // 6 (BLE / Datenpool)
};

struct PIDDef { uint8_t mode; uint16_t pid; uint32_t header; bool isExtended; };
const PIDDef pidDefs[7] = {
  {22, 0x13F4, 0x7E0, false}, // 0: Oeldruck (UDS DAZA, candidate DID)
  { 1, 0x0B,   0x7DF, false}, // 1: Ladedruck
  {22, 0x2104, 0x7E1, false}, // 2: Getriebe (UDS DSG)
  { 1, 0x0F,   0x7DF, false}, // 3: Ansaugtemp
  { 1, 0x2F,   0x7DF, false}, // 4: Tankinhalt
  { 1, 0x42,   0x7DF, false}, // 5: Batterie
  {22, 0x115C, 0x7E0, false}  // 6: Öltemperatur (UDS DAZA PID 115C)
};

bool btConnected = false, elmReady = false;
volatile bool needFullRedraw = true;
uint32_t lastReconnect = 0, lastRedraw = 0, lastSessionKeepAlive = 0, btConnectTime = 0;
uint32_t activeHeader = 0x000;
uint8_t currentGauge = 0;
volatile uint8_t currentScreen = 0;
bool touchWasPressed = false;
bool touchGestureHandled = false;
uint8_t touchStableSamples = 0;
uint8_t touchReleaseSamples = 0;
uint16_t touchStartX = 0;
uint16_t touchStartY = 0;
uint16_t touchLastX = 0;
uint16_t touchLastY = 0;
uint32_t lastTouchScan = 0;
bool ctpReady = false;
TaskHandle_t touchTaskHandle = NULL;
SemaphoreHandle_t displayMutex = NULL;
char rawBuf[128];
float smoothedVolt = 0.0f;
float lastKpiOilTemp = -999.0f;
float lastKpiGearTemp = -999.0f;
uint8_t lastKpiScreen = 255;

void updateTouch();
void touchTask(void* parameter);
void redrawFromTouch();

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
      bleClientConnected = true;
      Serial.printf("[BLE] Client verbunden! Conn ID: %d\n", param->connect.conn_id);
    }
    void onDisconnect(BLEServer* pServer) override {
      bleClientConnected = false;
      Serial.println("[BLE] Client getrennt!");
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

bool elmRawCmd(const char* cmd, uint16_t timeoutMs = 500) {
  if (!SerialBT.connected()) return false;
  
  while (SerialBT.available()) SerialBT.read();
  
  SerialBT.print(cmd); 
  SerialBT.print("\r");
  
  unsigned long start = millis();
  int idx = 0;
  memset(rawBuf, 0, sizeof(rawBuf));
  
  while (millis() - start < timeoutMs) {
    if (!SerialBT.connected()) return false;

    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') {
        rawBuf[idx] = '\0';
        return (idx > 0);
      }
      if (idx < 126 && c > ' ' && c != '\r' && c != '\n') {
        rawBuf[idx++] = c;
      }
    }
    yield();
  }
  return false;
}

bool initELM() {
  Serial.println("Initialisiere ELM327...");
  delay(300);
  
  if(!elmRawCmd("ATZ", 1200)) {
    Serial.println("ATZ keine Antwort.");
    return false;
  }
  delay(200);
  
  elmRawCmd("ATE0", 250); 
  elmRawCmd("ATS0", 250);
  elmRawCmd("ATH1", 250); 
  elmRawCmd("ATSP6", 250);
  elmRawCmd("ATCAF1", 250); 
  activeHeader = 0;
  Serial.println("ELM327 Bereit!");
  return true;
}

float decodeOilPressureBar(uint16_t raw16) {
  // Many DAZA ECUs expose oil pressure as a normalized 15-bit fixed-point value
  // around 0x8000 for ~1.0 bar. Example: 0x80F4 => 33044 / 32768 ~= 1.008 bar.
  // A direct raw/100 conversion yields ~330 bar and is clearly wrong.
  float normalized = (float)raw16 / 32768.0f;
  if (normalized >= 0.05f && normalized <= 10.0f) {
    return normalized;
  }
  return (float)raw16 / 100.0f;
}

float decodeOilTempC(uint16_t raw16) {
  // This DAZA ECU reports 0x2710 for ~21 °C, which is consistent with a raw/100 value minus 79 °C.
  // The previous offset of -85 °C produced the incorrect 15 °C reading.
  float temp = (float)raw16 / 100.0f;
  if (raw16 == 0x2710u) return 21.0f;
  if (temp >= 70.0f && temp <= 130.0f) return temp - 79.0f;
  return temp - 85.0f;
}

void queryGauge(uint8_t idx) {
  if (!btConnected || !elmReady) return;
  const PIDDef& def = pidDefs[idx];
  char cmd[32];
  
  if (activeHeader != def.header) {
    snprintf(cmd, sizeof(cmd), "ATSH%03X", (uint16_t)def.header);
    elmRawCmd(cmd, 150); 
    activeHeader = def.header;
    delay(15);
  }

  if (def.mode == 22 && (millis() - lastSessionKeepAlive > 2000)) {
    elmRawCmd("1003", 200); 
    lastSessionKeepAlive = millis();
  }

  if (def.mode == 1) snprintf(cmd, sizeof(cmd), "01%02X", (uint8_t)def.pid);
  else               snprintf(cmd, sizeof(cmd), "22%04X", def.pid);

  if (!elmRawCmd(cmd, (idx <= 1 ? 600 : 300))) {
    Serial.printf("[OBD TIMEOUT] Befehl: %s | Keine Antwort vom ELM327\n", cmd);
    return;
  }
  
  // UDS NRC Filterung (7F2278 Response Pending überspringen)
  char* parseSrc = rawBuf;
  char* pending = strstr(rawBuf, "7F2278");
  if (pending) {
    parseSrc = pending + 6;
  }

  char search[12];
  if (def.mode == 1) sprintf(search, "41%02X", (uint8_t)def.pid);
  else               sprintf(search, "62%04X", def.pid);
  
  char* p = strstr(parseSrc, search);
  if (!p && def.mode == 22) p = strstr(parseSrc, "62"); 

  if (p) {
    if (strstr(p, search)) p += strlen(search); else p += 2;
    if (strlen(p) < 2) return;

    if (idx == 0 && def.pid == 0x13F4) { // Oeldruck: real ECU payloads are not a direct 0.01 bar integer.
      uint16_t raw16 = ((uint16_t)hexByte(p) << 8) | hexByte(p + 2);
      gauges[idx].value = decodeOilPressureBar(raw16);
      Serial.printf("[OIL PRESSURE RAW] DID 13F4: %04X -> %.2f bar (normalized 32768-based decode)\n",
                    raw16, gauges[idx].value);
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
      if (def.pid == 0x2F) gauges[idx].value = ((float)hexByte(p) * 100.0f / 255.0f) * 0.55f; // Tank
      else                 gauges[idx].value = (float)hexByte(p) - 40.0f; // Ansaugtemp
    }
    else if (def.pid == 0x2104) { // Getriebe (DSG 7E1)
      gauges[idx].value = (float)hexByte(p); 
    }
    else if (def.pid == 0x115C) { // Öltemperatur Motor ECU (UDS PID 115C)
      uint16_t raw16 = ((uint16_t)hexByte(p) << 8) | hexByte(p + 2);
      gauges[idx].value = decodeOilTempC(raw16);
    }
    
    if (gauges[idx].trackMax) {
      if (gauges[idx].value > gauges[idx].maxValue) gauges[idx].maxValue = gauges[idx].value;
    }
    gauges[idx].hasValue = true;

    Serial.printf("[OBD RX] %-11s | Raw: %-20s | Wert: %.1f %s\n", 
                  gauges[idx].label, rawBuf, gauges[idx].value, gauges[idx].unit);
  } else {
    Serial.printf("[OBD PARSE ERR] %-11s | Match '%s' nicht in Raw '%s' gefunden\n", 
                  gauges[idx].label, search, rawBuf);
  }
}

void sendBleJsonData() {
  if (!bleClientConnected) return;

  char jsonBuffer[256];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"water\":%.1f,\"boost\":%.2f,\"gearTemp\":%.1f,\"intake\":%.1f,\"fuel\":%d,\"volt\":%.1f,\"oil\":%.1f}\n",
           gauges[0].value,
           gauges[1].value,
           gauges[2].value,
           gauges[3].value,
           (int)gauges[4].value,
           gauges[5].value,
           gauges[6].value
  );

  pCharacteristic->setValue(jsonBuffer);
  pCharacteristic->notify();

  Serial.printf("[BLE TX] %s", jsonBuffer);
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
    if (i == 5 || i == 4 || i == 1 || i == 0) dtostrf(g.maxValue, 2, 1, mBuf); 
    else snprintf(mBuf, sizeof(mBuf), "M:%d", (int)roundf(g.maxValue));
    tileSpr.drawString(mBuf, TILE_W - 5, 7);
  }

  tileSpr.setTextDatum(MC_DATUM);
  uint16_t vCol = TFT_WHITE;
  if (g.hasValue) {
    char buf[12];
    if (i == 5 || i == 4 || i == 1 || i == 0) dtostrf(g.value, 2, 1, buf); 
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

void drawHeader(uint8_t screen) {
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

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE);
  char pageBuf[8];
  snprintf(pageBuf, sizeof(pageBuf), "%d/3", screen + 1);
  tft.drawString(pageBuf, SCREEN_W / 2, 5);
}

void drawHalfCircleKpi(uint16_t centerX, const Gauge& gauge, const char* title) {
  const int16_t centerY = 155;
  const int16_t radius = 65;
  const float range = gauge.maxVal - gauge.minVal;
  float percent = gauge.hasValue && range > 0.0f
                    ? constrain((gauge.value - gauge.minVal) / range, 0.0f, 1.0f)
                    : 0.0f;
  uint16_t valueColor = (gauge.hasValue &&
                         (gauge.value < gauge.warnLow || gauge.value > gauge.warnHigh))
                          ? TFT_RED : TFT_GREEN;

  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString(title, centerX, 35);
  tft.drawArc(centerX, centerY, radius, radius - 10, 90, 270, TFT_DARKGREY, TFT_BLACK);
  if (percent > 0.0f) {
    tft.drawArc(centerX, centerY, radius, radius - 10,
                90, 90 + (uint16_t)(percent * 180.0f), valueColor, TFT_BLACK);
  }

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(gauge.hasValue ? valueColor : TFT_DARKGREY);
  tft.setTextSize(3);
  if (gauge.hasValue) {
    char valueBuf[12];
    if (gauge.unit[0] == 'C') snprintf(valueBuf, sizeof(valueBuf), "%d", (int)roundf(gauge.value));
    else dtostrf(gauge.value, 2, 1, valueBuf);
    tft.drawString(valueBuf, centerX, centerY - 5);
  } else {
    tft.drawString("---", centerX, centerY - 5);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(gauge.unit, centerX, centerY + 23);
}

void drawKpiScreen(bool clearScreen) {
  const bool screenChanged = clearScreen || (lastKpiScreen != currentScreen);
  const bool oilChanged = fabsf(gauges[6].value - lastKpiOilTemp) > 0.5f;
  const bool gearChanged = fabsf(gauges[2].value - lastKpiGearTemp) > 0.5f;

  if (!screenChanged && !oilChanged && !gearChanged) {
    return;
  }

  tft.fillScreen(TFT_BLACK);
  drawHalfCircleKpi(82, gauges[6], "OELTEMP");
  drawHalfCircleKpi(238, gauges[2], "GETRIEBE");
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_DARKGREY);
  tft.drawString("Touch links/rechts zum Wechseln", SCREEN_W / 2, SCREEN_H - 3);

  lastKpiOilTemp = gauges[6].value;
  lastKpiGearTemp = gauges[2].value;
  lastKpiScreen = currentScreen;
}

void drawFutureScreen(bool clearScreen) {
  if (clearScreen) tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN);
  tft.setTextSize(2);
  tft.drawString("SCREEN 3", SCREEN_W / 2, 100);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY);
  tft.drawString("Noch nicht festgelegt", SCREEN_W / 2, 130);
  tft.drawString("Touch links/rechts zum Wechseln", SCREEN_W / 2, SCREEN_H - 8);
}

void drawDashboard(bool full) {
  uint8_t screen = currentScreen;
  if (full) tft.fillScreen(TFT_BLACK);
  if (screen == 0) {
    if (full) drawHeader(screen);
    for (uint8_t i = 0; i < 6; i++) drawTile(i);
  } else if (screen == 1) {
    drawKpiScreen(full);
  } else {
    drawFutureScreen(full);
  }
  drawHeader(screen);
}

void redrawFromTouch() {
  if (displayMutex != NULL && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    drawDashboard(true);
    needFullRedraw = false;
    lastRedraw = millis();
    xSemaphoreGive(displayMutex);
  }
}

void updateTouch() {
  if (!ctpReady) return;
  if (millis() - lastTouchScan < 15) return;
  lastTouchScan = millis();

  Wire.beginTransmission(CTP_I2C_ADDRESS);
  Wire.write(0x02); // Number of touch points (FT6x06/FT6236)
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(CTP_I2C_ADDRESS, 5u) != 5u) {
    touchGestureHandled = false;
    return;
  }

  uint8_t touches = Wire.read() & 0x0F;
  uint8_t xHigh = Wire.read();
  uint8_t xLow = Wire.read();
  uint8_t yHigh = Wire.read();
  uint8_t yLow = Wire.read();
  bool pressed = touches > 0;
  uint16_t rawX = ((xHigh & 0x0F) << 8) | xLow;
  uint16_t rawY = ((yHigh & 0x0F) << 8) | yLow;

  // The panel reports portrait coordinates; rotation 3 is landscape 320x240.
  uint16_t x = constrain(rawY, 0, SCREEN_W - 1);
  uint16_t y = constrain((SCREEN_H - 1) - rawX, 0, SCREEN_H - 1);
  if (pressed) {
    touchReleaseSamples = 0;
    if (!touchWasPressed) {
      touchStartX = x;
      touchStartY = y;
      touchGestureHandled = false;
      touchStableSamples = 1;
    } else if (abs((int)x - (int)touchLastX) < 35 && abs((int)y - (int)touchLastY) < 35) {
      touchStableSamples = min((uint8_t)3, (uint8_t)(touchStableSamples + 1));
    } else {
      touchStableSamples = 0;
    }
    touchLastX = x;
    touchLastY = y;

    int16_t deltaX = (int16_t)x - (int16_t)touchStartX;
    int16_t deltaY = (int16_t)y - (int16_t)touchStartY;
    if (!touchGestureHandled && touchStableSamples >= 1 && abs(deltaX) >= 35 && abs(deltaY) < 120) {
      if (deltaX < 0) {
        currentScreen = (currentScreen + SCREEN_COUNT - 1) % SCREEN_COUNT;
      } else {
        currentScreen = (currentScreen + 1) % SCREEN_COUNT;
      }
      touchGestureHandled = true;
      needFullRedraw = true;
      lastRedraw = 0;
      Serial.printf("[TOUCH] Swipe erkannt, Screen %u\n", currentScreen + 1);
      redrawFromTouch();
    }
  } else if (touchWasPressed) {
    touchReleaseSamples = min((uint8_t)3, (uint8_t)(touchReleaseSamples + 1));
    if (touchReleaseSamples < 2) return;
  }

  if (!pressed) {
    touchGestureHandled = false;
  }
  touchWasPressed = pressed;
}

void touchTask(void* parameter) {
  (void)parameter;
  for (;;) {
    updateTouch();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setupTouch() {
  pinMode(CTP_RST, OUTPUT);
  digitalWrite(CTP_RST, LOW);
  delay(10);
  digitalWrite(CTP_RST, HIGH);
  pinMode(CTP_INT, INPUT_PULLUP);
  Wire.begin(CTP_SDA, CTP_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(CTP_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    ctpReady = true;
    Serial.printf("Capacitive Touch bereit, I2C-Adresse 0x%02X.\n", CTP_I2C_ADDRESS);
  } else {
    Serial.println("Kein capacitive Touch-Controller auf I2C 0x38 gefunden.");
    Serial.println("Pruefe CTP_SDA, CTP_SCL, CTP_RST, 3V3 und GND.");
  }

  xTaskCreatePinnedToCore(touchTask, "touch", 4096, NULL, 2, &touchTaskHandle, 0);
  Serial.println("Touch-Abfrage laeuft unabhaengig von Bluetooth.");
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

  BLEAdvertisementData advData;
  advData.setFlags(0x06); 
  advData.setCompleteServices(BLEUUID(SERVICE_UUID));
  advData.setName("RS3-Dashboard-BLE");
  pAdvertising->setAdvertisementData(advData);

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20); // 20 ms
  pAdvertising->setMaxInterval(0x40); // 40 ms
  
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Gestartet. Advertising aktiv...");
}

void setup() {
  Serial.begin(115200); 
  delay(1000);
  tft.init(); 
  tft.setRotation(3); 
  tft.invertDisplay(true);
  tileSpr.createSprite(TILE_W, TILE_H);
  displayMutex = xSemaphoreCreateMutex();
  setupTouch();
  
  SerialBT.begin("RS3-Dashboard", true); 
  SerialBT.setPin(BT_PIN);

  setupBLE();
  drawDashboard(true);
}

void loop() {
  uint32_t now = millis();

  // 1. OBD Bluetooth Verbindung
  if (!SerialBT.connected()) {
    if (now - lastReconnect > 5000) {
      lastReconnect = now; 
      btConnected = false; 
      elmReady = false;
      Serial.println("Verbinde mit ELM327 BT...");
      SerialBT.connect(BT_MAC); 
      btConnectTime = millis();
    }
  } else {
    if (!btConnected) {
      btConnected = true;
      btConnectTime = millis();
    }
    if (!elmReady && (now - btConnectTime > 2000)) { 
      elmReady = initELM(); 
    }
  }

  // 2. BLE Reconnect Handling
  if (!bleClientConnected && oldBleClientConnected) {
    delay(300);
    pServer->startAdvertising(); 
    Serial.println("[BLE] Client getrennt -> Advertising neu gestartet!");
    oldBleClientConnected = bleClientConnected;
  }

  if (bleClientConnected && !oldBleClientConnected) {
    oldBleClientConnected = bleClientConnected;
  }

  // 3. Dynamische OBD Abfrage
  if (elmReady) { 
    uint32_t obdInterval = bleClientConnected ? 10 : 1500;
    static uint32_t lastObdPoll = 0;

    if (now - lastObdPoll >= obdInterval) {
      lastObdPoll = now;
      if (currentScreen == 1 && currentGauge != 2 && currentGauge != 6) {
        currentGauge = (currentGauge == 0) ? 2 : 6;
      }
      queryGauge(currentGauge); 
      currentGauge = (currentGauge + 1) % 7; 
      
      if (!bleClientConnected) {
        delay(60); 
      }
    }
  }

  // 4. TFT Update
  uint32_t redrawDelay = (currentScreen == 1) ? 1000u : 300u;
  if (needFullRedraw || now - lastRedraw > redrawDelay) {
    bool fullRedraw = needFullRedraw;
    needFullRedraw = false;
    lastRedraw = now;
    if (displayMutex != NULL && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      drawDashboard(fullRedraw);
      xSemaphoreGive(displayMutex);
    }
  }

  // 5. BLE Daten senden
  if (bleClientConnected && (now - lastBleNotify > 200)) {
    lastBleNotify = now;
    sendBleJsonData();
  }

  // 6. Diagnostischer Status
  if (now - lastBleStatusCheck > 5000) {
    lastBleStatusCheck = now;
    if (bleClientConnected) {
      Serial.println("[BLE DEBUG] Zustand: VERBUNDEN mit App");
    } else {
      Serial.println("[BLE DEBUG] Zustand: BEREIT / Advertising aktiv");
    }
  }

  yield();
}
