#include <Arduino.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"

// --- Hardware & Bluetooth Setup ---
uint8_t     BT_MAC[6] = {Enter MAC Adress in HEX};
const char* BT_PIN    = "1234";

TFT_eSPI      tft = TFT_eSPI();
BluetoothSerial SerialBT;

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

Gauge gauges[6] = {
  {"Kuehlwasser", "C",     0.f, -100.f, -1.0f,  105.0f, 0.0f, 120.0f, false, true},  
  {"Ladedruck",   "bar",   0.f, -1.0f,  -0.9f,    1.5f, -1.0f,   2.0f, false, true}, // Kachel 2 angepasst für Ladedruck
  {"Getriebe",    "C",     0.f, -100.f, 20.0f,  110.0f, 0.f,  130.0f, false, true},  
  {"Ansaugtemp",  "C",     0.f, -100.f, -20.f,  60.0f, -20.f, 80.0f,  false, true},  
  {"Tankinhalt",  "L",     0.f, -100.f,  5.0f,  60.0f,  0.f,  60.0f,  false, false}, 
  {"Batterie",    "V",     0.f, -100.f, 11.5f,  15.3f,  10.f, 16.0f,  false, true}   
};

struct PIDDef { uint8_t mode; uint16_t pid; uint32_t header; bool isExtended; };
const PIDDef pidDefs[6] = {
  { 1, 0x05,   0x7DF, false},      // Kühlwasser
  { 1, 0x0B,   0x7DF, false},      // Saugrohrdruck (Standard OBD2 Mode 01 PID 0B)
  {22, 0x2104, 0x7E1, false},      // Getriebe
  { 1, 0x0F,   0x7DF, false},      // Ansaugtemp
  { 1, 0x2F,   0x7DF, false},      // Tank
  { 1, 0x42,   0x7DF, false}       // Batterie
};

bool btConnected = false, elmReady = false, needFullRedraw = true;
uint32_t lastReconnect = 0, lastRedraw = 0, lastSessionKeepAlive = 0;
uint32_t activeHeader = 0x000;
uint8_t currentGauge = 0;
char rawBuf[128];
float smoothedVolt = 0.0f;

uint8_t hexByte(const char* s) {
  uint8_t v = 0;
  for (int i = 0; i < 2; i++) {
    v <<= 4; char c = s[i];
    if (c >= '0' && c <= '9') v |= c - '0';
    else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
  }
  return v;
}

bool elmRawCmd(const char* cmd, uint16_t timeoutMs = 600) {
  while (SerialBT.available()) SerialBT.read();
  SerialBT.print(cmd); SerialBT.print("\r");
  unsigned long start = millis();
  int idx = 0;
  memset(rawBuf, 0, sizeof(rawBuf));
  while (millis() - start < timeoutMs) {
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
  Serial.println("Sende ATZ...");
  if(!elmRawCmd("ATZ", 2000)) return false;
  delay(500);
  elmRawCmd("ATE0"); 
  elmRawCmd("ATS0");
  elmRawCmd("ATH1"); 
  elmRawCmd("ATSP6");
  elmRawCmd("ATCAF1"); 
  activeHeader = 0;
  return true;
}

void queryGauge(uint8_t idx) {
  if (!btConnected || !elmReady) return;
  const PIDDef& def = pidDefs[idx];
  char cmd[32];
  
  if (activeHeader != def.header) {
    snprintf(cmd, sizeof(cmd), "ATSH%03X", (uint16_t)def.header);
    elmRawCmd(cmd, 250); 
    activeHeader = def.header;
    delay(50);
  }

  if (def.mode == 22 && (millis() - lastSessionKeepAlive > 2000)) {
    elmRawCmd("1003", 300); 
    lastSessionKeepAlive = millis();
  }

  if (def.mode == 1) snprintf(cmd, sizeof(cmd), "01%02X1", (uint8_t)def.pid);
  else               snprintf(cmd, sizeof(cmd), "22%04X1", def.pid);

  if (!elmRawCmd(cmd, (idx <= 1 ? 1200 : 400))) return;
  
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
    else if (idx == 1) { // Ladedruck Formel: (A kPa - 100 kPa atmosphärischer Druck) / 100.0 für bar
      float absBar = (float)hexByte(p) / 100.0f;
      gauges[idx].value = absBar - 1.0f; // Umrechnung in reinen Überdruck
    }
    else if (def.pid == 0x42) { // Batterie
      float instantVolt = (float)(hexByte(p) * 256 + hexByte(p + 2)) / 1000.0f;
      smoothedVolt = (smoothedVolt < 5.0f) ? instantVolt : (smoothedVolt * 0.8f) + (instantVolt * 0.2f);
      gauges[idx].value = smoothedVolt;
    }
    else if (def.mode == 1) {
      if (def.pid == 0x2F) gauges[idx].value = ((float)hexByte(p) * 100.0f / 255.0f) * 0.55f;
      else                 gauges[idx].value = (float)hexByte(p) - 40.0f;
    }
    else if (def.pid == 0x2104) {
      gauges[idx].value = (float)hexByte(p); 
    }
    
    Serial.printf("Update %s: %.2f (Raw: %s)\n", gauges[idx].label, gauges[idx].value, rawBuf);

    if (gauges[idx].trackMax) {
        if (gauges[idx].value > gauges[idx].maxValue) gauges[idx].maxValue = gauges[idx].value;
    }
    gauges[idx].hasValue = true;
  } else {
    Serial.printf("Fehler %s: Parse-Fail (Raw: %s)\n", gauges[idx].label, rawBuf);
  }
}

void drawTile(uint8_t i) {
  const Gauge& g = gauges[i];
  int16_t tx = (i % COLS) * TILE_W, ty = HEADER_H + (i / COLS) * TILE_H;
  int16_t cx = tx + TILE_W / 2, cy = ty + TILE_H / 2;
  
  tft.fillRect(tx + 2, ty + 4, TILE_W - 4, TILE_H - 8, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW); tft.setCursor(tx + 5, ty + 7); tft.print(g.label);

  if (g.trackMax && (g.maxValue > -99.f)) {
    tft.setTextDatum(TR_DATUM); tft.setTextColor(TFT_CYAN);
    char mBuf[10];
    if (i == 5 || i == 4 || i == 1) dtostrf(g.maxValue, 2, 1, mBuf); // Ladedruck Max-Wert mit Kommastelle
    else snprintf(mBuf, sizeof(mBuf), "M:%d", (int)roundf(g.maxValue));
    tft.drawString(mBuf, tx + TILE_W - 5, ty + 7);
  }

  tft.setTextDatum(MC_DATUM);
  uint16_t vCol = TFT_WHITE;
  if (g.hasValue) {
    char buf[12];
    if (i == 5 || i == 4 || i == 1) dtostrf(g.value, 2, 1, buf); // Ladedruck Live-Wert mit Kommastelle
    else snprintf(buf, sizeof(buf), "%d", (int)roundf(g.value));
    
    if (g.value < g.warnLow || g.value > g.warnHigh) vCol = TFT_RED;
    tft.setTextColor(vCol); tft.setTextSize(3);
    tft.drawString(buf, cx, cy - 5);
  } else {
    tft.setTextColor(TFT_DARKGREY); tft.setTextSize(3); tft.drawString("---", cx, cy - 5);
  }

  tft.setTextSize(1); tft.setTextColor(TFT_CYAN);
  tft.drawString(g.unit, cx, cy + 18);

  int16_t bX = tx + 8, bY = ty + TILE_H - 12, bW = TILE_W - 16, bH = 5;
  tft.drawRect(bX, bY, bW, bH, TFT_DARKGREY);
  if (g.hasValue) {
    float pct = constrain((g.value - g.minVal) / (g.maxVal - g.minVal), 0.0f, 1.0f);
    tft.fillRect(bX + 1, bY + 1, pct * (bW - 2), bH - 2, (vCol == TFT_RED) ? TFT_RED : TFT_GREEN);
  }
}

void drawDashboard(bool full) {
  if (full) {
    tft.fillScreen(TFT_BLACK);
    for (uint8_t i = 0; i < 6; i++) {
      int16_t tx = (i % COLS) * TILE_W, ty = HEADER_H + (i / COLS) * TILE_H;
      tft.drawRect(tx, ty, TILE_W, TILE_H, TFT_RED);
    }
  }
  for (uint8_t i = 0; i < 6; i++) drawTile(i);
  tft.fillRect(0, 0, SCREEN_W, HEADER_H, TFT_BLACK);
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.setCursor(5, 5); tft.print("RS3 8V DAZA Dashboard");
  tft.setTextDatum(TR_DATUM);
  if (elmReady) { tft.setTextColor(TFT_GREEN); tft.drawString("ONLINE", SCREEN_W - 5, 12); }
  else { tft.setTextColor(TFT_RED); tft.drawString("OFFLINE", SCREEN_W - 5, 12); }
}

void setup() {
  Serial.begin(115200); 
  delay(1000);
  tft.init(); tft.setRotation(3); tft.invertDisplay(true);
  SerialBT.begin("RS3-Dashboard", true); SerialBT.setPin(BT_PIN);
  drawDashboard(true);
}

void loop() {
  uint32_t now = millis();
  if (!SerialBT.connected()) {
    if (now - lastReconnect > 5000) {
      lastReconnect = now; btConnected = false; elmReady = false;
      SerialBT.connect(BT_MAC); needFullRedraw = true;
    }
  } else {
    if (!btConnected) btConnected = true;
    if (!elmReady && (now - lastReconnect > 2000)) { 
      elmReady = initELM(); 
      needFullRedraw = true; 
    }
  }
  if (elmReady) { queryGauge(currentGauge); currentGauge = (currentGauge + 1) % 6; }
  if (now - lastRedraw > 800) { lastRedraw = now; drawDashboard(needFullRedraw); needFullRedraw = false; }
  yield();
}
