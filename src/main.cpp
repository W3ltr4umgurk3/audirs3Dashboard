#include <Arduino.h>
#include <TFT_eSPI.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth SPP nicht aktiviert! Board-Partition prüfen."
#endif

// --- Bluetooth Setup ---
uint8_t     BT_MAC[6] = {0x13, 0xE0, 0x2F, 0x8D, 0x64, 0x43};
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
  float  warnLow;
  float  warnHigh;
  float  minVal;
  float  maxVal;
  bool   hasValue;
};

Gauge gauges[6] = {
  {"Oeldruck",    "bar", 0.f,  0.5f,  6.0f,   0.f,  8.0f, false}, 
  {"Tankinhalt",  "L",   0.f,  5.0f,  60.0f,  0.f,  55.0f, false}, 
  {"Getriebe",    "C",   0.f, 20.0f, 110.0f,  0.f, 130.0f, false}, 
  {"Ansaugtemp",  "C",   0.f, -20.f,  60.0f, -20.f, 80.0f, false}, 
  {"Kuehlwasser", "C",   0.f, 50.0f, 115.0f,  0.f, 130.0f, false}, 
  {"Batterie",    "V",   0.f, 11.5f, 15.5f,  10.f, 16.0f, false}  
};

struct PIDDef { uint8_t mode; uint16_t pid; uint16_t header; };
const PIDDef pidDefs[6] = {
  {22, 0x13F4, 0x7E0},   
  { 1, 0x2F,   0x7DF},   
  {22, 0x2104, 0x7E1},   
  { 1, 0x0F,   0x7DF},   
  { 1, 0x05,   0x7DF},   
  { 1, 0x42,   0x7DF}    
};

bool btConnected = false, elmReady = false, needFullRedraw = true;
uint32_t lastReconnect = 0, lastRedraw = 0, lastSessionRefresh = 0;
uint8_t currentGauge = 0;
char rawBuf[128];

bool elmRawCmd(const char* cmd, uint16_t pidLabel = 0, uint16_t timeoutMs = 800) {
  while (SerialBT.available()) SerialBT.read();
  SerialBT.print(cmd); SerialBT.print("\r");
  unsigned long start = millis();
  int idx = 0;
  memset(rawBuf, 0, sizeof(rawBuf));
  while (millis() - start < timeoutMs) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') {
        if (pidLabel > 0) Serial.printf("[PID 0x%04X]: %s\n", pidLabel, rawBuf);
        else              Serial.printf("[SYS]: %s\n", rawBuf);
        return (idx > 0);
      }
      if (idx < 126 && c >= ' ') rawBuf[idx++] = c;
    }
    yield();
  }
  return (idx > 0);
}

bool initELM() {
  elmRawCmd("ATZ", 0, 2000); delay(500);
  elmRawCmd("ATE0", 0); elmRawCmd("ATS0", 0);
  elmRawCmd("ATH1", 0); elmRawCmd("ATSP6", 0);
  elmRawCmd("ATCAF1", 0); 
  return true;
}

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

void queryGauge(uint8_t idx) {
  if (!btConnected || !elmReady) return;
  const PIDDef& def = pidDefs[idx];
  char cmd[32];
  
  snprintf(cmd, sizeof(cmd), "ATSH%03X", def.header);
  elmRawCmd(cmd, 0, 100);

  if (def.header == 0x7E0 && def.mode == 22 && (millis() - lastSessionRefresh > 4000)) {
    elmRawCmd("1003", 0, 200); 
    lastSessionRefresh = millis();
  }

  if (def.mode == 1) snprintf(cmd, sizeof(cmd), "01%02X1", def.pid);
  else               snprintf(cmd, sizeof(cmd), "22%04X1", def.pid);

  if (!elmRawCmd(cmd, def.pid, 1000)) { gauges[idx].hasValue = false; return; }
  
  char search[10];
  if (def.mode == 1) sprintf(search, "41%02X", (uint8_t)def.pid);
  else               sprintf(search, "62%04X", def.pid);
  
  // Suche nach dem LETZTEN Vorkommen der Kennung (wegen Pending Frames)
  char* p = NULL;
  char* nextP = strstr(rawBuf, search);
  while(nextP != NULL) {
    p = nextP;
    nextP = strstr(p + 1, search);
  }

  if (p) {
    p += strlen(search);
    while(*p == ' ') p++;
    
    uint8_t a = hexByte(p); 
    uint8_t b = hexByte(p + 2);

    if (def.mode == 1) {
      if (def.pid == 0x42) gauges[idx].value = (float)(a * 256 + b) / 1000.0f;
      else if (def.pid == 0x2F) gauges[idx].value = ((float)a * 100.0f / 255.0f) * 0.55f;
      else gauges[idx].value = (float)a - 40.0f;
    } else {
      if (def.pid == 0x2104) gauges[idx].value = (float)a; 
      else if (def.pid == 0x13F4) {
        float val16 = (float)(a * 256 + b);
        // ABSOLUTDRUCK: ca. 1.02 bar bei Zündung AN
        gauges[idx].value = val16 / 31100.0f; 
        if (gauges[idx].value < 0.0f) gauges[idx].value = 0.0f;
      }
      else gauges[idx].value = (float)a - 40.0f; 
    }
    gauges[idx].hasValue = true;
  } else { gauges[idx].hasValue = false; }
}

void drawTile(uint8_t i) {
  const Gauge& g = gauges[i];
  int16_t tx = (i % COLS) * TILE_W, ty = HEADER_H + (i / COLS) * TILE_H;
  int16_t cx = tx + TILE_W / 2, cy = ty + TILE_H / 2;
  tft.fillRect(tx + 2, ty + 4, TILE_W - 4, TILE_H - 8, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW); tft.setTextSize(1);
  tft.setCursor(tx + 5, ty + 7); tft.print(g.label);
  tft.setTextDatum(MC_DATUM); tft.setTextSize(3);
  uint16_t vCol = TFT_WHITE;
  if (g.hasValue) {
    char buf[12];
    if (i == 0 || i == 1 || i == 5) dtostrf(g.value, 3, 1, buf); 
    else snprintf(buf, sizeof(buf), "%d", (int)roundf(g.value));
    if (g.value < g.warnLow || g.value > g.warnHigh) vCol = TFT_RED;
    tft.setTextColor(vCol);
    tft.drawString(buf, cx, cy - 5);
  } else {
    tft.setTextColor(TFT_DARKGREY); tft.drawString("---", cx, cy - 5);
  }
  tft.setTextSize(1); tft.setTextColor(TFT_CYAN);
  tft.drawString(g.unit, cx, cy + 18);
  int16_t bX = tx + 8, bY = ty + TILE_H - 12, bW = TILE_W - 16, bH = 5;
  tft.drawRect(bX, bY, bW, bH, TFT_DARKGREY);
  if (g.hasValue) {
    float pct = constrain((g.value - g.minVal) / (g.maxVal - g.minVal), 0.0f, 1.0f);
    int16_t fillW = pct * (bW - 2);
    tft.fillRect(bX + 1, bY + 1, fillW, bH - 2, (vCol == TFT_RED) ? TFT_RED : TFT_GREEN);
  }
}

void drawDashboard(bool full) {
  if (full) {
    tft.fillScreen(TFT_BLACK);
    for (uint8_t i = 0; i < 6; i++) {
      int16_t tx = (i % COLS) * TILE_W, ty = HEADER_H + (i / COLS) * TILE_H;
      tft.drawRect(tx, ty, TILE_W, TILE_H, TFT_RED);
      tft.fillRect(tx, ty, TILE_W, 3, TFT_RED);
    }
  }
  for (uint8_t i = 0; i < 6; i++) drawTile(i);
  tft.fillRect(0, 0, SCREEN_W, HEADER_H, TFT_BLACK);
  tft.setTextSize(1); tft.setTextColor(TFT_WHITE);
  tft.setCursor(5, 5); tft.print("Audi RS3 8V DAZA");
  tft.setTextDatum(TR_DATUM);
  if (elmReady) { tft.setTextColor(TFT_GREEN); tft.drawString("VERBUNDEN", SCREEN_W - 5, 12); }
  else if (btConnected) { tft.setTextColor(TFT_YELLOW); tft.drawString("ELM...", SCREEN_W - 5, 12); }
  else { tft.setTextColor(TFT_RED); tft.drawString("GETRENNT", SCREEN_W - 5, 12); }
}

void setup() {
  Serial.begin(115200);
  tft.init(); tft.setRotation(3); tft.invertDisplay(true);
  SerialBT.begin("RS3-Dashboard", true);
  SerialBT.setPin(BT_PIN);
  drawDashboard(true);
}

void loop() {
  uint32_t now = millis();
  if (!SerialBT.connected()) {
    if (now - lastReconnect > 5000) {
      lastReconnect = now; btConnected = false; elmReady = false;
      SerialBT.connect(BT_MAC);
      needFullRedraw = true;
    }
  } else {
    if (!btConnected) { btConnected = true; needFullRedraw = true; }
    if (!elmReady && (now - lastReconnect > 2000)) {
      elmReady = initELM();
      needFullRedraw = true;
    }
  }
  if (elmReady) { queryGauge(currentGauge); currentGauge = (currentGauge + 1) % 6; }
  if (now - lastRedraw > 1500) { lastRedraw = now; drawDashboard(needFullRedraw); needFullRedraw = false; }
  yield();
}