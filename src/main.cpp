#include <Arduino.h>
#include "dashboard.h"
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

BluetoothSerial SerialBT;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
volatile bool bleClientConnected = false;
uint32_t lastBleNotify = 0;



// 7 Gauges: Indices 0..5 TFT, Index 6 BLE (Hintergrund)
Gauge gauges[GAUGE_COUNT] = {
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

// Debug-Ausgaben der OBD-/BLE-Daten (Serial.printf blockiert ~10 ms pro Zeile).
#define DEBUG_DATA 0
#if DEBUG_DATA
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...) do {} while (0)
#endif

// Werte, die laenger als STALE_MS nicht aktualisiert wurden, gelten als ungueltig ("---").
#define STALE_MS 6000
#define OBD_RETRY_GAP_MS  3000   // Pause zwischen zwei OBD-Verbindungsversuchen (Zeitfenster fuer BLE)
#define BLE_NOTIFY_MIN_MS 100   // max. 10 Pakete/s bei neuen Daten
#define BLE_HEARTBEAT_MS  1000   // Lebenszeichen auch ohne neue Daten
#define PACKET_VERSION    0x01

// Binaeres BLE-Paket (little endian, 18 Byte -> passt auch in die Default-MTU von 23).
// v[i] = Wert von Gauge i * 100 (int16), Reihenfolge wie in gauges[].
// validMask: Bit i gesetzt = Gauge i hat einen aktuellen Wert.
// status: Bit0 = ELM bereit, Bit1 = OBD-Bluetooth verbunden.
struct __attribute__((packed)) BlePacket {
  uint8_t version;
  uint8_t seq;
  uint8_t validMask;
  uint8_t status;
  int16_t v[GAUGE_COUNT];
};
static_assert(sizeof(BlePacket) == 18, "BlePacket muss 18 Byte gross sein");

// Abfrage-Reihenfolge nach Header gruppiert (7DF -> 7E0 -> 7E1), damit weniger ATSH noetig sind.
// "schnell" = jeder Zyklus, "langsam" = nur jeder SLOW_EVERY-te Zyklus (aendert sich kaum).
#define SLOW_EVERY 5
const uint8_t pollOrder[GAUGE_COUNT] = {1, 3, 4, 5, 0, 6, 2};
const bool    pollSlow[GAUGE_COUNT]  = {false, false, false, true, true, true, false};

volatile bool btConnected = false, elmReady = false;
volatile bool dataDirty = false;
bool useRespCount = true;     // ELM-Antwortanzahl anhaengen ("010B1"); wird bei "?" automatisch abgeschaltet
uint8_t blePacketSeq = 0;
uint32_t lastSessionKeepAlive = 0, btConnectTime = 0;
volatile uint32_t bleDisconnectedAt = 0;
uint32_t activeHeader = 0x000;
TaskHandle_t obdTaskHandle = NULL;
char rawBuf[128];
float smoothedVolt = 0.0f;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
      bleClientConnected = true;
      bleDisconnectedAt = 0;
      dataDirty = true;
      // Verbindungsparameter: 30-60 ms Intervall, kein Slave-Latency, 4 s Supervision-Timeout.
      // Schnell genug fuer ~10 Hz Daten, aber deutlich stromsparender als der Android-Default
      // und robust gegen kurze Funkaussetzer im Hintergrundbetrieb.
      pServer->updateConnParams(param->connect.remote_bda, 0x18, 0x30, 0, 400);
      Serial.printf("[BLE] Client verbunden! Conn ID: %d\n", param->connect.conn_id);
    }
    void onDisconnect(BLEServer* pServer) override {
      bleClientConnected = false;
      bleDisconnectedAt = millis();
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
    } else {
      vTaskDelay(1);   // Task abgeben statt busy-wait, solange der ELM noch nicht antwortet
    }
  }
  return false;
}

bool initELM() {
  Serial.println("Initialisiere ELM327...");
  vTaskDelay(pdMS_TO_TICKS(300));

  if(!elmRawCmd("ATZ", 1200)) {
    Serial.println("ATZ keine Antwort.");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(200));

  elmRawCmd("ATE0", 250);
  elmRawCmd("ATL0", 250);   // keine Linefeeds -> weniger Bytes pro Antwort
  elmRawCmd("ATAT1", 250);  // adaptives Timing: ELM wartet nur so lange wie das Steuergeraet braucht
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

// Liefert true, wenn eine auswertbare Antwort kam (Timeout/Parsefehler = false).
bool queryGauge(uint8_t idx) {
  if (!btConnected || !elmReady) return false;
  const PIDDef& def = pidDefs[idx];
  char cmd[32];

  if (activeHeader != def.header) {
    snprintf(cmd, sizeof(cmd), "ATSH%03X", (uint16_t)def.header);
    elmRawCmd(cmd, 150);
    activeHeader = def.header;
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (def.mode == 22 && (millis() - lastSessionKeepAlive > 2000)) {
    elmRawCmd("1003", 200);
    lastSessionKeepAlive = millis();
  }

  // Angehaengte "1" = "genau 1 Antwort erwartet": der ELM antwortet sofort nach der ersten
  // Antwort, statt seinen vollen Timeout abzuwarten. Wird der Befehl mit "?" quittiert
  // (aeltere Klone), schalten wir das Feature ab und senden ohne Suffix.
  const char* suffix = useRespCount ? "1" : "";
  if (def.mode == 1) snprintf(cmd, sizeof(cmd), "01%02X%s", (uint8_t)def.pid, suffix);
  else               snprintf(cmd, sizeof(cmd), "22%04X%s", def.pid, suffix);

  if (!elmRawCmd(cmd, (idx <= 1 ? 600 : 300))) {
    DBG("[OBD TIMEOUT] Befehl: %s | Keine Antwort vom ELM327\n", cmd);
    return false;
  }

  if (useRespCount && strchr(rawBuf, '?')) {
    useRespCount = false;
    Serial.println("[OBD] ELM kennt Antwortanzahl-Suffix nicht -> deaktiviert");
    return false;
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
    if (strlen(p) < 2) return false;

    if (idx == 0 && def.pid == 0x13F4) { // Oeldruck: real ECU payloads are not a direct 0.01 bar integer.
      uint16_t raw16 = ((uint16_t)hexByte(p) << 8) | hexByte(p + 2);
      gauges[idx].value = decodeOilPressureBar(raw16);
      DBG("[OIL PRESSURE RAW] DID 13F4: %04X -> %.2f bar (normalized 32768-based decode)\n",
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
    gauges[idx].lastUpdate = millis();
    dataDirty = true;

    DBG("[OBD RX] %-11s | Raw: %-20s | Wert: %.1f %s\n",
        gauges[idx].label, rawBuf, gauges[idx].value, gauges[idx].unit);
    return true;
  }

  DBG("[OBD PARSE ERR] %-11s | Match '%s' nicht in Raw '%s' gefunden\n",
      gauges[idx].label, search, rawBuf);
  return false;
}

// Werte ohne frische Messung ausblenden, damit App und Display nie "eingefrorene" Zahlen als live zeigen.
void expireStaleGauges(uint32_t now) {
  for (uint8_t i = 0; i < GAUGE_COUNT; i++) {
    if (gauges[i].hasValue && (now - gauges[i].lastUpdate) > STALE_MS) {
      gauges[i].hasValue = false;
      dataDirty = true;
    }
  }
}

// OBD-Task: haelt die Bluetooth-Classic-Verbindung zum ELM327 und fragt alle Werte zyklisch ab.
// Laeuft komplett getrennt von Display und BLE, damit blockierende Aufrufe (connect, Timeouts)
// weder die Anzeige noch die App-Benachrichtigungen aufhalten.
void obdTask(void* parameter) {
  (void)parameter;
  uint32_t cycle = 0;
  uint8_t failStreak = 0;

  for (;;) {
    if (!SerialBT.connected()) {
      btConnected = false;
      elmReady = false;
      activeHeader = 0;
      dataDirty = true;
      Serial.println("Verbinde mit ELM327 BT...");
      SerialBT.connect(BT_MAC);            // blockiert mehrere Sekunden (Paging belegt das gemeinsame Funkmodul)
      if (!SerialBT.connected()) {
        // OBD hat Vorrang: es wird dauerhaft erneut gepaged. In dieser Luecke ist das Funkmodul
        // aber frei, damit die App per BLE (Advertising alle 20-40 ms) verbinden kann.
        // Ist die BLE-Verbindung einmal aufgebaut, laeuft sie dank 4 s Supervision-Timeout weiter.
        vTaskDelay(pdMS_TO_TICKS(OBD_RETRY_GAP_MS));
        continue;
      }
    }

    if (!btConnected) {
      btConnected = true;
      btConnectTime = millis();
      dataDirty = true;
    }

    if (!elmReady) {
      uint32_t sinceConnect = millis() - btConnectTime;
      if (sinceConnect < 2000) vTaskDelay(pdMS_TO_TICKS(2000 - sinceConnect));
      elmReady = initELM();
      failStreak = 0;
      dataDirty = true;
      if (!elmReady) vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    bool includeSlow = (cycle % SLOW_EVERY) == 0;
    // Ohne App nur die Werte der sichtbaren Seite jeden Zyklus abfragen (schnellere Updates),
    // alle anderen nur jeden 4. Zyklus -> Warnungen bleiben global aktiv, Werte werden nicht "stale".
    const bool bgCycle = (cycle % 4) == 0;
    const uint8_t visible = ui_visibleMask();
    for (uint8_t n = 0; n < GAUGE_COUNT && elmReady; n++) {
      uint8_t idx = pollOrder[n];
      if (pollSlow[idx] && !includeSlow) continue;
      if (!bleClientConnected && !(visible & (1u << idx)) && !bgCycle) continue;

      if (queryGauge(idx)) {
        failStreak = 0;
      } else if (++failStreak >= 12) {      // ELM antwortet gar nicht mehr -> neu initialisieren
        Serial.println("[OBD] Zu viele Fehler in Folge -> ELM neu initialisieren");
        elmReady = false;
        break;
      }
      if (!SerialBT.connected()) break;
      vTaskDelay(bleClientConnected ? 1 : pdMS_TO_TICKS(20));
    }
    cycle++;
    vTaskDelay(1);
  }
}

static int16_t toFixed100(float x) {
  float v = x * 100.0f;
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)lroundf(v);
}

void sendBleData() {
  if (!bleClientConnected) return;

  BlePacket pkt;
  pkt.version = PACKET_VERSION;
  pkt.seq = blePacketSeq++;
  pkt.validMask = 0;
  pkt.status = (elmReady ? 0x01 : 0) | (btConnected ? 0x02 : 0);
  for (uint8_t i = 0; i < GAUGE_COUNT; i++) {
    bool ok = gauges[i].hasValue;
    if (ok) pkt.validMask |= (1 << i);
    pkt.v[i] = ok ? toFixed100(gauges[i].value) : 0;
  }

  pCharacteristic->setValue((uint8_t*)&pkt, sizeof(pkt));
  pCharacteristic->notify();

  DBG("[BLE TX] seq=%u mask=%02X\n", pkt.seq, pkt.validMask);
}

void setupBLE() {
  BLEDevice::init("RS3-Dashboard-BLE");
  BLEDevice::setMTU(247);   // grosse MTU erlauben; Pakete sind aber nur 18 Byte und passen auch in die Default-MTU
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
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
  delay(300);
  ui_begin();   // vor Bluetooth, solange noch genug zusammenhaengender RAM fuer den Framebuffer-Streifen frei ist

  SerialBT.begin("RS3-Dashboard", true);
  SerialBT.setPin(BT_PIN);

  setupBLE();

  // OBD-Abfrage in eigenem Task auf Core 0 (BT-Stack), Display + BLE bleiben in loop() auf Core 1.
  xTaskCreatePinnedToCore(obdTask, "obd", 6144, NULL, 1, &obdTaskHandle, 0);

  ui_splash();   // Startanimation, waehrend im Hintergrund schon BT/BLE hochlaufen
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastBleNotify = 0;

  // 1. Veraltete Messwerte ausblenden
  expireStaleGauges(now);

  // 2. BLE: nach Verbindungsabbruch Advertising nicht-blockierend neu starten
  if (bleDisconnectedAt != 0 && !bleClientConnected && (now - bleDisconnectedAt) > 300) {
    bleDisconnectedAt = 0;
    pServer->startAdvertising();
    Serial.println("[BLE] Client getrennt -> Advertising neu gestartet!");
  }

  // 3. BLE-Daten: bei neuen Werten (max. 10/s), sonst als Heartbeat jede Sekunde
  if (bleClientConnected) {
    uint32_t sinceLast = now - lastBleNotify;
    if ((dataDirty && sinceLast >= BLE_NOTIFY_MIN_MS) || sinceLast >= BLE_HEARTBEAT_MS) {
      dataDirty = false;
      lastBleNotify = now;
      sendBleData();
    }
  }

  // 4. Display: Touch, Animationen, Rendern (zeichnet nur, wenn sich etwas geaendert hat)
  ui_tick();

  vTaskDelay(pdMS_TO_TICKS(5));
}
