#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Gemeinsame Datenstruktur zwischen OBD/BLE (main.cpp) und Display (ui.cpp)
// ---------------------------------------------------------------------------
#define GAUGE_COUNT     9   // alle Messwerte (Display)
#define BLE_GAUGE_COUNT 7   // davon im BLE-Paket an die App (Protokoll bleibt unveraendert!)

// Indizes in gauges[] (Reihenfolge der ersten 7 ist Teil des BLE-Protokolls!)
enum GaugeId : uint8_t {
  G_OIL_P  = 0,   // Oeldruck
  G_BOOST  = 1,   // Ladedruck
  G_GEAR   = 2,   // Getriebetemperatur
  G_INTAKE = 3,   // Ansauglufttemperatur
  G_TANK   = 4,   // Tankinhalt
  G_BATT   = 5,   // Batteriespannung
  G_OIL_T  = 6,   // Oeltemperatur
  G_HALDEX = 7,   // Haldex-Temperatur (nur Display)
  G_CAT    = 8    // Katalysatortemperatur B1S1 (nur Display)
};

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
  uint32_t lastUpdate;      // millis() der letzten gueltigen Messung
};

extern Gauge gauges[GAUGE_COUNT];
extern volatile bool btConnected;
extern volatile bool elmReady;
extern volatile bool bleClientConnected;

// ---------------------------------------------------------------------------
// UI (ui.cpp)
// ---------------------------------------------------------------------------
void    ui_begin();        // Display, Touch und Framebuffer-Streifen initialisieren (vor BT/BLE aufrufen!)
void    ui_splash();       // kurze Startanimation (blockiert ~1 s, rendert nur auf Core 1)
void    ui_tick();         // aus loop() aufrufen: Touch, Animation, Rendern
uint16_t ui_visibleMask(); // Bitmaske der Gauges, die gerade sichtbar sind (fuer OBD-Abfragepriorisierung)
