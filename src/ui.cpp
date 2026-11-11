// ============================================================================
//  RS3 Dashboard - UI
//
//  Rendering-Konzept
//  -----------------
//  Der ESP32 hat neben Bluetooth-Classic + BLE nicht genug RAM fuer einen kompletten Framebuffer
//  (320x240x16 = 150 KB). Stattdessen wird das Bild in horizontalen Streifen (STRIP_H Zeilen)
//  zusammengesetzt: pro Streifen wird alles (Seite + ggf. Nachbarseite + Overlay) in einen kleinen
//  Sprite gezeichnet und dann auf einmal zum Display geschickt. Dadurch
//    * flackert nichts (jedes Pixel wird pro Frame genau einmal geschrieben),
//    * koennen Seiten beim Wischen live dem Finger folgen (zwei Seiten gleichzeitig sichtbar),
//    * ist der Code der Seiten trivial: sie zeichnen in "Seitenkoordinaten" (0..320 x 0..240),
//      die Zeichenhelfer rechnen Seitenversatz + Streifenversatz ein und ueberspringen alles,
//      was den aktuellen Streifen nicht beruehrt.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include "dashboard.h"

#define SCR_W        320
#define SCR_H        240
#define STRIP_H      30            // 240 / 30 = 8 Streifen, 19 KB RAM
#define PAGE_COUNT   4
#define CTP_I2C_ADDRESS 0x38
#define HIST_N       88            // Trendpunkte pro Gauge (1 pro Sekunde, 2 px breit)
#define PAGE_SPLASH  255

// ---------------------------------------------------------------------------
// Farben (RGB565)
// ---------------------------------------------------------------------------
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static const uint16_t C_BG    = rgb(5, 7, 12);
static const uint16_t C_CARD  = rgb(15, 19, 29);
static const uint16_t C_TRACK = rgb(34, 40, 56);
static const uint16_t C_TEXT  = rgb(238, 242, 250);
static const uint16_t C_DIM   = rgb(105, 116, 138);
static const uint16_t C_ACC   = rgb(0, 206, 255);
static const uint16_t C_OK    = rgb(0, 226, 150);
static const uint16_t C_COLD  = rgb(64, 156, 255);
static const uint16_t C_AMB   = rgb(255, 176, 0);
static const uint16_t C_BAD   = rgb(255, 62, 62);

static uint16_t mix(uint16_t a, uint16_t b, float t) {
  if (t <= 0.f) return a;
  if (t >= 1.f) return b;
  int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
  int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
  return (uint16_t)(((ar + (int)((br - ar) * t)) << 11) |
                    ((ag + (int)((bg - ag) * t)) << 5) |
                     (ab + (int)((bb - ab) * t)));
}

// ---------------------------------------------------------------------------
// Pro-Gauge Darstellung: Farbverlauf, Nachkommastellen, Alarmverhalten
//   -> hier koennen Schwellen/Farbzonen bequem angepasst werden.
// ---------------------------------------------------------------------------
struct Stop { float v; uint16_t c; };
static const Stop stOilP[]   = {{0.f, C_OK}, {7.f, C_OK}, {8.f, C_BAD}, {10.f, C_BAD}};
static const Stop stBoost[]  = {{-1.f, C_COLD}, {-0.15f, C_COLD}, {0.15f, C_OK}, {1.0f, C_OK},
                                {1.35f, C_AMB}, {1.5f, C_BAD}, {2.f, C_BAD}};
static const Stop stGear[]   = {{0.f, C_COLD}, {40.f, C_COLD}, {60.f, C_OK}, {95.f, C_OK},
                                {105.f, C_AMB}, {110.f, C_BAD}, {130.f, C_BAD}};
static const Stop stIntake[] = {{-20.f, C_OK}, {40.f, C_OK}, {55.f, C_AMB}, {60.f, C_BAD}, {80.f, C_BAD}};
static const Stop stTank[]   = {{0.f, C_BAD}, {10.f, C_AMB}, {20.f, C_OK}, {60.f, C_OK}};
static const Stop stBatt[]   = {{10.f, C_BAD}, {11.7f, C_BAD}, {12.1f, C_OK}, {14.9f, C_OK},
                                {15.3f, C_BAD}, {16.f, C_BAD}};
static const Stop stOilT[]   = {{0.f, C_COLD}, {60.f, C_COLD}, {80.f, C_OK}, {100.f, C_OK},
                                {105.f, C_AMB}, {110.f, C_BAD}, {130.f, C_BAD}};
static const Stop stAccent[] = {{0.f, C_ACC}, {1.f, rgb(120, 90, 255)}};

#define NSTOPS(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))
struct GaugeUi {
  const char* name;
  const char* shortName;
  const Stop* stops;
  uint8_t     nStops;
  uint8_t     decimals;
  bool        alertLow;    // Unterschreiten von warnLow ist ein Alarm (sonst nur "kalt" = blau)
  float       minSpan;     // minimale Skalenbreite im Trendgraph
};
static const GaugeUi gui[GAUGE_COUNT] = {
  {"OELDRUCK",   "OELDRUCK",  stOilP,   NSTOPS(stOilP),   1, true,  2.f},
  {"LADEDRUCK",  "LADEDRUCK", stBoost,  NSTOPS(stBoost),  2, false, 0.5f},
  {"GETRIEBE",   "GETRIEBE",  stGear,   NSTOPS(stGear),   0, false, 20.f},
  {"ANSAUGLUFT", "ANSAUG",    stIntake, NSTOPS(stIntake), 0, false, 20.f},
  {"TANK",       "TANK",      stTank,   NSTOPS(stTank),   1, true,  5.f},
  {"BATTERIE",   "BATTERIE",  stBatt,   NSTOPS(stBatt),   1, true,  1.f},
  {"OELTEMP",    "OELTEMP",   stOilT,   NSTOPS(stOilT),   0, false, 20.f},
};

// Welche Gauges zeigt welche Seite? (Bitmaske, fuer OBD-Abfragepriorisierung)
#define BIT(g) (1u << (g))
static const uint8_t pageMask[PAGE_COUNT] = {
  BIT(G_BOOST) | BIT(G_OIL_P) | BIT(G_OIL_T) | BIT(G_GEAR) | BIT(G_INTAKE),   // DRIVE
  BIT(G_OIL_P) | BIT(G_OIL_T),                                                 // MOTOR
  BIT(G_OIL_T) | BIT(G_GEAR) | BIT(G_INTAKE),                                  // TEMPERATUREN
  BIT(G_BATT)  | BIT(G_TANK),                                                  // FAHRZEUG
};

static uint16_t zone(uint8_t g, float v) {
  const Stop* s = gui[g].stops;
  uint8_t n = gui[g].nStops;
  if (v <= s[0].v) return s[0].c;
  for (uint8_t i = 1; i < n; i++) {
    if (v <= s[i].v) return mix(s[i - 1].c, s[i].c, (v - s[i - 1].v) / (s[i].v - s[i - 1].v));
  }
  return s[n - 1].c;
}
static uint16_t valColor(uint8_t g, float v) {
  uint16_t c = zone(g, v);
  return (c == C_OK) ? C_TEXT : c;
}
static float fracOf(uint8_t g, float v) {
  const Gauge& G = gauges[g];
  float f = (v - G.minVal) / (G.maxVal - G.minVal);
  return f < 0.f ? 0.f : (f > 1.f ? 1.f : f);
}
static bool inAlarm(uint8_t g) {
  const Gauge& G = gauges[g];
  if (!G.hasValue) return false;
  return G.value > G.warnHigh || (gui[g].alertLow && G.value < G.warnLow);
}
static void fmtValue(char* b, size_t n, uint8_t g, float v) {
  uint8_t d = gui[g].decimals;
  float p = (d == 0) ? 1.f : (d == 1 ? 10.f : 100.f);
  float r = roundf(v * p) / p;
  if (r == 0.f) r = 0.f;          // "-0.00" vermeiden
  snprintf(b, n, "%.*f", d, r);
}

// ---------------------------------------------------------------------------
// Zustand
// ---------------------------------------------------------------------------
static TFT_eSPI    tft;
static TFT_eSprite strip(&tft);
static bool        stripOk = false;

static float       disp[GAUGE_COUNT];            // weich nachgefuehrte Werte fuer Ringe/Balken
static int16_t     hist[GAUGE_COUNT][HIST_N];
static uint8_t     histHead = 0;
static uint32_t    histTick = 0, lastHist = 0;

static uint8_t     page = 0;                     // aktuell gefuehrte Seite
static float       slide = 0.f;                  // horizontaler Versatz der Seite in px (<0: Finger links)
static float       slideTarget = 0.f;
static bool        settling = false;
static volatile uint8_t visMask = 0;

static uint32_t    toastUntil = 0;
static bool        gBlink = false;               // Blinkphase fuer Warnungen (pro Frame konstant)
static int8_t      gAlert = -1;                  // erstes Gauge im Alarm (-1 = keins)
static uint32_t    lastSig = 0, lastFrame = 0, lastTick = 0;
static float       splashProgress = 0.f;

// aktuelle Zeichenposition (Seiten- zu Streifenkoordinaten)
static int16_t gOx = 0, gOy = 0;

// ---------------------------------------------------------------------------
// Zeichenhelfer (Seitenkoordinaten, mit Culling gegen den aktuellen Streifen)
// ---------------------------------------------------------------------------
static inline bool visY(int y, int h) { int yy = y - gOy; return yy + h > 0 && yy < STRIP_H; }
static inline bool visX(int x, int w) { int xx = x + gOx; return xx + w > 0 && xx < SCR_W; }

static void dRect(int x, int y, int w, int h, uint16_t c) {
  if (w <= 0 || h <= 0 || !visY(y, h) || !visX(x, w)) return;
  strip.fillRect(x + gOx, y - gOy, w, h, c);
}
static void dRRect(int x, int y, int w, int h, int r, uint16_t c) {
  if (!visY(y, h) || !visX(x, w)) return;
  strip.fillRoundRect(x + gOx, y - gOy, w, h, r, c);
}
static void dRRectO(int x, int y, int w, int h, int r, uint16_t c) {
  if (!visY(y, h) || !visX(x, w)) return;
  strip.drawRoundRect(x + gOx, y - gOy, w, h, r, c);
}
static void dText(const char* s, int x, int y, uint8_t font, uint8_t datum, uint16_t col,
                  int halfH, uint8_t size = 1) {
  if (!visY(y - halfH, halfH * 2)) return;
  strip.setTextSize(size);
  strip.setTextDatum(datum);
  strip.setTextColor(col);
  strip.drawString(s, x + gOx, y - gOy, font);
}
static void dTri(float x0, float y0, float x1, float y1, float x2, float y2, uint16_t c) {
  float lo = min(y0, min(y1, y2)), hi = max(y0, max(y1, y2));
  if (!visY((int)lo, (int)(hi - lo) + 2)) return;
  strip.fillTriangle(lroundf(x0) + gOx, lroundf(y0) - gOy, lroundf(x1) + gOx, lroundf(y1) - gOy,
                     lroundf(x2) + gOx, lroundf(y2) - gOy, c);
}
static void dLine(int x0, int y0, int x1, int y1, uint16_t c) {
  if (!visY(min(y0, y1), abs(y1 - y0) + 1)) return;
  strip.drawLine(x0 + gOx, y0 - gOy, x1 + gOx, y1 - gOy, c);
}
// Einheit zentriert; "C" bekommt ein Gradzeichen (die eingebauten Fonts haben keins).
static void dUnit(const char* u, int cx, int cy, uint8_t font, uint16_t col) {
  int fh = (font == 1) ? 8 : (font == 2 ? 16 : 26);
  if (!visY(cy - fh, fh * 2)) return;
  if (u[0] == 'C' && u[1] == 0) {
    int tw = strip.textWidth("C", font);
    int x0 = cx - (tw + 6) / 2;
    strip.drawCircle(x0 + 2 + gOx, cy - fh / 2 + 2 - gOy, 2, col);
    dText("C", x0 + 6, cy, font, ML_DATUM, col, fh / 2 + 1);
  } else {
    dText(u, cx, cy, font, MC_DATUM, col, fh / 2 + 1);
  }
}

// Ein Ringsegment (Kreisringausschnitt) von Winkel a0..a1 (Radiant, 0 = rechts, im Uhrzeigersinn).
static void dArcSeg(float cx, float cy, float a0, float a1, float rIn, float rOut, uint16_t col) {
  float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);
  float x0 = cx + c0 * rIn,  y0 = cy + s0 * rIn;
  float x1 = cx + c0 * rOut, y1 = cy + s0 * rOut;
  float x2 = cx + c1 * rOut, y2 = cy + s1 * rOut;
  float x3 = cx + c1 * rIn,  y3 = cy + s1 * rIn;
  dTri(x0, y0, x1, y1, x2, y2, col);
  dTri(x0, y0, x2, y2, x3, y3, col);
}

#define DEG2RAD 0.01745329252f
static const float ARC_START = 135.f * DEG2RAD;
static const float ARC_SWEEP = 270.f * DEG2RAD;

// 270-Grad-Segmentring. Segmente leuchten bis "f", Farbe laut Farbverlauf an der Segmentposition.
static void ringRaw(float cx, float cy, float rIn, float rOut, int segs, const Stop* st, uint8_t ns,
                    float vmin, float vmax, float f, bool dimOnly) {
  if (!visY((int)(cy - rOut - 10), (int)(2 * rOut + 20)) || !visX((int)(cx - rOut - 10), (int)(2 * rOut + 20))) return;
  float step = ARC_SWEEP / segs, gap = step * 0.16f;
  for (int i = 0; i < segs; i++) {
    float mid = (i + 0.5f) / segs;
    float v = vmin + mid * (vmax - vmin);
    uint16_t col = st[ns - 1].c;
    for (uint8_t k = 0; k < ns; k++) {
      if (v <= st[k].v) {
        col = (k == 0) ? st[0].c : mix(st[k - 1].c, st[k].c, (v - st[k - 1].v) / (st[k].v - st[k - 1].v));
        break;
      }
    }
    float lit = dimOnly ? 0.f : f * segs - i;           // 0..1: Helligkeit des (Teil-)Segments
    lit = lit < 0.f ? 0.f : (lit > 1.f ? 1.f : lit);
    uint16_t c = mix(mix(C_BG, col, 0.14f), col, lit);
    float a0 = ARC_START + i * step + gap * 0.5f;
    dArcSeg(cx, cy, a0, a0 + step - gap, rIn, rOut, c);
  }
}

static void ring(float cx, float cy, float rIn, float rOut, int segs, uint8_t g) {
  const Gauge& G = gauges[g];
  ringRaw(cx, cy, rIn, rOut, segs, gui[g].stops, gui[g].nStops, G.minVal, G.maxVal,
          G.hasValue ? fracOf(g, disp[g]) : 0.f, false);
  // Peak-Markierung ausserhalb des Rings
  if (G.trackMax && G.maxValue > -99.f && G.hasValue) {
    float a = ARC_START + fracOf(g, G.maxValue) * ARC_SWEEP;
    dArcSeg(cx, cy, a - 0.016f, a + 0.016f, rOut + 3, rOut + 9, C_TEXT);
  }
}

// Segmentierter Balken mit Farbverlauf und Peak-Strich.
static void gradBar(int x, int y, int w, int h, uint8_t g) {
  if (!visY(y - 4, h + 8) || !visX(x, w)) return;
  const Gauge& G = gauges[g];
  int fillW = G.hasValue ? (int)(fracOf(g, disp[g]) * w) : 0;
  int pxStart = max(0, -(x + gOx));
  pxStart -= pxStart % 4;
  int pxEnd = min(w, SCR_W - (x + gOx));
  for (int px = pxStart; px < pxEnd; px += 4) {
    float v = G.minVal + ((px + 2.f) / w) * (G.maxVal - G.minVal);
    uint16_t col = zone(g, v);
    uint16_t c = (px + 3 <= fillW) ? col : mix(C_CARD, col, 0.16f);
    dRect(x + px, y, min(3, w - px), h, c);
  }
  if (G.trackMax && G.maxValue > -99.f && G.hasValue) {
    int mx = x + (int)(fracOf(g, G.maxValue) * (w - 2));
    dRect(mx, y - 3, 2, h + 6, C_TEXT);
  }
}

// ---------------------------------------------------------------------------
// Trendverlauf
// ---------------------------------------------------------------------------
static void histPush() {
  for (uint8_t g = 0; g < GAUGE_COUNT; g++) {
    const Gauge& G = gauges[g];
    hist[g][histHead] = G.hasValue ? (int16_t)constrain(lroundf(G.value * 100.f), -32000, 32000) : INT16_MIN;
  }
  histHead = (histHead + 1) % HIST_N;
  histTick++;
}

static void sparkline(int x, int y, int w, int h, uint8_t g) {
  if (!visY(y, h) || !visX(x, w)) return;
  const Gauge& G = gauges[g];
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < HIST_N; i++) {
    if (hist[g][i] == INT16_MIN) continue;
    float v = hist[g][i] / 100.f;
    lo = min(lo, v); hi = max(hi, v);
  }
  if (G.hasValue) { lo = min(lo, G.value); hi = max(hi, G.value); }
  if (hi < lo) { dText("keine Daten", x + w / 2, y + h / 2, 1, MC_DATUM, C_DIM, 5); return; }
  float span = hi - lo;
  if (span < gui[g].minSpan) {
    float m = (hi + lo) * 0.5f;
    lo = m - gui[g].minSpan * 0.5f; hi = m + gui[g].minSpan * 0.5f; span = gui[g].minSpan;
  }
  lo -= span * 0.08f; hi += span * 0.08f; span = hi - lo;

  int px = -1, py = 0, lx = 0, ly = 0;
  uint16_t lc = C_ACC;
  for (int i = 0; i < HIST_N; i++) {
    int16_t raw = hist[g][(histHead + i) % HIST_N];
    if (raw == INT16_MIN) { px = -1; continue; }
    float v = raw / 100.f;
    int cx = x + i * 2;
    int cy = y + h - 1 - (int)((v - lo) / span * (h - 1));
    uint16_t col = zone(g, v);
    dRect(cx, cy, 2, y + h - cy, mix(C_CARD, col, 0.20f));
    if (px >= 0) { dLine(px, py, cx, cy, col); dLine(px, py + 1, cx, cy + 1, col); }
    px = cx; py = cy; lx = cx + 1; ly = cy; lc = col;
  }
  if (px >= 0 && visY(ly - 3, 7)) strip.fillCircle(lx + gOx, ly - gOy, 3, lc);
}

// ---------------------------------------------------------------------------
// Seiten
// ---------------------------------------------------------------------------
static void dTitle(const char* t) {
  dRect(10, 5, 3, 13, C_ACC);
  dText(t, 19, 11, 2, ML_DATUM, C_TEXT, 9);
}

// Kompakte Kachel mit Wert + Mini-Balken (DRIVE-Seite)
static void card(int x, int y, int w, int h, uint8_t g) {
  const Gauge& G = gauges[g];
  dRRect(x, y, w, h, 7, C_CARD);
  if (gBlink && inAlarm(g)) dRRectO(x, y, w, h, 7, C_BAD);
  dText(gui[g].shortName, x + w / 2, y + 11, 1, MC_DATUM, C_DIM, 5);
  char b[12];
  if (G.hasValue) {
    fmtValue(b, sizeof(b), g, G.value);
    dText(b, x + w / 2, y + 41, 4, MC_DATUM, valColor(g, G.value), 14);
  } else {
    dText("---", x + w / 2, y + 41, 4, MC_DATUM, C_TRACK, 14);
  }
  dUnit(G.unit, x + w / 2, y + 63, 1, C_DIM);
  gradBar(x + 7, y + h - 15, w - 14, 5, g);
}

// Grosser Ring mit Wert in der Mitte
static void ringBlock(float cx, float cy, float rIn, float rOut, int segs, uint8_t g,
                      int labelDy, int valDy, int unitDy, int peakDy) {
  const Gauge& G = gauges[g];
  ring(cx, cy, rIn, rOut, segs, g);
  dText(gui[g].name, (int)cx, (int)cy + labelDy, 2, MC_DATUM, C_DIM, 9);
  char b[12];
  if (G.hasValue) {
    fmtValue(b, sizeof(b), g, G.value);
    dText(b, (int)cx, (int)cy + valDy, 7, MC_DATUM, valColor(g, G.value), 26);
  } else {
    dText("---", (int)cx, (int)cy + valDy, 7, MC_DATUM, C_TRACK, 26);
  }
  dUnit(G.unit, (int)cx, (int)cy + unitDy, 2, C_ACC);
  if (G.trackMax && G.maxValue > -99.f) {
    char m[20], v[12];
    fmtValue(v, sizeof(v), g, G.maxValue);
    snprintf(m, sizeof(m), "MAX %s", v);
    dText(m, (int)cx, (int)cy + peakDy, 2, MC_DATUM, C_DIM, 9);
  }
}

static void pageDrive() {
  dTitle("DRIVE");
  const float cx = 160, cy = 124;
  ringBlock(cx, cy, 78, 94, 54, G_BOOST, -44, -6, 32, 72);

  // Skalenmarken: Nullpunkt (Atmosphaere) und Enden
  const Gauge& B = gauges[G_BOOST];
  float a0 = ARC_START + fracOf(G_BOOST, 0.f) * ARC_SWEEP;
  dArcSeg(cx, cy, a0 - 0.012f, a0 + 0.012f, 96, 102, C_DIM);
  dText("0", (int)(cx + cosf(a0) * 111), (int)(cy + sinf(a0) * 111), 1, MC_DATUM, C_DIM, 5);
  char lo[8], hi[8];
  snprintf(lo, sizeof(lo), "%g", B.minVal);
  snprintf(hi, sizeof(hi), "%g", B.maxVal);
  dText(lo, (int)cx - 70, (int)cy + 82, 1, MC_DATUM, C_DIM, 5);
  dText(hi, (int)cx + 70, (int)cy + 82, 1, MC_DATUM, C_DIM, 5);

  card(4, 28, 62, 94, G_OIL_T);
  card(4, 128, 62, 94, G_OIL_P);
  card(254, 28, 62, 94, G_GEAR);
  card(254, 128, 62, 94, G_INTAKE);
}

static void pageEngine() {
  dTitle("MOTOR");
  ringBlock(82, 122, 56, 70, 40, G_OIL_P, -37, -4, 31, 80);
  ringBlock(238, 122, 56, 70, 40, G_OIL_T, -37, -4, 31, 80);
  // Skalenenden
  for (int k = 0; k < 2; k++) {
    uint8_t g = (k == 0) ? G_OIL_P : G_OIL_T;
    int cx = (k == 0) ? 82 : 238;
    char lo[8], hi[8];
    snprintf(lo, sizeof(lo), "%g", gauges[g].minVal);
    snprintf(hi, sizeof(hi), "%g", gauges[g].maxVal);
    dText(lo, cx - 52, 182, 1, MC_DATUM, C_DIM, 5);
    dText(hi, cx + 52, 182, 1, MC_DATUM, C_DIM, 5);
  }
}

static void tempRow(int y, uint8_t g) {
  const Gauge& G = gauges[g];
  dRRect(6, y, 308, 64, 8, C_CARD);
  if (gBlink && inAlarm(g)) dRRectO(6, y, 308, 64, 8, C_BAD);
  dText(gui[g].name, 16, y + 8, 1, ML_DATUM, C_DIM, 5);
  char b[12];
  if (G.hasValue) {
    fmtValue(b, sizeof(b), g, G.value);
    dText(b, 112, y + 38, 7, MR_DATUM, valColor(g, G.value), 26);
  } else {
    dText("---", 112, y + 38, 7, MR_DATUM, C_TRACK, 26);
  }
  dUnit(G.unit, 119, y + 14, 1, C_ACC);
  if (G.trackMax && G.maxValue > -99.f) {
    char m[20], v[12];
    fmtValue(v, sizeof(v), g, G.maxValue);
    snprintf(m, sizeof(m), "MAX %s", v);
    dText(m, 306, y + 8, 1, MR_DATUM, C_DIM, 5);
  }
  sparkline(128, y + 16, HIST_N * 2, 42, g);
}

static void pageTemps() {
  dTitle("TEMPERATUREN");
  tempRow(26, G_OIL_T);
  tempRow(94, G_GEAR);
  tempRow(162, G_INTAKE);
}

static void bigBar(int y, uint8_t g, const char* tag, uint16_t tagCol) {
  const Gauge& G = gauges[g];
  dRRect(6, y, 308, 72, 8, C_CARD);
  if (gBlink && inAlarm(g)) dRRectO(6, y, 308, 72, 8, C_BAD);
  dText(gui[g].name, 16, y + 9, 1, ML_DATUM, C_DIM, 5);
  if (tag) dText(tag, 306, y + 9, 1, MR_DATUM, tagCol, 5);
  char b[12];
  if (G.hasValue) {
    fmtValue(b, sizeof(b), g, G.value);
    dText(b, 16, y + 41, 7, ML_DATUM, valColor(g, G.value), 26);
  } else {
    dText("---", 16, y + 41, 7, ML_DATUM, C_TRACK, 26);
  }
  dUnit(G.unit, 134, y + 54, 2, C_ACC);
  gradBar(160, y + 30, 146, 14, g);
  char lo[8], hi[8];
  snprintf(lo, sizeof(lo), "%g", G.minVal);
  snprintf(hi, sizeof(hi), "%g", G.maxVal);
  dText(lo, 160, y + 56, 1, ML_DATUM, C_DIM, 5);
  dText(hi, 306, y + 56, 1, MR_DATUM, C_DIM, 5);
}

static void statusChip(int x, int y, int w, const char* name, const char* state, uint16_t dot) {
  dRRect(x, y, w, 44, 8, C_CARD);
  if (visY(y + 10, 24) && visX(x, w)) strip.fillCircle(x + 14 + gOx, y + 15 - gOy, 4, dot);
  dText(name, x + 25, y + 15, 2, ML_DATUM, C_TEXT, 9);
  dText(state, x + 12, y + 33, 1, ML_DATUM, C_DIM, 5);
}

static void pageVehicle() {
  dTitle("FAHRZEUG");
  const Gauge& B = gauges[G_BATT];
  const char* bt = NULL; uint16_t bc = C_DIM;
  if (B.hasValue) {
    if (B.value > 13.2f)      { bt = "LADEND";  bc = C_OK; }
    else if (B.value < 12.0f) { bt = "SCHWACH"; bc = C_AMB; }
    else                      { bt = "RUHESPANNUNG"; bc = C_DIM; }
  }
  bigBar(26, G_BATT, bt, bc);

  const Gauge& T = gauges[G_TANK];
  bigBar(104, G_TANK, (T.hasValue && T.value < T.warnLow) ? "RESERVE" : NULL, C_AMB);

  bool obd = btConnected, elm = elmReady, app = bleClientConnected;
  statusChip(6,   182, 100, "OBD", obd ? "VERBUNDEN" : "SUCHE...", obd ? C_OK : C_AMB);
  statusChip(110, 182, 100, "ELM", elm ? "BEREIT" : (obd ? "INIT..." : "---"), elm ? C_OK : (obd ? C_AMB : C_BAD));
  statusChip(214, 182, 100, "APP", app ? "VERBUNDEN" : "WARTET", app ? C_ACC : C_TRACK);
}

static void pageSplash() {
  float p = splashProgress;
  const float cx = 160, cy = 112;
  ringRaw(cx, cy, 64, 78, 60, stAccent, 2, 0.f, 1.f, p, false);
  uint8_t a = (uint8_t)constrain((int)(p * 400), 0, 255);
  dText("RS3", (int)cx, (int)cy - 6, 1, MC_DATUM, mix(C_BG, C_TEXT, a / 255.f), 20, 4);
  dText("8V  DAZA", (int)cx, (int)cy + 28, 2, MC_DATUM, mix(C_BG, C_ACC, a / 255.f), 9);
  dText("DASHBOARD", (int)cx, 214, 2, MC_DATUM, mix(C_BG, C_DIM, a / 255.f), 9);
}

static void drawPage(uint8_t idx, int ox) {
  if (ox <= -SCR_W || ox >= SCR_W) return;
  gOx = ox;
  switch (idx) {
    case 0: pageDrive();   break;
    case 1: pageEngine();  break;
    case 2: pageTemps();   break;
    case 3: pageVehicle(); break;
    case PAGE_SPLASH: pageSplash(); break;
  }
}

// Statische Ebene ueber allen Seiten: Verbindungsstatus, Warnleiste, Toast, Seitenpunkte
static void drawOverlay() {
  gOx = 0;
  const uint32_t now = millis();

  if (visY(0, 24)) {
    bool toast = now < toastUntil;
    if (toast) {
      dRect(0, 0, SCR_W, 24, rgb(0, 60, 80));
      dText("PEAKS ZURUECKGESETZT", SCR_W / 2, 12, 2, MC_DATUM, C_ACC, 9);
    } else if (gAlert >= 0) {
      const Gauge& G = gauges[gAlert];
      dRect(0, 0, SCR_W, 24, gBlink ? rgb(190, 24, 24) : rgb(70, 0, 0));
      char v[12], t[40];
      fmtValue(v, sizeof(v), gAlert, G.value);
      snprintf(t, sizeof(t), "%s  %s", gui[gAlert].name, v);
      uint16_t tc = gBlink ? C_TEXT : C_BAD;
      int tw = strip.textWidth(t, 2);
      int x0 = SCR_W / 2 - (tw + 30) / 2;
      dTri(x0, 19, x0 + 7, 5, x0 + 14, 19, tc);                  // Warndreieck
      dRect(x0 + 6, 9, 2, 5, gBlink ? rgb(190, 24, 24) : rgb(70, 0, 0));
      dText(t, x0 + 22, 12, 2, ML_DATUM, tc, 9);
      dUnit(G.unit, x0 + 22 + tw + 8, 12, 2, tc);
    } else {
      dRect(222, 0, 98, 24, C_BG);                                // Seitentitel nicht unter den Status schieben
      const char* n[3] = {"OBD", "ELM", "APP"};
      bool on[3] = {btConnected, elmReady, bleClientConnected};
      uint16_t col[3] = {C_OK, C_OK, C_ACC};
      for (int i = 0; i < 3; i++) {
        int x = 228 + i * 31;
        if (visY(8, 8)) strip.fillCircle(x + 3, 12 - gOy, 3, on[i] ? col[i] : C_TRACK);
        dText(n[i], x + 9, 12, 1, ML_DATUM, on[i] ? C_TEXT : C_DIM, 5);
      }
    }
  }

  // Seitenpunkte: der aktive Punkt waechst zu einer Kapsel und folgt dem Finger
  if (visY(228, 12) && page != PAGE_SPLASH) {
    float pos = (float)page - slide / SCR_W;
    pos = fmodf(pos + PAGE_COUNT, PAGE_COUNT);
    for (int i = 0; i < PAGE_COUNT; i++) {
      float d = fabsf(i - pos);
      if (d > PAGE_COUNT / 2.f) d = PAGE_COUNT - d;
      float w = d >= 1.f ? 0.f : 1.f - d;
      int wd = 5 + (int)(w * 14);
      int xc = SCR_W / 2 + (int)((i - (PAGE_COUNT - 1) / 2.f) * 22);
      dRRect(xc - wd / 2, 232, wd, 5, 2, mix(C_TRACK, C_ACC, w));
    }
  }
}

static void renderFrame(bool splash) {
  if (!stripOk) return;
  uint8_t nb = 0; int nbx = 0;
  if (!splash && fabsf(slide) > 0.5f) {
    if (slide < 0) { nb = (page + 1) % PAGE_COUNT; nbx = (int)slide + SCR_W; }
    else           { nb = (page + PAGE_COUNT - 1) % PAGE_COUNT; nbx = (int)slide - SCR_W; }
  }
  for (int y0 = 0; y0 < SCR_H; y0 += STRIP_H) {
    gOy = y0;
    strip.fillSprite(C_BG);
    if (splash) {
      drawPage(PAGE_SPLASH, 0);
    } else {
      drawPage(page, (int)slide);
      if (nbx != 0) drawPage(nb, nbx);
      drawOverlay();
    }
    strip.pushSprite(0, y0);
  }
}

// ---------------------------------------------------------------------------
// Touch (FT6x06): Abtastung in eigenem Task, Gestenauswertung in ui_tick()
// ---------------------------------------------------------------------------
static bool ctpReady = false;
static portMUX_TYPE touchMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool    tPressed = false;
static volatile int16_t tX = 0, tY = 0;

static void touchTask(void* parameter) {
  (void)parameter;
  uint8_t releaseSamples = 0;
  for (;;) {
    if (ctpReady) {
      Wire.beginTransmission(CTP_I2C_ADDRESS);
      Wire.write(0x02);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom(CTP_I2C_ADDRESS, 5u) == 5u) {
        uint8_t touches = Wire.read() & 0x0F;
        uint8_t xh = Wire.read(), xl = Wire.read(), yh = Wire.read(), yl = Wire.read();
        uint16_t rawX = ((xh & 0x0F) << 8) | xl;
        uint16_t rawY = ((yh & 0x0F) << 8) | yl;
        if (touches > 0) {
          releaseSamples = 0;
          // Panel liefert Portrait-Koordinaten; Rotation 3 = Landscape 320x240
          int16_t x = constrain((int)rawY, 0, SCR_W - 1);
          int16_t y = constrain((SCR_H - 1) - (int)rawX, 0, SCR_H - 1);
          portENTER_CRITICAL(&touchMux);
          tX = x; tY = y; tPressed = true;
          portEXIT_CRITICAL(&touchMux);
        } else if (++releaseSamples >= 2) {     // Entprellen: erst nach 2 leeren Abtastungen loslassen
          releaseSamples = 2;
          tPressed = false;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void setupTouch() {
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
  xTaskCreatePinnedToCore(touchTask, "touch", 4096, NULL, 2, NULL, 0);
}

// Gestenzustand
static bool     prevPressed = false, dragging = false, longDone = false;
static uint32_t pressT = 0;
static int16_t  startX = 0, startY = 0, lastX = 0;
static float    velocity = 0.f;                 // px/ms

static void updateVisMask() {
  uint8_t m = pageMask[page];
  if (fabsf(slide) > 0.5f) {
    uint8_t nb = slide < 0 ? (page + 1) % PAGE_COUNT : (page + PAGE_COUNT - 1) % PAGE_COUNT;
    m |= pageMask[nb];
  }
  visMask = m;
}

static void startSettle(float target) { slideTarget = target; settling = true; dragging = false; }

static void resetPeaks() {
  for (uint8_t i = 0; i < GAUGE_COUNT; i++) if (gauges[i].trackMax) gauges[i].maxValue = -100.f;
}

static void handleTouch(uint32_t now, float dtMs) {
  bool pressed; int16_t x, y;
  portENTER_CRITICAL(&touchMux);
  pressed = tPressed; x = tX; y = tY;
  portEXIT_CRITICAL(&touchMux);

  if (settling) { prevPressed = pressed; return; }   // waehrend des Einrastens keine neue Geste

  if (pressed && !prevPressed) {
    pressT = now; startX = lastX = x; startY = y;
    dragging = false; longDone = false; velocity = 0.f;
  } else if (pressed) {
    int dx = x - startX, dy = y - startY;
    if (!dragging && abs(dx) > 10 && abs(dx) > abs(dy)) dragging = true;
    if (dragging) {
      slide = constrain((float)dx, -(SCR_W - 1.f), SCR_W - 1.f);
      float inst = (x - lastX) / max(dtMs, 1.f);
      velocity = velocity * 0.6f + inst * 0.4f;
    } else if (!longDone && now - pressT >= 700 && abs(dx) < 12 && abs(dy) < 12) {
      longDone = true;
      resetPeaks();
      toastUntil = now + 1400;
    }
    lastX = x;
  } else if (prevPressed) {                          // losgelassen
    int dx = lastX - startX;
    if (dragging) {
      if (slide < -70 || (slide < -25 && velocity < -0.4f))      startSettle(-SCR_W);
      else if (slide > 70 || (slide > 25 && velocity > 0.4f))    startSettle(SCR_W);
      else                                                       startSettle(0);
    } else if (!longDone && now - pressT < 350 && abs(dx) < 12 && abs(startY - y) < 12 && startY >= 212) {
      startSettle(startX < SCR_W / 2 ? SCR_W : -SCR_W);          // Tipp auf die Seitenpunkte
    }
  }
  prevPressed = pressed;
}

// ---------------------------------------------------------------------------
// Oeffentliche API
// ---------------------------------------------------------------------------
void ui_begin() {
  tft.init();
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(C_BG);
  stripOk = strip.createSprite(SCR_W, STRIP_H) != NULL;
  if (!stripOk) Serial.println("[UI] Streifen-Sprite konnte nicht angelegt werden (zu wenig RAM)!");
  for (uint8_t g = 0; g < GAUGE_COUNT; g++) {
    disp[g] = gauges[g].minVal;
    for (int i = 0; i < HIST_N; i++) hist[g][i] = INT16_MIN;
  }
  updateVisMask();
  setupTouch();
}

void ui_splash() {
  const uint32_t dur = 1100, t0 = millis();
  uint32_t el;
  while ((el = millis() - t0) < dur) {
    float p = (float)el / dur;
    splashProgress = 1.f - (1.f - p) * (1.f - p);     // ease-out
    renderFrame(true);
  }
  splashProgress = 1.f;
  lastTick = millis();
  lastSig = 0;
}

uint8_t ui_visibleMask() { return visMask; }

void ui_tick() {
  const uint32_t now = millis();
  float dt = (float)(now - lastTick);
  if (dt < 1.f) return;
  dt = min(dt, 100.f);
  lastTick = now;

  // 1. Trenddaten (1 Hz)
  if (now - lastHist >= 1000) { lastHist = now; histPush(); }

  // 2. Touch / Gesten / Einrasten
  handleTouch(now, dt);
  bool moving = dragging || settling;
  if (settling) {
    float diff = slideTarget - slide;
    float step = diff * (1.f - expf(-dt / 55.f));
    if (fabsf(step) < 18.f) step = (diff > 0 ? 18.f : -18.f);   // Mindestgeschwindigkeit
    if (fabsf(step) >= fabsf(diff)) {
      slide = slideTarget;
      settling = false;
      if (slideTarget < 0)      page = (page + 1) % PAGE_COUNT;
      else if (slideTarget > 0) page = (page + PAGE_COUNT - 1) % PAGE_COUNT;
      slide = 0.f;
      slideTarget = 0.f;
    } else {
      slide += step;
    }
  }
  updateVisMask();

  // 3. Anzeigewerte weich nachfuehren
  bool moved = false;
  for (uint8_t g = 0; g < GAUGE_COUNT; g++) {
    if (!(visMask & BIT(g))) continue;
    const Gauge& G = gauges[g];
    float tgt = G.hasValue ? G.value : G.minVal;
    float diff = tgt - disp[g];
    if (fabsf(diff) < (G.maxVal - G.minVal) * 0.002f) {
      if (diff != 0.f) { disp[g] = tgt; moved = true; }
    } else {
      disp[g] += diff * (1.f - expf(-dt / 80.f));
      moved = true;
    }
  }

  // 4. Warnstatus (blinkt mit ~2 Hz)
  gAlert = -1;
  for (uint8_t g = 0; g < GAUGE_COUNT; g++) if (inAlarm(g)) { gAlert = g; break; }
  gBlink = ((now / 300) & 1) == 0;

  // 5. Muss neu gezeichnet werden?
  uint32_t sig = 2166136261u;
  #define MIXSIG(x) sig = (sig ^ (uint32_t)(x)) * 16777619u
  MIXSIG(page); MIXSIG(visMask);
  MIXSIG((btConnected ? 1 : 0) | (elmReady ? 2 : 0) | (bleClientConnected ? 4 : 0));
  MIXSIG(gAlert + 1); MIXSIG(gAlert >= 0 ? gBlink : 0); MIXSIG(now < toastUntil);
  if (page == 2) MIXSIG(histTick);
  for (uint8_t g = 0; g < GAUGE_COUNT; g++) {
    if (!(visMask & BIT(g))) continue;
    const Gauge& G = gauges[g];
    float p = gui[g].decimals == 0 ? 1.f : (gui[g].decimals == 1 ? 10.f : 100.f);
    MIXSIG(G.hasValue);
    if (G.hasValue) MIXSIG(lroundf(G.value * p));
    if (G.trackMax) MIXSIG(lroundf(G.maxValue * p));
  }
  #undef MIXSIG

  if (!(moving || moved || sig != lastSig)) return;
  if (now - lastFrame < 15) return;
  lastFrame = now;
  lastSig = sig;
  renderFrame(false);
}
