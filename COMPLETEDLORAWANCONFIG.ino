#include <cstdint>

/**
 * RAK3112 — ESP32-S3 + SX1262 (SPI) — LoRaWAN config portal + live log
 *
 * NO AT commands. LoRaWAN MAC + PHY over SPI via RadioLib.
 *
 * IMPORTANT (vs raw Semtech Radio API):
 *   Your snippet (lora_rak3112_init, Radio.Init, SetTx/RxConfig) configures ONLY the
 *   radio PHY. TTN requires the LoRaWAN MAC (join, encryption, frame counters, ADR).
 *   This firmware uses RadioLib::SX1262 + RadioLoRaWAN::LoRaWANNode for a real OTAA/ABP
 *   session — still SPI to the SX1262, still the same pins.
 *
 * Arduino IDE libraries:
 *   - RadioLib (Jan Gromeš) — recent version with LoRaWAN examples
 *   - ArduinoJson
 *
 * TTN console (LoRaWAN 1.0.x):
 *   - OTAA: enable "Resets join nonces" while testing, or keep NVS session (this sketch saves nonces/session).
 *   - ABP: enable "Resets frame counters" while testing without full persistence, or rely on saved session buffer.
 *
 * Single UART (USB CDC): debug + downlink summaries both go to Serial and to the web live log.
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ─── Pins (RAK3112 internal SX1262 SPI) ───────────────────────────────────────
#ifndef LORA_SX126X_SCK
#define LORA_SX126X_SCK     5
#define LORA_SX126X_MISO    3
#define LORA_SX126X_MOSI    6
#define LORA_SX126X_CS      7
#define LORA_SX126X_RESET   8
#define LORA_SX126X_DIO1    47
#define LORA_SX126X_BUSY    48
#endif
#ifndef PIN_LED1
#define PIN_LED1  46
#define PIN_LED2  45
#endif

// --- RadioLib ---
#include <RadioLib.h>
Module   sx1262Module(LORA_SX126X_CS, LORA_SX126X_DIO1, LORA_SX126X_RESET, LORA_SX126X_BUSY, SPI);
SX1262   radio(&sx1262Module);
LoRaWANNode* lorawan = nullptr;
static const LoRaWANBand_t* lwBand = &EU868;
static uint8_t lwSubBand = 0;

// --- Session keys (filled from config) ---
uint8_t binJoinEUI[8];
uint8_t binDevEUI[8];
uint8_t binAppKey[16];
uint8_t binNwkKey[16];
uint8_t binNwkSKey[16];
uint8_t binAppSKey[16];
uint32_t binDevAddr = 0;

// ...existing code...

// ─── WiFi AP ─────────────────────────────────────────────────────────────────
static const char* AP_SSID     = "RAK3112-LoRaWAN";
static const char* AP_PASSWORD = "";   // min 8 chars
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_SN(255, 255, 255, 0);


// Remove LMIC pinmap and runtime flags

// ─── Runtime flags ───────────────────────────────────────────────────────────
bool     g_radioPhyReady = false;
bool     g_lwActivated   = false;
uint32_t g_lastUplinkMs  = 0;

// ─── Config (persisted) ──────────────────────────────────────────────────────
struct DeviceConfig {
  char mode[8];           // "OTAA" / "ABP"
  char region[16];       // EU868, US915, ...
  uint32_t frequencyHz;   // informational default for region; MAC uses band plan
  uint8_t subBand;        // US915/AU915/CN470 — see RadioLib notes (often 2 for US915)
  bool adr;
  bool confirmed;
  uint8_t fPort;
  int8_t txPower;
  uint32_t serialBaud;  // applied after reboot
  // OTAA
  char joinEUI[32];       // hex string, 16 chars typical
  char devEUI[32];
  char appKey[64];        // 32 hex chars
  char nwkKey[64];        // LoRaWAN 1.1 / optional; if empty, appKey used for OTAA 1.0
  // ABP
  char devAddr[16];       // 8 hex chars
  char nwkSKey[64];
  char appSKey[64];
  // --- Class C and interval scheduler additions ---
  char lwClass[4];         // Always "C" (Class C only)
  uint8_t intervalSec;     // Health ping interval (seconds)
  uint8_t intervalMin;     // Health ping interval (minutes)
  uint8_t intervalHour;    // Health ping interval (hours)
  uint8_t intervalDay;     // Health ping interval (days)
  uint8_t intervalMonth;   // Health ping interval (months)
  char customPayload[128]; // Custom uplink payload (ASCII)
} cfg;

Preferences prefs;
WebServer server(80);



// Parsed binary keys (filled when joining)
// (already defined above)

// Remove LMIC global arrays

#define PREFS_NS "lwcfg"

// ─── Live log ring buffer ───────────────────────────────────────────────────
#define MAX_LOG 200
struct LogEntry {
  char t[20];
  char dir[10];
  char msg[240];
};
static LogEntry logs[MAX_LOG];
static int logHead = 0;
static int logCount = 0;

// Dedicated downlink buffer for Downlink Monitor
#define MAX_DOWNLINK 50
struct DownlinkEntry {
  char t[20];
  char hex[128];
  char ascii[65];
  int len;
  uint8_t fport;
};
static DownlinkEntry downlinks[MAX_DOWNLINK];
static int downlinkHead = 0;
static int downlinkCount = 0;

void addDownlink(const char* t, const char* hex, const char* ascii, int len, uint8_t fport) {
  DownlinkEntry& e = downlinks[downlinkHead];
  strncpy(e.t, t, sizeof(e.t) - 1);
  e.t[sizeof(e.t) - 1] = 0;
  strncpy(e.hex, hex, sizeof(e.hex) - 1);
  e.hex[sizeof(e.hex) - 1] = 0;
  strncpy(e.ascii, ascii, sizeof(e.ascii) - 1);
  e.ascii[sizeof(e.ascii) - 1] = 0;
  e.len = len;
  e.fport = fport;
  downlinkHead = (downlinkHead + 1) % MAX_DOWNLINK;
  if (downlinkCount < MAX_DOWNLINK) downlinkCount++;
}

void addLog(const char* dir, const String& m) {
  LogEntry& e = logs[logHead];
  unsigned long ms = millis();
  snprintf(e.t, sizeof(e.t), "%lu.%03lu", ms / 1000, ms % 1000);
  strncpy(e.dir, dir, sizeof(e.dir) - 1);
  e.dir[sizeof(e.dir) - 1] = 0;
  strncpy(e.msg, m.c_str(), sizeof(e.msg) - 1);
  e.msg[sizeof(e.msg) - 1] = 0;
  logHead = (logHead + 1) % MAX_LOG;
  if (logCount < MAX_LOG) logCount++;
  if (Serial) Serial.printf("[%s][%s] %s\n", e.t, e.dir, e.msg);
}

// ─── Helpers: hex ───────────────────────────────────────────────────────────
static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

bool parseHexString(const char* s, uint8_t* out, size_t nBytes) {
  if (!s) return false;
  size_t len = strlen(s);
  char tmp[128];
  size_t j = 0;
  for (size_t i = 0; i < len && j < sizeof(tmp) - 1; i++) {
    char c = s[i];
    if (c == ' ' || c == ':' || c == '-') continue;
    tmp[j++] = c;
  }
  tmp[j] = 0;
  if (strlen(tmp) != nBytes * 2) return false;
  for (size_t i = 0; i < nBytes; i++) {
    int hi = hexVal(tmp[i * 2]);
    int lo = hexVal(tmp[i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

bool parseHexU64MsbFirst(const char* s, uint64_t* out) {
  uint8_t b[8];
  if (!parseHexString(s, b, 8)) return false;
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v = (v << 8) | b[i];
  *out = v;
  return true;
}

bool parseHexU32(const char* s, uint32_t* out) {
  uint8_t b[4];
  if (!parseHexString(s, b, 4)) return false;
  *out = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
  return true;
}


// ─── Region → band pointer + default Hz ──────────────────────
uint32_t defaultHzForRegion(const char* reg) {
  if (!strcmp(reg, "EU868")) return 868100000UL;
  if (!strcmp(reg, "EU433")) return 433175000UL;
  if (!strcmp(reg, "US915")) return 902300000UL;
  if (!strcmp(reg, "AU915")) return 915200000UL;
  if (!strcmp(reg, "AS923")) return 923200000UL;
  if (!strcmp(reg, "AS923_2")) return 923200000UL;
  if (!strcmp(reg, "AS923_3")) return 923200000UL;
  if (!strcmp(reg, "AS923_4")) return 923200000UL;
  if (!strcmp(reg, "KR920"))  return 922100000UL;
  if (!strcmp(reg, "IN865"))  return 865062500UL;
  if (!strcmp(reg, "CN470"))  return 470300000UL;
  return 868100000UL;
}

const LoRaWANBand_t* bandPtr(const char* reg) {
  if (!strcmp(reg, "EU868")) return &EU868;
  if (!strcmp(reg, "EU433")) return &EU433;
  if (!strcmp(reg, "US915")) return &US915;
  if (!strcmp(reg, "AU915")) return &AU915;
  if (!strcmp(reg, "AS923")) return &AS923;
  if (!strcmp(reg, "AS923_2")) return &AS923_2;
  if (!strcmp(reg, "AS923_3")) return &AS923_3;
  if (!strcmp(reg, "AS923_4")) return &AS923_4;
  if (!strcmp(reg, "KR920"))  return &KR920;
  if (!strcmp(reg, "IN865"))  return &IN865;
  if (!strcmp(reg, "CN470"))  return &CN470;
  return &EU868;
}

String lwErrStr(int16_t e) {
  return String(e) + " (" + String((int)e) + ")";
}

// ─── NVS ────────────────────────────────────────────────────────────────────
void loadConfig() {
  prefs.begin(PREFS_NS, true);
  strlcpy(cfg.mode, prefs.getString("mode", "OTAA").c_str(), sizeof(cfg.mode));
  strlcpy(cfg.region, prefs.getString("region", "EU868").c_str(), sizeof(cfg.region));
  cfg.frequencyHz = prefs.getUInt("freqHz", defaultHzForRegion(cfg.region));
  cfg.subBand     = prefs.getUChar("subBand", 0);
  cfg.adr         = prefs.getBool("adr", true);
  cfg.confirmed   = prefs.getBool("confirmed", false);
  cfg.fPort       = prefs.getUChar("fPort", 1);
  cfg.txPower     = (int8_t)prefs.getChar("txPower", 14);
  cfg.serialBaud  = prefs.getUInt("baud", 115200);

  strlcpy(cfg.joinEUI, prefs.getString("joinEUI", "").c_str(), sizeof(cfg.joinEUI));
  strlcpy(cfg.devEUI, prefs.getString("devEUI", "").c_str(), sizeof(cfg.devEUI));
  strlcpy(cfg.appKey, prefs.getString("appKey", "").c_str(), sizeof(cfg.appKey));
  strlcpy(cfg.nwkKey, prefs.getString("nwkKey", "").c_str(), sizeof(cfg.nwkKey));
  strlcpy(cfg.devAddr, prefs.getString("devAddr", "").c_str(), sizeof(cfg.devAddr));
  strlcpy(cfg.nwkSKey, prefs.getString("nwkSKey", "").c_str(), sizeof(cfg.nwkSKey));
  strlcpy(cfg.appSKey, prefs.getString("appSKey", "").c_str(), sizeof(cfg.appSKey));
  // Force Class C
  strlcpy(cfg.lwClass, "C", sizeof(cfg.lwClass));
  cfg.intervalSec = prefs.getUChar("intervalSec", 0);
  cfg.intervalMin = prefs.getUChar("intervalMin", 0);
  cfg.intervalHour = prefs.getUChar("intervalHour", 0);
  cfg.intervalDay = prefs.getUChar("intervalDay", 0);
  cfg.intervalMonth = prefs.getUChar("intervalMonth", 0);
  strlcpy(cfg.customPayload, prefs.getString("customPayload", "").c_str(), sizeof(cfg.customPayload));
  prefs.end();
  strlcpy(cfg.lwClass, "C", sizeof(cfg.lwClass));  // HARD LOCK CLASS Cstrlcpy(cfg.lwClass, "C", sizeof(cfg.lwClass));  // HARD LOCK CLASS C
}

void saveConfig() {
  prefs.begin(PREFS_NS, false);
  prefs.putString("mode", cfg.mode);
  prefs.putString("region", cfg.region);
  prefs.putUInt("freqHz", cfg.frequencyHz);
  prefs.putUChar("subBand", cfg.subBand);
  prefs.putBool("adr", cfg.adr);
  prefs.putBool("confirmed", cfg.confirmed);
  prefs.putUChar("fPort", cfg.fPort);
  prefs.putChar("txPower", cfg.txPower);
  prefs.putUInt("baud", cfg.serialBaud);
  prefs.putString("joinEUI", cfg.joinEUI);
  prefs.putString("devEUI", cfg.devEUI);
  prefs.putString("appKey", cfg.appKey);
  prefs.putString("nwkKey", cfg.nwkKey);
  prefs.putString("devAddr", cfg.devAddr);
  prefs.putString("nwkSKey", cfg.nwkSKey);
  prefs.putString("appSKey", cfg.appSKey);
  prefs.putString("lwClass", "C"); // Always Class C
  prefs.putString("customPayload", cfg.customPayload);
  prefs.putUChar("intervalSec", cfg.intervalSec);
  prefs.putUChar("intervalMin", cfg.intervalMin);
  prefs.putUChar("intervalHour", cfg.intervalHour);
  prefs.putUChar("intervalDay", cfg.intervalDay);
  prefs.putUChar("intervalMonth", cfg.intervalMonth);
  prefs.end();
  addLog("INFO", "Configuration saved to NVS (Preferences).");
}


// No LMIC session buffer save/restore needed
void saveLwBuffers() {}
bool restoreLwBuffers() { return false; }
void clearLwBuffers() {}




// --- Prepare RadioLib credentials from config ---
bool prepareCredentials() {
  memset(binJoinEUI, 0, sizeof(binJoinEUI));
  memset(binDevEUI, 0, sizeof(binDevEUI));
  memset(binAppKey, 0, sizeof(binAppKey));
  memset(binNwkKey, 0, sizeof(binNwkKey));
  memset(binNwkSKey, 0, sizeof(binNwkSKey));
  memset(binAppSKey, 0, sizeof(binAppSKey));
  binDevAddr = 0;

  if (!strcasecmp(cfg.mode, "OTAA")) {
    uint64_t joinEUI64 = 0, devEUI64 = 0;
    if (!parseHexU64MsbFirst(cfg.joinEUI, &joinEUI64)) {
      addLog("ERROR", "OTAA: JoinEUI parse failed (expect 16 hex chars, MSB first).");
      return false;
    }
    if (!parseHexU64MsbFirst(cfg.devEUI, &devEUI64)) {
      addLog("ERROR", "OTAA: DevEUI parse failed (expect 16 hex chars, MSB first).");
      return false;
    }
    if (!parseHexString(cfg.appKey, binAppKey, 16)) {
      addLog("ERROR", "OTAA: AppKey parse failed (expect 32 hex chars).");
      return false;
    }
    if (strlen(cfg.nwkKey) >= 32) {
      if (!parseHexString(cfg.nwkKey, binNwkKey, 16)) {
        addLog("ERROR", "OTAA: NwkKey parse failed.");
        return false;
      }
    } else {
      memcpy(binNwkKey, binAppKey, 16); // LoRaWAN 1.0.x TTN: AppKey == NwkKey
    }
    (void)joinEUI64;
    (void)devEUI64;
    return true;
  }

  if (!strcasecmp(cfg.mode, "ABP")) {
    if (!parseHexU32(cfg.devAddr, &binDevAddr)) {
      addLog("ERROR", "ABP: DevAddr parse failed (8 hex chars).");
      return false;
    }
    if (!parseHexString(cfg.nwkSKey, binNwkSKey, 16)) {
      addLog("ERROR", "ABP: NwkSKey parse failed.");
      return false;
    }
    if (!parseHexString(cfg.appSKey, binAppSKey, 16)) {
      addLog("ERROR", "ABP: AppSKey parse failed.");
      return false;
    }
    return true;
  }

  addLog("ERROR", "Unknown mode (use OTAA or ABP).");
  return false;
}

// --- RadioLib radio and node setup/teardown ---
void destroyLoRaWAN() {
  if (lorawan) {
    lorawan->clearSession();
    delete lorawan;
    lorawan = nullptr;
  }
  g_lwActivated = false;
}

bool createLoRaWAN() {
  destroyLoRaWAN();
  lwBand = bandPtr(cfg.region);
  lwSubBand = cfg.subBand;
  lorawan = new LoRaWANNode(&radio, lwBand, lwSubBand);
  return lorawan != nullptr;
}

int initRadioPhy() {
  addLog("INIT", "PHY: SPI.begin(SCK,MISO,MOSI,CS) on internal SX1262 bus");
  SPI.begin(LORA_SX126X_SCK, LORA_SX126X_MISO, LORA_SX126X_MOSI, LORA_SX126X_CS);

  float mhz = cfg.frequencyHz / 1e6f;
  addLog("INIT", String("PHY: radio.begin() LoRa ") + mhz + " MHz, 125k, SF7, CR4/5, TCXO 1.8V (RAK3112)");
  int16_t st = radio.begin(
    mhz,
    125.0f,
    7,
    5,
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    cfg.txPower,
    8,
    1.8f,
    false
  );
  if (st != RADIOLIB_ERR_NONE) {
    addLog("ERROR", String("radio.begin failed: ") + lwErrStr(st));
    g_radioPhyReady = false;
    return st;
  }

  g_radioPhyReady = true;
  addLog("INIT", "PHY OK — equivalent to: lora_rak3112_init + Radio.Init + Sleep/SetChannel/SetTx/Rx/Rx()");
  addLog("INIT", "MAC layer will override datarate/channels per band plan during LoRaWAN join/uplink.");
  return RADIOLIB_ERR_NONE;
}



bool doJoin() {
  addLog("INFO", "══ Join / activate started ══");
  if (!prepareCredentials()) return false;

  destroyLoRaWAN();

  if (initRadioPhy() != RADIOLIB_ERR_NONE) return false;

  if (!createLoRaWAN()) {
    addLog("ERROR", "LoRaWANNode allocation failed.");
    return false;
  }

  int16_t st;

  if (!strcasecmp(cfg.mode, "OTAA")) {
    uint64_t joinEUI = 0, devEUI = 0;
    parseHexU64MsbFirst(cfg.joinEUI, &joinEUI);
    parseHexU64MsbFirst(cfg.devEUI, &devEUI);

    st = lorawan->beginOTAA(joinEUI, devEUI, binNwkKey, binAppKey);
    if (st != RADIOLIB_ERR_NONE) {
      addLog("ERROR", String("beginOTAA failed: ") + lwErrStr(st));
      return false;
    }
    addLog("INFO", "OTAA: beginOTAA OK (keys loaded into MAC).");

    if (restoreLwBuffers()) {
      addLog("INFO", "Attempting activateOTAA() with restored NVS session...");
    } else {
      addLog("INFO", "No valid NVS session — full OTAA join will run.");
    }

    st = lorawan->activateOTAA();
    if (st != RADIOLIB_LORAWAN_NEW_SESSION && st != RADIOLIB_LORAWAN_SESSION_RESTORED) {
      addLog("ERROR", String("activateOTAA failed: ") + lwErrStr(st));
      addLog("ERROR", "Check TTN keys, region, sub-band, gateway coverage, and 'Resets join nonces' if testing.");
      return false;
    }
    addLog("UP", st == RADIOLIB_LORAWAN_NEW_SESSION
                ? "OTAA NEW SESSION — Join-Accept received (device ↔ gateway ↔ TTN join path OK)."
                : "OTAA SESSION RESTORED from NVS (no new Join-Accept this boot).");
  } else {
    // ABP LoRaWAN 1.0: pass NULL, NULL for 1.1 keys; NwkSKey in nwkSEncKey slot
    st = lorawan->beginABP(binDevAddr, nullptr, nullptr, binNwkSKey, binAppSKey);
    if (st != RADIOLIB_ERR_NONE) {
      addLog("ERROR", String("beginABP failed: ") + lwErrStr(st));
      return false;
    }
    st = lorawan->activateABP();
    if (st != RADIOLIB_ERR_NONE) {
      addLog("ERROR", String("activateABP failed: ") + lwErrStr(st));
      return false;
    }
    addLog("INFO", "ABP session activated (no Join-Accept; DevAddr + session keys programmed).");
  }


  lorawan->setADR(cfg.adr);
  lorawan->setTxPower(cfg.txPower);
  // ===== FORCE CLASS C (HARD LOCK) =====
  int16_t classResult = lorawan->setClass(RADIOLIB_LORAWAN_CLASS_C);
  addLog("INFO", "CLASS C FORCED (hard lock active)");
  addLog("DEBUG", String("setClass(Class C) result: ") + classResult);
  addLog("DEBUG", String("Region: ") + cfg.region + ", subBand: " + cfg.subBand);

  saveLwBuffers();
  g_lwActivated = lorawan->isActivated();
  g_lastUplinkMs = millis();
  addLog("INFO", String("LoRaWAN activated. ADR=") + (cfg.adr ? "on" : "off") +
                 " confirmedUL=" + (cfg.confirmed ? "on" : "off") +
                 " FPort=" + cfg.fPort + " txPow=" + cfg.txPower + " dBm");
  return true;
}

void doLeave() {
  destroyLoRaWAN();
  clearLwBuffers();
  g_radioPhyReady = false;
  addLog("INFO", "Left LoRaWAN — session cleared. Re-Join required.");
}

// ─── Uplink interval ──────────────────────────────────────────────────────────
uint32_t getUplinkIntervalMs() {
  uint32_t ms = 0;
  ms += (uint32_t)cfg.intervalSec * 1000UL;
  ms += (uint32_t)cfg.intervalMin * 60UL * 1000UL;
  ms += (uint32_t)cfg.intervalHour * 60UL * 60UL * 1000UL;
  ms += (uint32_t)cfg.intervalDay * 24UL * 60UL * 60UL * 1000UL;
  ms += (uint32_t)cfg.intervalMonth * 30UL * 24UL * 60UL * 60UL * 1000UL;
  if (ms == 0) ms = 60000UL;
  return ms;
}

// ─── HTTP: embedded UI ───────────────────────────────────────────────────────

static const char PAGE[] = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>RAK3112 LoRaWAN</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;}
body{font-family:system-ui,-apple-system,sans-serif;background:#f5f7fa;color:#1a1d23;font-size:14px;}

/* ── Login ── */
#loginPage{min-height:100vh;display:flex;align-items:center;justify-content:center;}
.login-card{background:#fff;border:1px solid #e0e4ec;border-radius:14px;padding:40px 36px;width:340px;box-shadow:0 2px 16px rgba(0,0,0,.07);}
.login-card h2{font-size:20px;font-weight:600;color:#1a1d23;margin-bottom:6px;}
.login-card p{font-size:13px;color:#6b7280;margin-bottom:24px;}
.login-card input{width:100%;padding:10px 12px;border:1px solid #d1d5db;border-radius:8px;font-size:14px;color:#1a1d23;background:#fff;outline:none;transition:border .15s;}
.login-card input:focus{border-color:#4f6ef7;}
.login-card .err{color:#dc2626;font-size:12px;margin-top:6px;min-height:16px;}
.login-card button{width:100%;margin-top:18px;padding:11px;background:#4f6ef7;color:#fff;border:none;border-radius:8px;font-size:14px;font-weight:600;cursor:pointer;transition:background .15s,transform .1s;}
.login-card button:hover{background:#3d5ce0;}

/* ── Layout ── */
header{background:#fff;border-bottom:1px solid #e0e4ec;padding:0 24px;height:56px;display:flex;align-items:center;justify-content:space-between;}
header h1{font-size:15px;font-weight:600;color:#1a1d23;}
.status-pill{display:inline-flex;align-items:center;gap:6px;padding:4px 12px;border-radius:20px;font-size:12px;font-weight:500;}
.pill-ok{background:#dcfce7;color:#166534;}
.pill-err{background:#fee2e2;color:#991b1b;}
.pill-idle{background:#f1f5f9;color:#475569;}
.dot{width:7px;height:7px;border-radius:50%;background:currentColor;opacity:.75;}

.page{max-width:1400px;margin:0 auto;padding:20px 20px;}
.grid{display:grid;grid-template-columns:2fr 1fr;gap:18px;}
@media(max-width:1000px){.grid{grid-template-columns:1fr;}}

/* ── Cards ── */
.card{background:#fff;border:1px solid #e0e4ec;border-radius:12px;padding:20px;}
.card h2{font-size:13px;font-weight:600;color:#6b7280;text-transform:uppercase;letter-spacing:.06em;margin-bottom:16px;}

/* ── Form ── */
.formgrid{display:grid;grid-template-columns:1fr 1fr;gap:12px 18px;}
.full{grid-column:span 2;}
label{display:block;font-size:12px;font-weight:500;color:#374151;margin-bottom:4px;}
input[type=text],input[type=number],input[type=password],select{width:100%;padding:9px 11px;border:1px solid #d1d5db;border-radius:7px;font-size:13px;color:#1a1d23;background:#fff;outline:none;transition:border .15s;}
input:focus,select:focus{border-color:#4f6ef7;box-shadow:0 0 0 3px rgba(79,110,247,.1);}
.check-row{display:flex;align-items:center;gap:8px;padding:9px 0;}
.check-row input[type=checkbox]{width:15px;height:15px;accent-color:#4f6ef7;cursor:pointer;}
.check-row span{font-size:13px;color:#374151;}

.interval-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:10px;margin-top:4px;}
.interval-grid .lbl{font-size:11px;color:#6b7280;margin-bottom:3px;font-weight:500;}

/* ── Buttons ── */
.btn-row{display:flex;gap:10px;flex-wrap:wrap;margin-top:18px;}
.btn{padding:9px 20px;border-radius:7px;border:none;cursor:pointer;font-size:13px;font-weight:600;transition:opacity .15s,transform .1s;}
.btn:active{transform:scale(.97);}
.btn-save{background:#4f6ef7;color:#fff;}
.btn-save:hover{background:#3d5ce0;}
.btn-join{background:#059669;color:#fff;}
.btn-join:hover{background:#047857;}
.btn-leave{background:#f1f5f9;color:#475569;border:1px solid #e0e4ec;}
.btn-leave:hover{background:#e2e8f0;}

/* ── Log ── */
#log{font-family:'SF Mono','Fira Code',ui-monospace,monospace;font-size:11.5px;background:#f8fafc;border:1px solid #e0e4ec;border-radius:8px;padding:12px;height:320px;overflow-y:auto;white-space:pre-wrap;line-height:1.5;color:#1e293b;}
.log-up{color:#0369a1;}
.log-down{color:#7c3aed;}
.log-err{color:#dc2626;}
.log-info{color:#374151;}
.log-init{color:#059669;}

/* ── Downlink monitor ── */
#downlinkBox{background:#f8fafc;border:1px solid #e0e4ec;border-radius:8px;padding:12px;min-height:90px;font-family:'SF Mono','Fira Code',ui-monospace,monospace;font-size:11.5px;line-height:1.6;color:#1e293b;overflow-y:auto;}
.dl-row{padding:5px 0;border-bottom:1px solid #f1f5f9;}
.dl-row:last-child{border-bottom:none;}
.dl-time{color:#9ca3af;font-size:11px;}
.dl-hex{color:#4f6ef7;font-weight:600;}
.dl-ascii{color:#374151;}
.dl-len{color:#9ca3af;font-size:11px;}
.dl-port{background:#ede9fe;color:#5b21b6;border-radius:4px;padding:1px 6px;font-size:11px;}
.no-dl{color:#9ca3af;font-size:12px;}

/* ── Section dividers ── */
.section-title{font-size:11px;font-weight:600;color:#6b7280;text-transform:uppercase;letter-spacing:.08em;margin:18px 0 10px;padding-bottom:6px;border-bottom:1px solid #f1f5f9;}
</style>
</head><body>

<!-- ═══ LOGIN ═══ -->
<div id="loginPage">
  <div class="login-card">
    <h2>RAK3112 Portal</h2>
    <p>Enter your portal password to continue.</p>
    <input id="pass" type="password" placeholder="Password" onkeydown="if(event.key==='Enter')doLogin()">
    <div class="err" id="err"></div>
    <button onclick="doLogin()">Sign in</button>
  </div>
</div>

<!-- ═══ MAIN ═══ -->
<div id="mainPage" style="display:none">

<header>
  <h1>RAK3112 &mdash; LoRaWAN Config Portal</h1>
  <span class="status-pill pill-idle" id="statusPill"><span class="dot"></span><span id="statusTxt">Idle</span></span>
</header>

<div class="page">
<div class="grid">

  <!-- Left: config -->
  <div class="card">
    <h2>Device Configuration</h2>
    <div class="formgrid">
      <div>
        <label>Activation mode</label>
        <select id="mode">
          <option value="OTAA">OTAA</option>
          <option value="ABP">ABP</option>
        </select>
      </div>
      <!-- Class C only: no class selection -->
      <div style="display:none">
        <label>LoRaWAN class</label>
        <select id="lwClass" disabled><option value="C" selected>Class C</option></select>
      </div>

      <!-- OTAA fields -->
      <div id="f_joinEUI"><label>JoinEUI (16 hex)</label><input id="joinEUI" type="text" placeholder="0000000000000000"></div>
      <div id="f_devEUI"><label>DevEUI (16 hex)</label><input id="devEUI" type="text" placeholder="0000000000000000"></div>
      <div id="f_appKey"><label>AppKey (32 hex)</label><input id="appKey" type="text" placeholder="00000000000000000000000000000000"></div>
      <div id="f_nwkKey"><label>NwkKey (32 hex, optional)</label><input id="nwkKey" type="text" placeholder="Leave blank for LoRaWAN 1.0.x"></div>

      <!-- ABP fields -->
      <div id="f_devAddr" style="display:none"><label>DevAddr (8 hex)</label><input id="devAddr" type="text" placeholder="00000000"></div>
      <div id="f_nwkSKey" style="display:none"><label>NwkSKey (32 hex)</label><input id="nwkSKey" type="text"></div>
      <div id="f_appSKey" style="display:none"><label>AppSKey (32 hex)</label><input id="appSKey" type="text"></div>

      <!-- Radio -->
      <div>
        <label>Region</label>
        <select id="region">
          <option>EU868</option><option>EU433</option><option>US915</option><option>AU915</option>
          <option>AS923</option><option>AS923_2</option><option>AS923_3</option><option>AS923_4</option>
          <option>KR920</option><option>IN865</option><option>CN470</option>
        </select>
      </div>
      <div>
        <label>Frequency (Hz)</label>
        <input id="frequency" type="number">
      </div>
      <div>
        <label>Sub-band (0 = auto)</label>
        <input id="subBand" type="number" value="0">
      </div>
      <div>
        <label>Serial baud</label>
        <select id="baud">
          <option>115200</option><option>921600</option><option>460800</option>
          <option>230400</option><option>57600</option><option>38400</option>
        </select>
      </div>

      <div>
        <label>FPort</label>
        <input id="fPort" type="number" value="1">
      </div>
      <div>
        <label>TX Power (dBm)</label>
        <input id="txPower" type="number" value="14">
      </div>

      <div class="check-row"><input type="checkbox" id="adr"><span>Adaptive Data Rate (ADR)</span></div>
      <div class="check-row"><input type="checkbox" id="confirmed"><span>Confirmed uplinks</span></div>

      <div class="full">
        <label>Custom payload (ASCII)</label>
        <input id="customPayload" type="text" placeholder="hi and im Dhaanes">
      </div>

      <div class="full">
        <div class="section-title">Health ping interval</div>
        <div class="interval-grid">
          <div><div class="lbl">Seconds</div><input id="intervalSec" type="number" min="0" max="60" value="0"></div>
          <div><div class="lbl">Minutes</div><input id="intervalMin" type="number" min="0" max="60" value="0"></div>
          <div><div class="lbl">Hours</div><input id="intervalHour" type="number" min="0" max="24" value="0"></div>
          <div><div class="lbl">Days</div><input id="intervalDay" type="number" min="0" max="30" value="0"></div>
          <div><div class="lbl">Months</div><input id="intervalMonth" type="number" min="0" max="12" value="0"></div>
        </div>
      </div>

      <div class="full btn-row">
        <button class="btn btn-save" onclick="save()">Save config</button>
        <button class="btn btn-join" onclick="join()">Join / Activate</button>
        <button class="btn btn-leave" onclick="leave()">Leave</button>
      </div>
    </div>
  </div>

  <!-- Right: log + downlink -->
  <div style="display:flex;flex-direction:column;gap:18px;">

    <div class="card">
      <h2>Live log</h2>
      <div id="log">Waiting for events...</div>
    </div>

    <div class="card">
      <h2>Downlink monitor</h2>
      <div id="downlinkBox"><span class="no-dl">No downlink received yet.</span></div>
    </div>

  </div>

</div>
</div>
</div>

<script>
/* ── Login ── */
function doLogin(){
  const PASS="2240624";
  if(document.getElementById("pass").value===PASS){
    document.getElementById("loginPage").style.display="none";
    document.getElementById("mainPage").style.display="block";
    loadCfg();
  }else{
    document.getElementById("err").textContent="Incorrect password.";
  }
}

/* ── Mode toggle ── */
function toggle(){
  const isOTAA=document.getElementById("mode").value==="OTAA";
  ["f_joinEUI","f_devEUI","f_appKey","f_nwkKey"].forEach(id=>{
    document.getElementById(id).style.display=isOTAA?"block":"none";
  });
  ["f_devAddr","f_nwkSKey","f_appSKey"].forEach(id=>{
    document.getElementById(id).style.display=isOTAA?"none":"block";
  });
}
document.getElementById("mode").addEventListener("change",toggle);

/* ── Load config ── */
async function loadCfg(){
  try{
    const r=await fetch('/api/config');const j=await r.json();
    document.getElementById("mode").value=j.mode||"OTAA";
    document.getElementById("joinEUI").value=j.joinEUI||"";
    document.getElementById("devEUI").value=j.devEUI||"";
    document.getElementById("appKey").value=j.appKey||"";
    document.getElementById("nwkKey").value=j.nwkKey||"";
    document.getElementById("devAddr").value=j.devAddr||"";
    document.getElementById("nwkSKey").value=j.nwkSKey||"";
    document.getElementById("appSKey").value=j.appSKey||"";
    document.getElementById("region").value=j.region||"EU868";
    document.getElementById("frequency").value=j.frequencyHz||868100000;
    document.getElementById("subBand").value=j.subBand||0;
    document.getElementById("baud").value=String(j.serialBaud||115200);
    document.getElementById("adr").checked=!!j.adr;
    document.getElementById("confirmed").checked=!!j.confirmed;
    document.getElementById("fPort").value=j.fPort||1;
    document.getElementById("txPower").value=j.txPower||14;
    document.getElementById("lwClass").value=j.lwClass||"C";
    document.getElementById("customPayload").value=j.customPayload||"";
    document.getElementById("intervalSec").value=j.intervalSec||0;
    document.getElementById("intervalMin").value=j.intervalMin||0;
    document.getElementById("intervalHour").value=j.intervalHour||0;
    document.getElementById("intervalDay").value=j.intervalDay||0;
    document.getElementById("intervalMonth").value=j.intervalMonth||0;
    toggle();
  }catch(e){console.error(e);}
}

/* ── Save ── */
async function save(){
  const body=JSON.stringify({
    mode:document.getElementById("mode").value,
    joinEUI:document.getElementById("joinEUI").value.trim(),
    devEUI:document.getElementById("devEUI").value.trim(),
    appKey:document.getElementById("appKey").value.trim(),
    nwkKey:document.getElementById("nwkKey").value.trim(),
    devAddr:document.getElementById("devAddr").value.trim(),
    nwkSKey:document.getElementById("nwkSKey").value.trim(),
    appSKey:document.getElementById("appSKey").value.trim(),
    region:document.getElementById("region").value,
    frequencyHz:parseInt(document.getElementById("frequency").value)||868100000,
    subBand:parseInt(document.getElementById("subBand").value)||0,
    serialBaud:parseInt(document.getElementById("baud").value)||115200,
    adr:document.getElementById("adr").checked,
    confirmed:document.getElementById("confirmed").checked,
    fPort:parseInt(document.getElementById("fPort").value)||1,
    txPower:parseInt(document.getElementById("txPower").value)||14,
    lwClass:document.getElementById("lwClass").value,
    customPayload:document.getElementById("customPayload").value,
    intervalSec:parseInt(document.getElementById("intervalSec").value)||0,
    intervalMin:parseInt(document.getElementById("intervalMin").value)||0,
    intervalHour:parseInt(document.getElementById("intervalHour").value)||0,
    intervalDay:parseInt(document.getElementById("intervalDay").value)||0,
    intervalMonth:parseInt(document.getElementById("intervalMonth").value)||0
  });
  await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body});
}

async function join(){
  const r=await fetch('/api/join',{method:'POST'});
  const j=await r.json();
  setStatus(j.ok,'join');
}
async function leave(){
  await fetch('/api/leave',{method:'POST'});
  setStatus(false,'idle');
}

/* ── Status pill ── */
function setStatus(ok,hint){
  const pill=document.getElementById("statusPill");
  const txt=document.getElementById("statusTxt");
  pill.className="status-pill "+(ok?"pill-ok":(hint==='idle'?"pill-idle":"pill-err"));
  txt.textContent=ok?"Joined":(hint==='idle'?"Idle":"Failed");
}

/* ── Log colouring ── */
function colourLine(line){
  const d=line.match(/\[([A-Z]+)\]/);
  if(!d)return line;
  const map={UP:'log-up',DOWN:'log-down',ERROR:'log-err',INIT:'log-init'};
  const cls=map[d[1]]||'log-info';
  const span=document.createElement('span');
  span.className=cls;
  span.textContent=line+'\n';
  return span;
}

/* ── Poll logs ── */
async function pollLogs(){
  try{
    const r=await fetch('/api/logs');const j=await r.json();
    const box=document.getElementById("log");
    box.innerHTML="";
    for(const e of j.logs){
      const line=`[${e.t}][${e.d}] ${e.m}`;
      box.appendChild(typeof colourLine==="function"?colourLine(line):document.createTextNode(line+'\n'));
    }
    box.scrollTop=box.scrollHeight;
    // update pill if state known
    const pill=document.getElementById("statusPill");
    if(j.joined){
      pill.className="status-pill pill-ok";
      document.getElementById("statusTxt").textContent="Joined";
    } else if(j.phy){
      pill.className="status-pill pill-idle";
      document.getElementById("statusTxt").textContent="PHY ready";
    }
  }catch(e){}
}

/* ── Poll downlink ── */
async function pollDownlink(){
  try{
    const r=await fetch('/api/downlink');const j=await r.json();
    const box=document.getElementById("downlinkBox");
    if(!j.downlink||j.downlink.length===0){
      box.innerHTML='<span class="no-dl">No downlink received yet.</span>';
      return;
    }
    box.innerHTML="";
    for(const e of [...j.downlink].reverse()){
      if(!e.hex&&e.len===0)continue;
      const row=document.createElement("div");
      row.className="dl-row";
      let ascii="";
      for(let i=0;i<(e.hex||"").length;i+=2){
        const c=parseInt(e.hex.substr(i,2),16);
        ascii+=(c>=32&&c<=126)?String.fromCharCode(c):'.';
      }
      row.innerHTML=
        `<span class="dl-time">${e.t}</span> `+
        `<span class="dl-port">port ${e.fport||'?'}</span> `+
        `<span class="dl-hex">${e.hex||'(empty)'}</span> `+
        (ascii?`<span class="dl-ascii"> '${ascii}'</span> `:'')+
        `<span class="dl-len">${e.len} B</span>`;
      box.appendChild(row);
    }
  }catch(e){}
}

setInterval(pollLogs,1000);
setInterval(pollDownlink,1000);
</script>
</body></html>
)HTML";

void handleRoot() {
  server.send(200, "text/html", PAGE);
}

void handleGetConfig() {
  DynamicJsonDocument doc(2048);
  doc["mode"] = cfg.mode;
  doc["region"] = cfg.region;
  doc["frequencyHz"] = cfg.frequencyHz;
  doc["subBand"] = cfg.subBand;
  doc["adr"] = cfg.adr;
  doc["confirmed"] = cfg.confirmed;
  doc["fPort"] = cfg.fPort;
  doc["txPower"] = cfg.txPower;
  doc["serialBaud"] = cfg.serialBaud;
  doc["joinEUI"] = cfg.joinEUI;
  doc["devEUI"] = cfg.devEUI;
  doc["appKey"] = cfg.appKey;
  doc["nwkKey"] = cfg.nwkKey;
  doc["devAddr"] = cfg.devAddr;
  doc["nwkSKey"] = cfg.nwkSKey;
  doc["appSKey"] = cfg.appSKey;
  doc["lwClass"] = cfg.lwClass;
  doc["customPayload"] = cfg.customPayload;
  doc["intervalSec"] = cfg.intervalSec;
  doc["intervalMin"] = cfg.intervalMin;
  doc["intervalHour"] = cfg.intervalHour;
  doc["intervalDay"] = cfg.intervalDay;
  doc["intervalMonth"] = cfg.intervalMonth;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handlePostConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }
  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }
  strlcpy(cfg.mode, doc["mode"] | "OTAA", sizeof(cfg.mode));
  strlcpy(cfg.region, doc["region"] | "EU868", sizeof(cfg.region));
  cfg.frequencyHz = doc["frequencyHz"] | defaultHzForRegion(cfg.region);
  cfg.subBand = doc["subBand"] | 0;
  cfg.adr = doc["adr"] | true;
  cfg.confirmed = doc["confirmed"] | false;
  cfg.fPort = doc["fPort"] | 1;
  cfg.txPower = (int8_t)(doc["txPower"] | 14);
  cfg.serialBaud = doc["serialBaud"] | 115200UL;
  strlcpy(cfg.joinEUI, doc["joinEUI"] | "", sizeof(cfg.joinEUI));
  strlcpy(cfg.devEUI, doc["devEUI"] | "", sizeof(cfg.devEUI));
  strlcpy(cfg.appKey, doc["appKey"] | "", sizeof(cfg.appKey));
  strlcpy(cfg.nwkKey, doc["nwkKey"] | "", sizeof(cfg.nwkKey));
  strlcpy(cfg.devAddr, doc["devAddr"] | "", sizeof(cfg.devAddr));
  strlcpy(cfg.nwkSKey, doc["nwkSKey"] | "", sizeof(cfg.nwkSKey));
  strlcpy(cfg.appSKey, doc["appSKey"] | "", sizeof(cfg.appSKey));
  strlcpy(cfg.lwClass, "C", sizeof(cfg.lwClass));  // UI ignored
  strlcpy(cfg.customPayload, doc["customPayload"] | "", sizeof(cfg.customPayload));
  cfg.intervalSec = doc["intervalSec"] | 0;
  cfg.intervalMin = doc["intervalMin"] | 0;
  cfg.intervalHour = doc["intervalHour"] | 0;
  cfg.intervalDay = doc["intervalDay"] | 0;
  cfg.intervalMonth = doc["intervalMonth"] | 0;
  saveConfig();
  addLog("INFO", String("Saved: region=") + cfg.region + " Hz=" + cfg.frequencyHz +
                 " subBand=" + cfg.subBand + " mode=" + cfg.mode);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleGetLogs() {
  const int kMaxLines = 120;
  DynamicJsonDocument doc(24576);
  JsonArray arr = doc.createNestedArray("logs");
  int n = logCount < kMaxLines ? logCount : kMaxLines;
  int start = logCount >= MAX_LOG ? (logHead - n + MAX_LOG) % MAX_LOG : 0;
  for (int i = 0; i < n; i++) {
    int idx = (start + i) % MAX_LOG;
    JsonObject o = arr.createNestedObject();
    o["t"] = logs[idx].t;
    o["d"] = logs[idx].dir;
    String m = logs[idx].msg;
    m.replace("\\", "\\\\");
    m.replace("\"", "'");
    o["m"] = m;
  }
  doc["joined"] = g_lwActivated;
  doc["phy"] = g_radioPhyReady;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleGetDownlink() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("downlink");
  int n = downlinkCount < MAX_DOWNLINK ? downlinkCount : MAX_DOWNLINK;
  int start = downlinkCount >= MAX_DOWNLINK ? (downlinkHead - n + MAX_DOWNLINK) % MAX_DOWNLINK : 0;
  for (int i = 0; i < n; i++) {
    int idx = (start + i) % MAX_DOWNLINK;
    JsonObject o = arr.createNestedObject();
    o["t"] = downlinks[idx].t;
    o["hex"] = downlinks[idx].hex;
    o["ascii"] = downlinks[idx].ascii;
    o["len"] = downlinks[idx].len;
    o["fport"] = downlinks[idx].fport;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleJoinReq() {
  bool ok = doJoin();
  server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

void handleLeaveReq() {
  doLeave();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ─── setup / loop ────────────────────────────────────────────────────────────
void setup() {
  loadConfig();
  Serial.begin(cfg.serialBaud);
  delay(300);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);

  addLog("INIT", "RAK3112 LoRaWAN portal boot");
  addLog("INIT", String("Serial baud=") + cfg.serialBaud);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_SN);
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    addLog("INIT", String("AP ") + AP_SSID + " → http://192.168.4.1");
  } else {
    addLog("ERROR", "AP start failed");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/logs", HTTP_GET, handleGetLogs);
  server.on("/api/downlink", HTTP_GET, handleGetDownlink);
  server.on("/api/join", HTTP_POST, handleJoinReq);
  server.on("/api/leave", HTTP_POST, handleLeaveReq);
  server.begin();

  addLog("INFO", "Load TTN keys, Save, then Join.");
}


// ─── Periodic uplink + FIXED downlink reception ───────────────────────────────
void doPeriodicUplink() {
  if (!lorawan || !lorawan->isActivated()) return;
  lorawan->setClass(RADIOLIB_LORAWAN_CLASS_C);
  uint32_t interval = getUplinkIntervalMs();
  if (millis() - g_lastUplinkMs < interval) return;
  g_lastUplinkMs = millis();

  const char* payload = cfg.customPayload[0] ? cfg.customPayload : "hi and im Dhaanes";
  size_t payloadLen = strlen(payload);

  // ── DOWNLINK FIX: allocate buffer, track received length separately ──
  uint8_t down[256];
  size_t downLen = 0;  // FIX: must start at 0, not sizeof(down)
                       // RadioLib writes the actual received byte count here

  addLog("UP", String("Uplink → TTN FPort=") + cfg.fPort +
               " \"" + payload + "\" (" + payloadLen + " B)");

  // For Class C, use sendReceive to ensure RX windows and immediate downlink check
  int16_t st = lorawan->sendReceive(
    (const uint8_t*)payload,
    payloadLen,
    cfg.fPort,
    down,
    &downLen,
    cfg.confirmed
  );

  if (st == RADIOLIB_ERR_NONE) {
    // RADIOLIB_ERR_NONE (0) = uplink sent, RX windows opened, no app payload received
    // For Class C, this means no downlink was queued at TTN at this moment
    addLog("INFO", "Uplink OK — RX windows opened, no downlink payload queued at TTN.");

  } else if (st > 0) {
    // Positive return = downlink received (value = fPort of downlink, or MAC-only)
    if (downLen > 0) {
      String hx;
      String asc;
      for (size_t i = 0; i < downLen; i++) {
        char b[4];
        snprintf(b, sizeof(b), "%02X", down[i]);
        hx += b;
        char ac = (down[i] >= 32 && down[i] <= 126) ? (char)down[i] : '.';
        asc += ac;
      }
      unsigned long ms = millis();
      char t[20];
      snprintf(t, sizeof(t), "%lu.%03lu", ms / 1000, ms % 1000);

      uint8_t rxFPort = (uint8_t)st;  // RadioLib returns fPort as the positive value
      addDownlink(t, hx.c_str(), asc.c_str(), (int)downLen, rxFPort);

      addLog("DOWN", String("Downlink from TTN: ") + downLen +
                     " B port=" + rxFPort + " hex=" + hx + " ascii='" + asc + "'");
      digitalWrite(PIN_LED2, HIGH);
      delay(80);
      digitalWrite(PIN_LED2, LOW);
    } else {
      // Positive st but downLen==0 → MAC command only (no app payload)
      addLog("DOWN", String("Downlink window: MAC command only (no app payload), st=") + st);
    }

  } else {
    // Negative = error
    addLog("ERROR", String("sendReceive failed: ") + lwErrStr(st));
  }

  saveLwBuffers();
}



// ─── Class C always-on downlink reception ─────────────────────────────
// This function checks for and processes Class C downlinks at any time.
// It should be called as often as possible in the main loop.
void checkClassCDownlink() {
  if (!lorawan || !lorawan->isActivated()) return;
  uint8_t downlinkPayload[255];
  size_t downlinkLen = 0;
  LoRaWANEvent_t downlinkEvent;
  int16_t state = lorawan->getDownlinkClassC(downlinkPayload, &downlinkLen, &downlinkEvent);
  if (state > 0 && downlinkLen > 0) {
    String hx, asc;
    for (size_t i = 0; i < downlinkLen; i++) {
      char b[4];
      snprintf(b, sizeof(b), "%02X", downlinkPayload[i]);
      hx += b;
      char ac = (downlinkPayload[i] >= 32 && downlinkPayload[i] <= 126) ? (char)downlinkPayload[i] : '.';
      asc += ac;
    }
    unsigned long ms = millis();
    char t[20];
    snprintf(t, sizeof(t), "%lu.%03lu", ms / 1000, ms % 1000);
    uint8_t rxFPort = downlinkEvent.fPort;
    addDownlink(t, hx.c_str(), asc.c_str(), (int)downlinkLen, rxFPort);
    addLog("DOWN", String("Class C downlink: ") + downlinkLen +
                   " B port=" + rxFPort + " hex=" + hx + " ascii='" + asc + "'");
    digitalWrite(PIN_LED2, HIGH);
    delay(80);
    digitalWrite(PIN_LED2, LOW);
  }
}

void loop() {
  server.handleClient();
  doPeriodicUplink();
  checkClassCDownlink();
}


