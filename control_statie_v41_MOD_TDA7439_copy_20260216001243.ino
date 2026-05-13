#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <time.h>
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <radio.h>
#include <SI4703.h>
#include <SI47xx.h>
#include <RDSParser.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <RTClib.h>
#include <TDA7439.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <IRremote.h> // Add this at the top if not already present
#include <Adafruit_MCP23X17.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#endif

// Debug logging can make WebUI feel sluggish (Serial can block).
// Set to 1 only while diagnosing.
#ifndef DEBUG_SERIAL_LOGS
#define DEBUG_SERIAL_LOGS 0
#endif

// PC bridge polling can be frequent; keep its Serial logs off by default.
// Turn on only while debugging PC bridge networking.
#ifndef DEBUG_PC_BRIDGE_SERIAL
#define DEBUG_PC_BRIDGE_SERIAL 0
#endif

// 1 = PC /state polling in FreeRTOS task (recommended; avoids long HTTP in loop contending with WebServer WiFi).
// 0 = poll from main loop via tickSourceTransitionsAndBridgeIo() (debug only; can add multi-second /api/v2 lag).
#ifndef ENABLE_PC_BRIDGE_POLLING
#define ENABLE_PC_BRIDGE_POLLING 0
#endif

// USB Serial: la runtime, printf-urile pot bloca loop-ul dacă bufferul e plin și încetinesc Web UI-ul.
// Păstrăm loguri utile la boot: setup(), connectToWiFi(), initRpiSerial, scanI2CBus, test TDA7439, OTA callbacks.
// Web UI: POST-uri scurte; starea se reîncarcă cu refreshState (un singur GET în zbor, coalesce) + kickSync după mutații
// (EQ/volum/frecvență se aplică în loop, nu în handler HTTP — altfel UI „nu se actualizează” fără refresh).

#define TDA7439_ADDRESS 0x44

// Web UI server/task enable.
#define ENABLE_WEBUI 1

// PC-only mode: disable all Raspberry/Moode UART bridge work.
// Useful when diagnosing sluggish WebUI / foobar bridge issues.
#ifndef PC_ONLY_FOOBAR
#define PC_ONLY_FOOBAR 0
#endif

// Forward declarations
class FMRadioController;
void showTime();
void drawsetScreen();
void showTime();  // redraw time/date on top
void checkTouch();
void testTDA7439Comms();
bool reinitSI4703();
bool resetSI4703Hardware();
void i2cBusRecover();
bool syncRTCWithNTP();
bool syncRTCWithNTPRetries();
void maintainWiFiConnection();
void setTDA7439Balance(int8_t bal);
void initRpiSerial();
void pollRpiSerial();
bool sendRpiCommand(const char *cmd);
void processRpiLine(const char *line);
static void setRpiUartEnabled(bool enabled);
void updateRpiTitleTicker(bool forceRedraw);
void updateRpiTransportPlayIcon(bool forceRedraw);
void updateRpiPlayIndicator(bool forceRedraw);
bool sendPcCommand(const char *cmd);
void pollPcBridge();
void processPcStatLine(const char *line);
void updatePcNowPlayingUi(bool forceRedraw);
void setPcAnalogUi(bool enabled, bool persistPrefs = true);
void handlePc();
extern bool standbyState;
extern bool rpiBridgeArmed;

// Standby control pins
#define POWER_INDICATOR_PIN 33  // OUTPUT: HIGH la boot — nu se mai comută la standby (fără relay pe GPIO33)
#define STANDBY_CTRL_PIN 32     // GPIO button to toggle standby
#define RADIO_LED_PIN 17        // OUTPUT: HIGH = FM radio active, LOW = radio off
#define STANDBY_DEBOUNCE_MS 50  // Debounce interval (ms)
#define RPI_SERIAL_RX_PIN 25    // ESP32 RX <- Raspberry TX
#define RPI_SERIAL_TX_PIN 27    // ESP32 TX -> Raspberry RX
#define RPI_SERIAL_BAUD 115200
// Touch compensation for RPI control row (positive=to right, negative=to left)
#define RPI_TOUCH_X_OFFSET -72

// Windows / foobar2000 bridge (LAN).
// Expected to be a small HTTP server running on the PC.
static const char *PC_BRIDGE_HOST = "192.168.7.77"; // set to your PC IP
static const uint16_t PC_BRIDGE_PORT = 8765;
// Too small timeouts can cause intermittent -11 (connect fail) during brief WiFi/PC stalls.
static const uint16_t PC_BRIDGE_TIMEOUT_MS = 800;

// PC audio routing:
// - PCA: PC audio via XMOS/USB path (current setup: keep TDA input same as Raspberry)
// - PCD: PC audio "analog" path (legacy PC input on TDA7439)
//
// If you wire a hardware mux/relay, set PC_AUDIO_SEL_PIN to the GPIO that selects XMOS vs analog.
// - PCA (XMOS) => LOW, PCD (analog) => HIGH (invert with PC_AUDIO_SEL_ACTIVE_HIGH if needed)
static const int PC_AUDIO_SEL_PIN = -1;  // set to a valid GPIO when hardware is wired
static const bool PC_AUDIO_SEL_ACTIVE_HIGH = true;

// Optional I2S selector output (for external DAC boards like Audiophonics ES9038):
// - PCD (source id=5) is the "PC digital" mode (foobar UI) but still uses the RPi transport path in the UI.
// - Hardware often needs a GPIO to switch I2S input between Amanero (PC) and Raspberry (RPi).
//
// Configure this pin to drive your I2S mux/relay:
// - Set to 13 or 14 (or any free GPIO) to enable; set to -1 to disable.
// - Polarity is board-dependent. Configure which level selects Amanero:
//    - true  => Amanero selected with HIGH, Raspberry with LOW
//    - false => Amanero selected with LOW,  Raspberry with HIGH (common)
static const int PC_DIGITAL_STATUS_PIN = 14;
static const bool PC_DIGITAL_STATUS_AMANERO_LEVEL_HIGH = true;

// Persisted preferences keys (stored in NVS via FMRadioController::saveSettingsNow)
static bool pcAnalogUi = false;      // true => show big "PC ANALOG" overlay (hides metadata)

static void fmtShorten(const char *s, size_t maxLen, char *out, size_t outCap) {
  if (!out || outCap == 0) return;
  if (!s) s = "";
  size_t len = strlen(s);
  if (len < maxLen) maxLen = len;
  if (maxLen >= outCap) maxLen = outCap - 1;
  if (len <= maxLen) {
    memcpy(out, s, maxLen);
    out[maxLen] = '\0';
    return;
  }
  if (maxLen <= 3) {
    memcpy(out, s, maxLen);
    out[maxLen] = '\0';
    return;
  }
  size_t keep = maxLen - 3;
  memcpy(out, s, keep);
  memcpy(out + keep, "...", 4); // includes NUL
}

static bool streqLower(const char *a, const char *b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if (ca != cb) return false;
  }
  return *a == *b;
}

static bool rpiPlayingFromState(const char *state) {
  return streqLower(state, "play") || streqLower(state, "playing");
}

static void appendJsonEscaped(String &json, const char *raw) {
  if (!raw) raw = "";
  json += '"';
  for (const char *p = raw; *p; p++) {
    char c = *p;
    if (c == '\\' || c == '"') {
      json += '\\';
      json += c;
    } else {
      json += c;
    }
  }
  json += '"';
}

static void printJsonEscaped(Print &out, const char *raw) {
  if (!raw) raw = "";
  out.print('"');
  for (const char *p = raw; *p; p++) {
    char c = *p;
    if (c == '\\' || c == '"') {
      out.print('\\');
      out.print(c);
    } else if (c == '\n') {
      out.print("\\n");
    } else if (c == '\r') {
      out.print("\\r");
    } else if (c == '\t') {
      out.print("\\t");
    } else {
      out.print(c);
    }
  }
  out.print('"');
}

static void sendJsonEscapedContent(WebServer &srv, const char *raw) {
  if (!raw) raw = "";
  // Batch into RAM buffer — per-character sendContent() was hundreds of TCP segments per /status.
  uint8_t buf[192];
  size_t pos = 0;
  auto flush = [&]() {
    if (pos) {
      srv.sendContent((const char *)buf, pos);
      pos = 0;
    }
  };
  auto room = [&](size_t need) {
    if (pos + need > sizeof(buf)) flush();
  };

  room(1);
  buf[pos++] = (uint8_t)'"';
  for (const char *p = raw; *p; p++) {
    unsigned char c = (unsigned char)*p;
    if (c == '\\' || c == '"') {
      room(2);
      buf[pos++] = (uint8_t)'\\';
      buf[pos++] = c;
    } else if (c == '\n') {
      room(2);
      buf[pos++] = (uint8_t)'\\';
      buf[pos++] = (uint8_t)'n';
    } else if (c == '\r') {
      room(2);
      buf[pos++] = (uint8_t)'\\';
      buf[pos++] = (uint8_t)'r';
    } else if (c == '\t') {
      room(2);
      buf[pos++] = (uint8_t)'\\';
      buf[pos++] = (uint8_t)'t';
    } else {
      room(1);
      buf[pos++] = c;
    }
  }
  room(1);
  buf[pos++] = (uint8_t)'"';
  flush();
}

// Buffer repeated sendContent() calls to avoid many tiny TCP segments (big impact on WebUI "Content Download" time).
// Designed for chunked responses (CONTENT_LENGTH_UNKNOWN) where we already use sendContent() streaming.
class WebChunkedWriter {
public:
  explicit WebChunkedWriter(WebServer &srv) : srv_(srv) {}

  inline void write(const char *s) {
    if (!s) return;
    while (*s) writeChar(*s++);
  }

  inline void writeChar(char c) {
    if (pos_ >= sizeof(buf_)) flush();
    buf_[pos_++] = (uint8_t)c;
  }

  inline void writeInt(long v) {
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%ld", v);
    write(tmp);
  }

  inline void writeFloat2(double v) {
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.2f", v);
    write(tmp);
  }

  inline void writeJsonEscaped(const char *raw) {
    if (!raw) raw = "";
    writeChar('"');
    for (const char *p = raw; *p; p++) {
      unsigned char c = (unsigned char)*p;
      if (c == '\\' || c == '"') {
        writeChar('\\');
        writeChar((char)c);
      } else if (c == '\n') {
        writeChar('\\');
        writeChar('n');
      } else if (c == '\r') {
        writeChar('\\');
        writeChar('r');
      } else if (c == '\t') {
        writeChar('\\');
        writeChar('t');
      } else {
        writeChar((char)c);
      }
    }
    writeChar('"');
  }

  inline void flush() {
    if (!pos_) return;
    srv_.sendContent((const char *)buf_, pos_);
    pos_ = 0;
  }

private:
  WebServer &srv_;
  uint8_t buf_[1536];
  size_t pos_ = 0;
};

static void sendContentInt(WebServer &srv, long v) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%ld", v);
  srv.sendContent(buf);
}

static void sendContentFloat2(WebServer &srv, double v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", v);
  srv.sendContent(buf);
}

// Defined later (after globals like fmRadio / rpi buffers exist).
static void sendStatusJson(WebServer &srv);
static void sendStateV2Json(WebServer &srv);

// ESP32 lwIP: one client + keep-alive often leaves sockets wedged; close after each response for reliability.
static void webHeadersNoCacheClose(WebServer &srv) {
  srv.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  srv.sendHeader("Pragma", "no-cache");
  srv.sendHeader("Connection", "close");
}

static void webBeginJsonStream(WebServer &srv) {
  webHeadersNoCacheClose(srv);
  srv.setContentLength(CONTENT_LENGTH_UNKNOWN);
  srv.send(200, "application/json", "");
}

static void webSendProgmemChunked(WebServer &srv, const char *contentType, const char *progmem) {
  if (!progmem) {
    webSendText(srv, 500, "missing content");
    return;
  }
  webHeadersNoCacheClose(srv);
  srv.sendHeader("Expires", "0");
  srv.setContentLength(CONTENT_LENGTH_UNKNOWN);
  srv.send(200, contentType ? contentType : "text/plain", "");

  const size_t total = strlen_P(progmem);
  const size_t CHUNK = 1024;
  for (size_t off = 0; off < total; off += CHUNK) {
    size_t n = total - off;
    if (n > CHUNK) n = CHUNK;
    srv.sendContent_P(progmem + off, n);
    yield();
  }
  webEndStream(srv);
}

// When using CONTENT_LENGTH_UNKNOWN (chunked), we must terminate the response body.
static inline void webEndStream(WebServer &srv) {
  srv.sendContent("");
}

// Tiny JSON for /rpi and /pc — full sendStatusJson() here was heavy and stacked with Moode STAT traffic.
static void webSendJsonOk(WebServer &srv) {
  webBeginJsonStream(srv);
  srv.sendContent("{\"ok\":true}");
  webEndStream(srv);
}

static inline void webSendStateV2(WebServer &srv) {
  webBeginJsonStream(srv);
  sendStateV2Json(srv);
  webEndStream(srv);
}

static inline void webSendText(WebServer &srv, int code, const char *msg) {
  webHeadersNoCacheClose(srv);
  srv.send(code, "text/plain", msg ? msg : "");
}

static inline void webSendBadRequest(WebServer &srv, const char *msg) {
  webSendText(srv, 400, msg);
}

static inline void webSendConflict(WebServer &srv, const char *msg) {
  webSendText(srv, 409, msg);
}

static volatile bool webStatusBusy = false;
static volatile bool webSystemInfoBusy = false;
static volatile bool webPresetsBusy = false;

static bool webRejectIfStandby(WebServer &srv) {
  if (!standbyState) return false;
  webSendText(srv, 403, "Device is in standby");
  return true;
}

#if defined(ARDUINO_ARCH_ESP32)
static bool popPendingCommand(volatile bool &pendingFlag, char *sharedBuf, size_t sharedCap, char *outBuf, size_t outCap, portMUX_TYPE *mux) {
  bool have = false;
  portENTER_CRITICAL(mux);
  if (pendingFlag) {
    size_t n = (sharedCap < outCap) ? sharedCap : outCap;
    memcpy(outBuf, (const void *)sharedBuf, n);
    outBuf[outCap - 1] = '\0';
    pendingFlag = false;
    have = true;
  }
  portEXIT_CRITICAL(mux);
  return have;
}

static void setPendingCommand(volatile bool &pendingFlag, char *sharedBuf, size_t sharedCap, const char *cmd, portMUX_TYPE *mux) {
  portENTER_CRITICAL(mux);
  strncpy(sharedBuf, cmd ? cmd : "", sharedCap - 1);
  sharedBuf[sharedCap - 1] = '\0';
  pendingFlag = true;
  portEXIT_CRITICAL(mux);
}
#else
static bool popPendingCommand(volatile bool &pendingFlag, char *sharedBuf, size_t sharedCap, char *outBuf, size_t outCap) {
  if (!pendingFlag) return false;
  size_t n = (sharedCap < outCap) ? sharedCap : outCap;
  memcpy(outBuf, (const void *)sharedBuf, n);
  outBuf[outCap - 1] = '\0';
  pendingFlag = false;
  return true;
}

static void setPendingCommand(volatile bool &pendingFlag, char *sharedBuf, size_t sharedCap, const char *cmd) {
  strncpy(sharedBuf, cmd ? cmd : "", sharedCap - 1);
  sharedBuf[sharedCap - 1] = '\0';
  pendingFlag = true;
}
#endif

bool inStandby = false;  // track our current mode

#if defined(ARDUINO_ARCH_ESP32)
// Forward declaration for global web server instance
extern WebServer server;
// Globals defined later; used by background tasks.
extern bool standbyState;
extern volatile int lastPcHttpCode;
extern volatile bool pcBridgeOk;
// PC bridge diagnostics (defined later; updated by pcBridgeTask and reported via /api/v2/state)
extern volatile uint32_t pcOkCount;
extern volatile uint32_t pcFailCount;
extern volatile uint32_t pcConnectFailCount;
extern volatile uint32_t pcReadFailCount;
extern volatile uint32_t pcLastOkMs;
extern volatile uint32_t pcLastFailMs;
extern volatile int pcLastFailCode;
extern volatile uint32_t pcLastDtMs;

// Web task diagnostics (helps debug "WebUI sluggish" without Serial spam)
static volatile uint32_t webTaskLastUs = 0;
static volatile uint32_t webTaskMaxUs = 0;
static volatile uint32_t webTaskLoops = 0;
static volatile uint32_t webTaskLastMs = 0;

static TaskHandle_t webServerTaskHandle = nullptr;
static void webServerTask(void *param) {
  for (;;) {
    uint32_t t0 = micros();
    ArduinoOTA.handle();
    server.handleClient();
    uint32_t dt = micros() - t0;
    webTaskLastUs = dt;
    if (dt > webTaskMaxUs) webTaskMaxUs = dt;
    webTaskLoops++;
    webTaskLastMs = (uint32_t)millis();
    vTaskDelay(1);
  }
}

// PC bridge polling: keep blocking HTTP out of loop().
static TaskHandle_t pcBridgeTaskHandle = nullptr;
static portMUX_TYPE pcStatMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool pcStatPending = false;
static char pcStatPendingLine[220];

// Snapshot of UI state for background tasks (avoid referencing `fmRadio` here; it is defined later).
static volatile int pcUiSource = 0;
static volatile bool pcUiUtilMode = false;

static void pcSetPendingStatLine(const char *line) {
  if (!line) return;
  portENTER_CRITICAL(&pcStatMux);
  strncpy(pcStatPendingLine, line, sizeof(pcStatPendingLine) - 1);
  pcStatPendingLine[sizeof(pcStatPendingLine) - 1] = '\0';
  pcStatPending = true;
  portEXIT_CRITICAL(&pcStatMux);
}

static bool pcPopPendingStatLine(char *out, size_t outCap) {
  if (!out || outCap == 0) return false;
  bool have = false;
  portENTER_CRITICAL(&pcStatMux);
  if (pcStatPending) {
    strncpy(out, pcStatPendingLine, outCap - 1);
    out[outCap - 1] = '\0';
    pcStatPending = false;
    have = true;
  }
  portEXIT_CRITICAL(&pcStatMux);
  return have;
}

static int pcFetchStateLine(char *outLine, size_t outCap, uint32_t timeoutMs) {
  if (!outLine || outCap == 0) return -1;
  outLine[0] = '\0';
  if (WiFi.status() != WL_CONNECTED) return -1;
  if (!PC_BRIDGE_HOST || !PC_BRIDGE_HOST[0]) return -1;

  WiFiClient client;
  client.setTimeout((timeoutMs + 50) / 1000.0f);

  const uint32_t t0 = millis();
  // Prefer numeric IP connect (avoids any hostname parsing/DNS edge cases on ESP32).
  // Also retry connect until timeout to reduce "flapping" during short WiFi/PC stalls.
  bool connected = false;
  IPAddress ip;
  const bool isIp = ip.fromString(PC_BRIDGE_HOST);
  while ((millis() - t0) <= timeoutMs) {
    if (isIp) connected = client.connect(ip, PC_BRIDGE_PORT);
    else connected = client.connect(PC_BRIDGE_HOST, PC_BRIDGE_PORT);
    if (connected) break;
    delay(10);
  }
  if (!connected) {
    return -11; // connect failure
  }

  client.print("GET /state HTTP/1.1\r\nHost: ");
  client.print(PC_BRIDGE_HOST);
  client.print("\r\nConnection: close\r\n\r\n");

  // Read status line (best-effort) + headers until blank line.
  int code = -1;
  String status = client.readStringUntil('\n');
  status.trim();
  if (status.startsWith("HTTP/")) {
    int sp = status.indexOf(' ');
    if (sp > 0 && sp + 1 < status.length()) {
      code = status.substring(sp + 1).toInt();
    }
  }
  // Drain headers.
  while (client.connected()) {
    if ((millis() - t0) > timeoutMs) break;
    String h = client.readStringUntil('\n');
    if (h.length() <= 1) break; // "\r" or empty
  }

  // Body is a single STAT|... line.
  String body = client.readStringUntil('\n');
  body.trim();
  if (body.length() == 0) return (code > 0) ? code : -12;
  strncpy(outLine, body.c_str(), outCap - 1);
  outLine[outCap - 1] = '\0';
  return (code > 0) ? code : 200;
}

static void pcBridgeTask(void *param) {
  (void)param;
  uint8_t failStreak = 0;
  uint32_t lastOkMs = 0;
  for (;;) {
    if (!ENABLE_PC_BRIDGE_POLLING) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    // Only poll when PCA/PCD is active and not in standby.
    // NOTE: `utilMode` is a touchscreen-only mode; it shouldn't disable foobar polling,
    // otherwise WebUI can get stuck showing Offline with transport disabled.
    const int src = pcUiSource;
    const bool active = (!standbyState) && (src == 4 || src == 5);
    if (!active || WiFi.status() != WL_CONNECTED) {
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }

    const uint32_t now = millis();
    const uint32_t timeoutMs = PC_BRIDGE_TIMEOUT_MS; // keep consistent with existing config
    char line[220];
    uint32_t t0 = millis();
    int code = pcFetchStateLine(line, sizeof(line), timeoutMs);
    uint32_t dt = millis() - t0;

    lastPcHttpCode = code;
    pcLastDtMs = (uint32_t)dt;
    // HTTP OK means the bridge is reachable; body may be empty on a slow read.
    if (code >= 200 && code < 300) {
      pcBridgeOk = true;
      failStreak = 0;
      lastOkMs = (uint32_t)millis();
      pcLastOkMs = lastOkMs;
      pcOkCount++;
      if (line[0]) pcSetPendingStatLine(line);
      // Reduce socket churn (each poll opens a new TCP connection).
      // 650ms was still aggressive on ESP32 + WebUI traffic; use ~1.5s with small jitter.
      vTaskDelay((1500 + (esp_random() % 150)) / portTICK_PERIOD_MS);
    } else {
      // Avoid flapping the UI Offline on a single transient connect/read failure.
      // Keep last known ONLINE for a short grace window after the last success.
      const uint32_t nowMs = (uint32_t)millis();
      const bool inGrace = (lastOkMs != 0) && ((uint32_t)(nowMs - lastOkMs) < 2500U);
      if (!inGrace) pcBridgeOk = false;
      if (failStreak < 20) failStreak++;
      pcFailCount++;
      pcLastFailMs = nowMs;
      pcLastFailCode = code;
      if (code == -11) pcConnectFailCount++;
      else pcReadFailCount++;
      // Recover quickly when the PC bridge comes back:
      // - first few failures: retry fast (250ms)
      // - after that: ramp to a modest max (5s), not 60s
      uint32_t backoff = 250UL;
      if (failStreak > 6) {
        backoff = 250UL * (uint32_t)(failStreak - 5); // 250,500,750,...
        if (backoff > 5000UL) backoff = 5000UL;
      }
      // Also add a tiny jitter while failing so retries don't phase-lock with WebUI refresh.
      vTaskDelay((backoff + (esp_random() % 80)) / portTICK_PERIOD_MS);
    }
  }
}
#endif

//------------------------------------
// Pin Definitions & Constants
//------------------------------------
#define TFT_CS 15
#define TFT_RST -1
#define TFT_DC 26
#define RESET_PIN 4

#define TOUCH_CS_PIN 16
#define TOUCH_IRQ 255
#define TOUCH_MIN_X 300
#define TOUCH_MAX_X 3800
#define TOUCH_MIN_Y 300
#define TOUCH_MAX_Y 3700
// Touch panel orientation.
// Coordinate system expected by hitboxes: (0,0) = top-left.
// User measurement: bottom-right reports (0,0) and top-left reports (320,240),
// so we must mirror BOTH axes to get UI coordinates.
#define TOUCH_MIRROR_X 1
#define TOUCH_MIRROR_Y 1

// MCP23017: INT A = GPIO36, INT B = GPIO39; SDA/SCL shared with I2C bus
#define MCP_INT_A_PIN 36
#define MCP_INT_B_PIN 39
#define MCP23017_ADDR 0x27

// MCP23017 pin numbers (Adafruit: 0-7 = GPA, 8-15 = GPB); all inputs, pull-up
// Port A: PA4=A encoder, PA5=B encoder, PA7-PA0 = MEM1..MEM6 (statii radio memorate)
#define MCP_PA4_ENC_A    4
#define MCP_PA5_ENC_B    5
#define MCP_PA7_MEM1     7
#define MCP_PA6_MEM2     6
#define MCP_PA0_MEM3     0
#define MCP_PA1_MEM4     1
#define MCP_PA2_MEM5     2
#define MCP_PA3_MEM6     3
// Port B: PB0=RPI, PB1=PC, PB2=encoder switch, PB4=MUTE, PB5=MODE, PB6=TUN, PB7=BT (surse)
#define MCP_PB0_RPI      8
#define MCP_PB1_PC       9
#define MCP_PB2_ENC_SW  10
#define MCP_PB4_MUTE    12
#define MCP_PB5_MODE    13
#define MCP_PB6_TUN     14
#define MCP_PB7_BT      15

#define RADIO_SCLPIN 22
#define RADIO_SDAPIN 21
#define RADIO_BAND RADIO_BAND_FM
#define MIN_FREQ 8750
#define MAX_FREQ 10800
#define DEFAULT_FREQ 10060
#define MIN_VOLUME 0
#define MAX_VOLUME 48
#define DEFAULT_VOLUME 5
#define DEFAULT_SURSA 1

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Transport row (RPI/PC): draw rects MUST match touch — tx = mappedX + RPI_TOUCH_X_OFFSET.
#define TRANSPORT_ROW_Y     149
#define TRANSPORT_ROW_H     45
#define TRANSPORT_BTN_W     70
#define TRANSPORT_BTN_GAP   2
#define TRANSPORT_BTN0_X    0
#define TRANSPORT_BTN1_X    (TRANSPORT_BTN0_X + TRANSPORT_BTN_W + TRANSPORT_BTN_GAP)
#define TRANSPORT_BTN2_X    (TRANSPORT_BTN1_X + TRANSPORT_BTN_W + TRANSPORT_BTN_GAP)
/* PCA/PCD: aceeași linie ca transportul, aceeași lățime ca Prev/Play/Next, lipit de marginea
   dreaptă a display-ului (orizontal în dreptul butonului TUN din rândul de sus). */
#define TRANSPORT_TX_MAX0   71
#define TRANSPORT_TX_MAX1   143
#define TRANSPORT_TX_MAX2   214

#define CUSTOM_GREY 0x7BEF
#define CUSTOM_LIGHTGREY 0xC618
#define CUSTOM_DARKGREEN 0x03E0
#define CUSTOM_LIGHTBLUE 0x7DDF
#define CUSTOM_CYAN 0x07FF

#define DEBOUNCE_DELAY 50  // Debounce delay in milliseconds
constexpr unsigned long AUTO_MODE_SWITCH_DELAY = 10000;

// Seek buttons (triangles) are drawn in drawSeekButton() around y=160..190 on the right side.
// Use bounding boxes that cover the triangles.
#define SEEK_UP_X_MIN   270
#define SEEK_UP_X_MAX   (SCREEN_WIDTH - 1)
#define SEEK_UP_Y_MIN   155
#define SEEK_UP_Y_MAX   195
#define SEEK_DN_X_MIN   205
#define SEEK_DN_X_MAX   265
#define SEEK_DN_Y_MIN   155
#define SEEK_DN_Y_MAX   195

#define SETUP_BUTTON_X_MIN 260
#define SETUP_BUTTON_Y_MIN 200
#define SETUP_BUTTON_WIDTH 60
#define SETUP_BUTTON_HEIGHT 40
// UTIL EQ screen: must match FMRadioController::drawsetScreen() plus/minus rects.
#define UTIL_EQ_PLUS_Y_MIN   88
#define UTIL_EQ_PLUS_Y_MAX   127
#define UTIL_EQ_MINUS_Y_MIN  156
#define UTIL_EQ_MINUS_Y_MAX  195

#define SI4703_ADDR 0x10  // Default I2C address for SI4703

//------------------------------------
// Global Objects & Structures
//------------------------------------
RTC_DS3231 rtc;
Preferences preferences;
WebServer server(80);  // Web server runs on port 80

// for our "software RTC" in Standby
static DateTime standbyStartTime;
static uint32_t standbyStartMs;

// near the top, with your other globals:
bool standbyState = false;            // false = active, true = standby
static bool lastStandbyState = HIGH;  // for buttonPressed()
static unsigned long lastStandbyPressTime = 0;

TDA7439 tda7439;

// Global variable to prevent sensors from overwriting web-set frequency
unsigned long lastWebFreqChange = 0;

// Track last activity in util mode
unsigned long lastUtilActivity = 0;

// Add a global flag to indicate if SI4703 is powered
bool si4703Powered = true;

// Encoder state for volume mode (global, not inside any class)
int32_t encoderLastEncVal = 0;

// MCP23017: expander object and encoder count (PA4/PA5 = quadrature)
Adafruit_MCP23X17 mcp;
bool mcpOk = false;
uint8_t mcpI2cAddr = 0;  // 0 = not found, else 0x20..0x27
int32_t mcpEncoderCount = 0;
// Cache pentru /systemInfo ca handler-ul sa nu faca I2C (evita blocarea web UI)
static bool cachedTdaOk = false, cachedRtcOk = false, cachedMcpOk = false, cachedSi47Ok = false;
static unsigned long lastSystemInfoCacheMs = 0;
static uint8_t mcpLastEncA = 1, mcpLastEncB = 1;  // pull-up = released
static unsigned long lastMcpPresetPress[6] = { 0 };
static int lastMcpPresetIndex = -1;
static bool mcpPresetWasActive = false;

// Add at the top with other globals:
bool tda7439Active = true;

// Add IR volume repeat state variables near the top with other globals:
static unsigned long irVolumePressTime = 0;
static bool irVolumeHeld = false;
static bool irVolumeRepeatActive = false;
static uint8_t lastIrVolumeCode = 0xFF; // 0x05 for up, 0x00 for down
static unsigned long lastIrVolumeRepeat = 0;

// Add at the top with other globals:
const uint32_t IR_REPEAT_CODE = 0xFFFFFFFF;
static unsigned long lastIrRepeatReceived = 0;

// Post-boot delayed audio settings re-apply to ensure gain latches
static bool postBootAudioApplyPending = false;
// Defer heavy I2C/UI work from HTTP handlers to the main loop
volatile bool volumeApplyPending = false;
volatile int pendingVolume = 0;
volatile bool eqApplyPending = false;
volatile bool freqApplyPending = false;
volatile float pendingFreq = 0.0f;
volatile int8_t pendingSeekDir = 0; // -1 = down, 1 = up, 0 = none
volatile bool uiRefreshPending = false;
volatile bool powerOnPending = false;
volatile bool powerOffPending = false;
// Cached values computed in main loop to avoid I2C from HTTP task
volatile int lastFmRssi = -1;            // SI4703 RSSI (0..127), -1 if N/A
static unsigned long lastFmRssiPoll = 0;  // millis for throttling RSSI sampling
static unsigned long postBootAudioApplyAt = 0;
static unsigned long lastRpiPollMs = 0;
static unsigned long lastRpiRxMs = 0;
static bool rpiConnected = false;
static char rpiRxLine[300];
static uint16_t rpiRxLen = 0;
static char rpiStateBuf[16] = "unknown";
static char rpiTitleBuf[96] = "-";
static char rpiArtistBuf[96] = "-";
static char rpiFileBuf[96] = "-";
static unsigned long lastRpiCmdTxMs = 0;
static char lastRpiCmdTxBuf[16] = "";
static unsigned long rpiForceGetAtMs = 0;
static unsigned long lastRpiBlinkMs = 0;
static bool rpiBlinkOn = false;

// PC (foobar) metadata via windows_foobar_bridge.py (/state -> STAT|... line)
static char pcStateBuf[16] = "unknown";
static char pcTitleBuf[96] = "-";
static char pcArtistBuf[96] = "-";
static char pcExtraBuf[96] = "-";
volatile bool pcBridgeOk = false;
static unsigned long lastPcPollMs = 0;
static unsigned long pcForcePollAtMs1 = 0;
static unsigned long pcForcePollAtMs2 = 0;
static unsigned long lastPcRxMs = 0;
volatile int lastPcHttpCode = 0;
// PC bridge diagnostics (helps root-cause intermittent -11/-12 without Serial spam)
volatile uint32_t pcOkCount = 0;
volatile uint32_t pcFailCount = 0;
volatile uint32_t pcConnectFailCount = 0; // -11
volatile uint32_t pcReadFailCount = 0;    // -12 or other non-2xx without body
volatile uint32_t pcLastOkMs = 0;
volatile uint32_t pcLastFailMs = 0;
volatile int pcLastFailCode = 0;
volatile uint32_t pcLastDtMs = 0;

// WiFi.RSSI() can block the web server task; refresh from main loop (~1 Hz).
static volatile int webStatusWifiRssi = -127;
static unsigned long webStatusWifiRssiAtMs = 0;

// Web UI -> PC bridge transport: run HTTPClient in loop(), never in the web server task
// (blocking WiFi client freezes /, /status, and causes partial page loads).
static const size_t WEB_CMD_BUF_CAP = 16;
volatile bool pcWebCmdPending = false;
static char pcWebCmdBuf[WEB_CMD_BUF_CAP];
#if defined(ARDUINO_ARCH_ESP32)
static portMUX_TYPE pcWebCmdMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// Web UI -> UART: never call Serial2 from the HTTP task (TX can block; RPi off => Web UI freeze).
volatile bool rpiWebCmdPending = false;
static char rpiWebCmdBuf[WEB_CMD_BUF_CAP];
#if defined(ARDUINO_ARCH_ESP32)
static portMUX_TYPE rpiWebCmdMux = portMUX_INITIALIZER_UNLOCKED;
#endif
// After failed UART TX, wait before retry (avoid tight spin on full TX buffer).
static unsigned long rpiWebCmdNextTryMs = 0;

// PCD mode: uses source 3 (RPi input) but shows/controls foobar via PC bridge.
static bool pcdMode = false;

// #region agent log: periodic UI freeze profiler (ESP32 Serial)
static uint32_t profDrawUsMax = 0;
static uint32_t profMainContentUsMax = 0;
static uint32_t profLoopUsMax = 0;
static uint32_t profPollUsMax = 0;
static uint32_t profFmUpdateUsMax = 0;
static uint32_t profTouchUsMax = 0;
static uint32_t profShowTimeUsMax = 0;
static uint32_t profStatMiniUsMax = 0;
static uint32_t profStatUsMax = 0;
static uint32_t profTickerUsMax = 0;
static uint16_t profTickerDraws = 0;
static uint32_t profHeapMin = 0xFFFFFFFFu;
static uint32_t profHeapNow = 0;
static uint32_t profPrintUsMax = 0;
static uint32_t profApplyTdaUsMax = 0;
static uint16_t profProfSkipped = 0;
static uint16_t profProfSent = 0;
static uint16_t profSysInfoRuns = 0;
static uint16_t profNvsWrites = 0;
static uint32_t profStatCount = 0;
static unsigned long profWindowStartMs = 0;
// #endregion

// Short-deferred TDA apply to avoid blocking HTTP handlers
static bool audioApplyPending = false;
static unsigned long audioApplyAt = 0;

// --- Audio debug snapshot (visible in /api/v2/state) ---
static volatile unsigned long lastTdaWriteMs = 0;
static volatile int lastTdaInput = -1;
static volatile int lastTdaVolume = -1;
static char lastTdaWhy[16] = "boot";

// Boot-time TDA volume reporter: prints every 10s for 5 minutes after boot.
#ifndef BOOT_TDA_VOLUME_LOG
#define BOOT_TDA_VOLUME_LOG 0
#endif
static bool bootTdaLogActive = false;
static unsigned long bootTdaLogUntilMs = 0;
static unsigned long bootTdaLogNextMs = 0;

static inline void log_tda_event(const char *why, int input, int vol) {
#if !DEBUG_SERIAL_LOGS
  (void)why; (void)input; (void)vol;
  return;
#else
  // Throttle to avoid making performance worse.
  static unsigned long lastLogMs = 0;
  unsigned long now = millis();
  if (now - lastLogMs < 750UL) return;
  lastLogMs = now;

  Serial.print("[AUDIO] t=");
  Serial.print(now);
  Serial.print(" why=");
  Serial.print(why ? why : "?");
  if (input >= 0) { Serial.print(" in="); Serial.print(input); }
  if (vol >= 0) { Serial.print(" vol="); Serial.print(vol); }
  Serial.println();
#endif
}

static inline void noteTdaWrite(const char *why, int input, int vol) {
  lastTdaWriteMs = millis();
  if (input >= 0) lastTdaInput = input;
  if (vol >= 0) lastTdaVolume = vol;
  if (why && why[0]) {
    strncpy(lastTdaWhy, why, sizeof(lastTdaWhy) - 1);
    lastTdaWhy[sizeof(lastTdaWhy) - 1] = '\0';
  }
  log_tda_event(why, input, vol);
}

static inline void tickBootTdaVolumeLog(int uiVol, bool muted) {
#if !BOOT_TDA_VOLUME_LOG
  (void)uiVol;
  (void)muted;
  return;
#else
  if (!bootTdaLogActive) return;
  const unsigned long now = millis();
  if ((long)(now - bootTdaLogUntilMs) >= 0) {
    bootTdaLogActive = false;
    return;
  }
  if ((long)(now - bootTdaLogNextMs) < 0) return;
  bootTdaLogNextMs = now + 10000UL;
  if (Serial.availableForWrite() < 64) return;
  Serial.print("[BOOT][TDA] t=");
  Serial.print(now);
  Serial.print(" uiVol=");
  Serial.print(uiVol);
  Serial.print(" muted=");
  Serial.print(muted ? 1 : 0);
  Serial.print(" lastTdaVol=");
  Serial.print(lastTdaVolume);
  Serial.print(" lastWriteMs=");
  Serial.print(lastTdaWriteMs);
  Serial.print(" why=");
  Serial.println(lastTdaWhy);
#endif
}

bool buttonPressed(uint8_t pin, bool &lastState, unsigned long &lastPressTime) {
  bool currentState = digitalRead(pin);
  unsigned long now = millis();

  if (currentState == LOW && lastState == HIGH && (now - lastPressTime > DEBOUNCE_DELAY)) {
    lastPressTime = now;
    lastState = currentState;
    return true;
  }
  lastState = currentState;
  return false;
}

// MCP23017: cached port state (updated in readMcp()); LOW = pressed (pull-up)
static uint8_t cachedPortA = 0xFF, cachedPortB = 0xFF;
static uint32_t mainLoopCounter = 0;
static uint32_t mcpLastReadLoopCounter = 0;
static int lastSourceSeen = -1;

bool buttonPressedMcp(uint8_t mcpPin, bool &lastState, unsigned long &lastPressTime) {
  bool currentState;
  if (mcpPin <= 7)
    currentState = (cachedPortA & (1u << mcpPin)) ? HIGH : LOW;
  else
    currentState = (cachedPortB & (1u << (mcpPin - 8))) ? HIGH : LOW;
  unsigned long now = millis();
  if (currentState == LOW && lastState == HIGH && (now - lastPressTime > DEBOUNCE_DELAY)) {
    lastPressTime = now;
    lastState = currentState;
    return true;
  }
  lastState = currentState;
  return false;
}

int32_t getEncoderCount() { return mcpEncoderCount; }
void setEncoderCount(int32_t v) { mcpEncoderCount = v; }

// Read MCP ports, update encoder count (half-quad on PA4/PA5), update cachedPortA/B
// MCP23017: readGPIOAB() low byte = GPIOA, high byte = GPIOB (datasheet order)
void readMcp(bool forceRead = false) {
  if (!mcpOk) return;
  // Avoid multiple I2C transactions per main loop; forceRead bypasses the cache
  // (used in blocking wait loops like "wait until key released").
  if (!forceRead && mcpLastReadLoopCounter == mainLoopCounter) return;
  mcpLastReadLoopCounter = mainLoopCounter;
  uint16_t ab = mcp.readGPIOAB();
  uint8_t portA = ab & 0xFF;
  uint8_t portB = (ab >> 8) & 0xFF;
  cachedPortA = portA;
  cachedPortB = portB;
  yield();
  // Half-quadrature: encoder A=PA4, B=PA5
  uint8_t a = (portA >> 5) & 1, b = (portA >> 4) & 1;  // A=PA5, B=PA4 (inversat)
  if (a != mcpLastEncA) {
    if (a == b) mcpEncoderCount++; else mcpEncoderCount--;
    mcpLastEncA = a;
  }
  if (b != mcpLastEncB) {
    if (b != a) mcpEncoderCount++; else mcpEncoderCount--;
    mcpLastEncB = b;
  }
}

// Default preset freqs (6 memorii radio MEM1..MEM6: PA7,PA6,PA0..PA3); no voltage/channel selection
static const float defaultPresetFreqs[6] = { 93.5f, 96.9f, 100.2f, 105.3f, 92.8f, 96.1f };

// Source id mapping:
// 1=TUN, 2=BT, 3=RPI, 4=PCA, 5=PCD
const char *sourceNames[] = { "", "TUN", "Bluetooth", "Raspberry PI", "PCA", "PCD" };

int Bass = 0;
int Middle = 0;
int Treble = 0;
int Gain = -30;
int Balance = 0;

bool forceFullTimeRedraw = true;
static bool standbyFullRedrawPending = true;

//------------------------------------
// FMRadioController Class
//------------------------------------
class FMRadioController {
public:
  Adafruit_ST7789 tft;
  XPT2046_Touchscreen ts;
  SI4703 radio;
  RDSParser rds;
  // Encoder: MCP23017 PA4(A)/PA5(B), count in getEncoderCount()/setEncoderCount()

  uint16_t rds_b1 = 0, rds_b2 = 0, rds_b3 = 0, rds_b4 = 0;
  int sursa = 1, sursaVeche = 1;
  float currentFrequency = 93.5, oldFrequency = 0;
  int32_t currentVolume = 10, oldVolume = -1;
  bool isMuted = false, isInVolumeMode = true;
  float SET_FREQ = 93.5;
  unsigned long lastMutePress = 0, lastModePress = 0,
                lastSeekPressUp = 0, lastSeekPressDown = 0;
  unsigned long lastFrequencyChangeTime = 0, bootTime = 0;
  unsigned long lastSourceChangeMs = 0;
  unsigned long lastPresetActionMs = 0;
  bool utilMode = false, wasSetup = true;
  bool eqEditMode = false;
  int eqIndex = 0;

  char days[7][3];
  char daysFull[7][12];  // ziua saptamanii cuvant intreg (standby)
  float sensor1MemFreq[6];
  char radioText[65] = { 0 };
  char radioTextnow[65] = { 0 };
  bool rtUpdated = false;

  static FMRadioController *instance;

  unsigned long rtcUpdateMsgMillis = 0;
  bool rtcUpdateSuccess = false;
  bool setMessageActive = false;
  unsigned long setMessageStart = 0;

  // Debounced NVS writes (Preferences). Many UI actions call saveSettings();
  // we coalesce them and write after a short quiet period to reduce lag and flash wear.
  bool settingsDirty = false;
  unsigned long settingsDirtySinceMs = 0;
  unsigned long lastSettingsWriteMs = 0;

  FMRadioController()
    : tft(TFT_CS, TFT_DC, TFT_RST),
      ts(TOUCH_CS_PIN, TOUCH_IRQ),
      currentFrequency(DEFAULT_FREQ / 100.0),
      currentVolume(DEFAULT_VOLUME),
      SET_FREQ(DEFAULT_FREQ / 100.0) {
    // day names (short)
    strcpy(days[0], "Du");
    strcpy(days[1], "Lu");
    strcpy(days[2], "Ma");
    strcpy(days[3], "Mi");
    strcpy(days[4], "Jo");
    strcpy(days[5], "Vi");
    strcpy(days[6], "Sa");
    // day names full (standby)
    strcpy(daysFull[0], "Duminica");
    strcpy(daysFull[1], "Luni");
    strcpy(daysFull[2], "Marti");
    strcpy(daysFull[3], "Miercuri");
    strcpy(daysFull[4], "Joi");
    strcpy(daysFull[5], "Vineri");
    strcpy(daysFull[6], "Sambata");
    for (int i = 0; i < 6; i++) sensor1MemFreq[i] = defaultPresetFreqs[i];
    instance = this;
  }

  // Encoder switch (MCP PB2): push-release for EQ cycle in util mode
  bool eqButtonPressed() {
    if (!mcpOk) return false;
    if ((cachedPortB & (1u << (MCP_PB2_ENC_SW - 8))) == 0) {
      delay(50);
      while (mcpOk && (cachedPortB & (1u << (MCP_PB2_ENC_SW - 8))) == 0) {
        readMcp(true);
        delay(10);
      }
      return true;
    }
    return false;
  }

  // Cycle EQ parameter and update display labels accordingly
  void cycleEqParameter() {
    if (!eqEditMode) {
      eqEditMode = true;
      eqIndex = 0;
      // Serial.println("EQ mode activated: Bass");
    } else {
      eqIndex = (eqIndex + 1) % 4;
      if (eqIndex == 0) {
        // Serial.println("Switched to Bass");
      } else if (eqIndex == 1) {
        // Serial.println("Switched to Mids");
      } else if (eqIndex == 2) {
        // Serial.println("Switched to Treble");
      } else if (eqIndex == 3) {
        // Serial.println("Switched to Gain");
      }
    }
    // Redraw the full EQ setup screen when cycling parameters.
    drawsetScreen();
    showTime();  // redraw time/date on top
  }

  // Update only the numeric value display for the active EQ parameter.
  void updateCurrentEqValueDisplay() {
    const int valueY = 134;
    const int valueX[4] = { 14, 94, 174, 259 };
    int currentVal = 0;
    switch (eqIndex) {
      case 0: currentVal = Bass; break;
      case 1: currentVal = Middle; break;
      case 2: currentVal = Treble; break;
      case 3: currentVal = Gain; break;
    }
    // Clear only the area where the value is shown.
    tft.fillRect(valueX[eqIndex], valueY, 40, 20, ST77XX_BLACK);
    tft.setCursor(valueX[eqIndex], valueY);
    tft.setTextColor(ST77XX_YELLOW);
    char buf[5];
    sprintf(buf, "%3d", currentVal);
    tft.print(buf);
  }

  // New function: When switching sursa in util mode, only clear/update the EQ values.
  void updateUtilSursaChange() {
    const int valueY = 134;
    const int valueX[4] = { 14, 94, 174, 259 };
    int values[4] = { Bass, Middle, Treble, Gain };
    tft.setTextSize(2);
    for (int i = 0; i < 4; i++) {
      // Only clear the numeric value area (no full redraw of top bars).
      tft.fillRect(valueX[i], valueY, 40, 20, ST77XX_BLACK);
      tft.setCursor(valueX[i], valueY);
      // Highlight the active parameter if in EQ edit mode.
      if (eqEditMode && (i == eqIndex))
        tft.setTextColor(ST77XX_YELLOW);
      else
        tft.setTextColor(ST77XX_WHITE);
      char buf[5];
      sprintf(buf, "%3d", values[i]);
      tft.print(buf);
    }
  }

  void drawUtilButton() {
    tft.fillRect(0, 60, 320, 200, ST77XX_BLACK);
    // RTC button (bottom left)
    tft.fillRect(0, 200, 60, 40, ST77XX_BLUE);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 212);
    tft.print("RTC");
    // Util/Back button (bottom right)
    tft.fillRect(SETUP_BUTTON_X_MIN, SETUP_BUTTON_Y_MIN, SETUP_BUTTON_WIDTH, SETUP_BUTTON_HEIGHT, ST77XX_RED);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(2);
    tft.setCursor(267, 212);
    tft.print("Back");
    // If RTC update was successful, show message
    if (rtcUpdateSuccess && millis() - rtcUpdateMsgMillis < 2000) {
      tft.fillRect(70, 200, 180, 40, ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(90, 215);
      tft.print("RTC Updated!");
      // Cover 2-3 white pixels that appear to the right of the text (font spill)
      int cx = tft.getCursorX();
      if (cx < 257) tft.fillRect(cx, 215, 4, 14, ST77XX_BLACK);
      // Do NOT show IP address while RTC message is visible
      return;
    }
    // Always show IP address in the center bottom area in util mode
    if (WiFi.status() == WL_CONNECTED) {
      tft.fillRect(70, 200, 180, 40, ST77XX_BLACK);  // clear center so IP is on black
      IPAddress ip = WiFi.localIP();
      char ipStr[20];
      sprintf(ipStr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setTextSize(2);
      tft.setCursor(90, 215);
      tft.print(ipStr);
      // Cover 2-3 white pixels that appear to the right of the text (font spill)
      int cx = tft.getCursorX();
      if (cx < 257) tft.fillRect(cx, 215, 4, 14, ST77XX_BLACK);
    }
  }

  void initDisplay() {
    tft.init(SCREEN_HEIGHT, SCREEN_WIDTH);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.invertDisplay(false);
    drawInitialScreen();
  }

  void initRadio() {
    Serial.println("[I2C] Starting radio initialization...");
    
    // First check if I2C bus is working
    Wire.begin();
    Wire.beginTransmission(SI4703_ADDR);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.print("[I2C] Error during I2C initialization: ");
      Serial.println(error);
      Serial.println("[I2C] Please check:");
      Serial.println("1. SDA and SCL connections");
      Serial.println("2. Power supply to SI4703");
      Serial.println("3. I2C address (should be 0x10)");
      si4703Powered = false;
      return;
    }
    
    Serial.println("[I2C] I2C bus OK, initializing radio...");
    
    radio.setup(RADIO_SDAPIN, RADIO_SCLPIN);
    radio.setup(RADIO_FMSPACING, RADIO_FMSPACING_100);
    radio.setup(RADIO_DEEMPHASIS, RADIO_DEEMPHASIS_50);
    radio.initWire(Wire);
    
    // Verify radio initialization
    Wire.beginTransmission(SI4703_ADDR);
    error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.print("[I2C] Error after radio initialization: ");
      Serial.println(error);
      si4703Powered = false;
      return;
    }
    
    Serial.println("[I2C] Radio initialized successfully");
    radio.setBandFrequency(RADIO_BAND, DEFAULT_FREQ);
    radio.setMono(false);
    radio.setMute(false);
    radio.setVolume(DEFAULT_VOLUME);
    radio.setup(RADIO_RESETPIN, RESET_PIN);
    si4703Powered = true;
  }

  void initButtons() {
    pinMode(MCP_INT_A_PIN, INPUT);
    pinMode(MCP_INT_B_PIN, INPUT);
    lastMutePress = lastModePress = lastSeekPressUp = lastSeekPressDown = 0;
  }

  void drawInitialScreen() {
    tft.setTextWrap(false);
    // Top source bar: 5 buttons across 320px => 64px each
    const int w = 64;
    tft.fillRect(0 * w, 0, w, 60, CUSTOM_DARKGREEN); // PCA
    tft.fillRect(1 * w, 0, w, 60, ST77XX_RED);      // PCD
    tft.fillRect(2 * w, 0, w, 60, CUSTOM_GREY);     // RPI
    tft.fillRect(3 * w, 0, w, 60, ST77XX_BLUE);     // BT
    tft.fillRect(4 * w, 0, w, 60, ST77XX_ORANGE);   // TUN
    // Underline band
    tft.fillRect(0, 50, 320, 15, ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(6, 20);   tft.print("PCA");
    tft.setCursor(70, 20);  tft.print("PCD");
    tft.setCursor(138, 20); tft.print("RPI");
    tft.setCursor(214, 20); tft.print("BT");
    tft.setCursor(270, 20); tft.print("TUN");
    drawSetupButton();
    updateMainContent();
    updateVolumeDisplay();
    updateRpiPlayIndicator(true);
  }

  void drawSetupButton() {
    tft.fillRect(SETUP_BUTTON_X_MIN, SETUP_BUTTON_Y_MIN, SETUP_BUTTON_WIDTH, SETUP_BUTTON_HEIGHT, ST77XX_RED);
    tft.setTextColor(ST77XX_GREEN);
    tft.setTextSize(2);
    tft.setCursor(268, 212);
    tft.print("Util");
  }

  // Redraw the full EQ setup screen.
  void drawsetScreen() {
    tft.fillRect(0, 100, SCREEN_WIDTH, 100, ST77XX_BLACK);
    tft.setTextSize(2);
    const int labelY = 67;
    const char *labels[] = { "Bass", "Mids", "Treb", "Gain" };
    const int labelX[] = { 17, 97, 177, 257 };
    for (int i = 0; i < 4; i++) {
      tft.setCursor(labelX[i], labelY);
      if (utilMode && (i == eqIndex))
        tft.setTextColor(ST77XX_YELLOW);
      else
        tft.setTextColor(ST77XX_WHITE);
      tft.print(labels[i]);
    }
    // tft.setCursor(17, 210);
    // tft.setTextColor(ST77XX_WHITE);
    // tft.print("Link settings");

    const int rectWidth = 76;
    const int rectHeight = 40;
    const int xStep = 80;
    const int plusRectY = 88;
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_BLACK);
    for (int i = 0; i < 4; i++) {
      int rectX = i * xStep;
      tft.fillRect(rectX, plusRectY, rectWidth, rectHeight, ST77XX_WHITE);
      tft.setCursor(rectX + 30, plusRectY + 10);
      tft.print("+");
    }
    const int minusRectY = 156;
    for (int i = 0; i < 4; i++) {
      int rectX = i * xStep;
      tft.fillRect(rectX, minusRectY, rectWidth, rectHeight, CUSTOM_LIGHTGREY);
      tft.setCursor(rectX + 30, minusRectY + 12);
      tft.print("-");
    }
    tft.setTextSize(2);
    const int valueY = 134;
    int values[] = { Bass, Middle, Treble, Gain };
    const int valueX[] = { 14, 94, 174, 259 };
    for (int i = 0; i < 4; i++) {
      tft.setCursor(valueX[i], valueY);
      char buf[5];
      sprintf(buf, "%3d", values[i]);
      if (utilMode && (i == eqIndex))
        tft.setTextColor(ST77XX_YELLOW);
      else
        tft.setTextColor(ST77XX_WHITE);
      tft.print(buf);
    }
  }

  void toggleUtilMode() {
    static unsigned long lastToggle = 0;
    if (millis() - lastToggle < 700) return;  // debounce mai mare ca sa nu se faca ecran alb
    lastToggle = millis();
    yield();
    if (utilMode) {
        // Intrare mod util: golim zona, apoi desenam (evita ecran alb / garbage)
        eqEditMode = false;
        setEncoderCount(0);
        tft.fillRect(0, 60, SCREEN_WIDTH, SCREEN_HEIGHT - 60, ST77XX_BLACK);
        yield();
        drawUtilButton();
        yield();
        drawsetScreen();
        yield();
        showTime();
        wasSetup = true;
        lastUtilActivity = millis();
    } else {
        // Exiting util mode
        tft.fillRect(0, 60, 320, 200, ST77XX_BLACK);
        drawInitialScreen();
        updateMainContent();
        updateVolumeDisplay();
        updateSelectedSourceDisplay();
        showTime();
        wasSetup = false;
        setEncoderCount(currentVolume);
        encoderLastEncVal = currentVolume;
    }
    // Print IP address to Serial when util mode is activated
    if (utilMode && WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        char ipStr[20];
        sprintf(ipStr, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        // Serial.print("Device IP address: ");
        // Serial.println(ipStr);
    }
  }

  void updateSelectedSourceDisplay() {
    // Red underline under selected source in the 5-button top bar.
    const int w = 64;
    tft.fillRect(0, 50, 320, 15, ST77XX_BLACK);
    int idx = 0;
    if (sursa == 4) idx = 0;       // PCA
    else if (sursa == 5) idx = 1;  // PCD
    else if (sursa == 3) idx = 2;  // RPI
    else if (sursa == 2) idx = 3;  // BT
    else if (sursa == 1) idx = 4;  // TUN
    tft.fillRect(idx * w, 50, w, 15, ST77XX_RED);
  }

  void updateMainContent() {
    static int lastRenderedSource = -1;
    // On any source change, clear the full content area once
    // to prevent leftovers between differently laid out screens.
    if (lastRenderedSource != -1 && lastRenderedSource != sursa) {
      tft.fillRect(0, 60, 320, 140, ST77XX_BLACK);
      // Force full time/date repaint after leaving/entering RPI layouts.
      forceFullTimeRedraw = true;
    }
    tft.fillRect(0, 110, 320, 50, ST77XX_BLACK);
    if (sursa == 1) {
      tft.fillRect(25, 160, 220, 40, ST77XX_BLACK);
      tft.setTextColor(ST77XX_ORANGE);
      tft.setTextSize(3);
      tft.setCursor(25, 165);
      tft.print(currentFrequency, 2);
      tft.print(" MHz");
      drawSeekButton();
      if (rtUpdated) {
        displayRadioText();
        rtUpdated = false;
      }
    } else if (sursa == 3) {
      tft.fillRect(0, 60, 320, 140, ST77XX_BLACK);
      // Transport row: 3×70px + reserved slot (touch uses tx<=71/143/214 + offset for first three).
      tft.fillRect(TRANSPORT_BTN0_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, CUSTOM_DARKGREEN);
      tft.fillRect(TRANSPORT_BTN1_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, ST77XX_ORANGE);
      tft.fillRect(TRANSPORT_BTN2_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, CUSTOM_DARKGREEN);
      tft.setTextColor(ST77XX_WHITE);
      // PREV: two filled left-pointing triangles
      tft.fillTriangle(20, 171, 34, 161, 34, 181, ST77XX_WHITE);
      tft.fillTriangle(38, 171, 52, 161, 52, 181, ST77XX_WHITE);
      updateRpiTransportPlayIcon(true);
      // NEXT: two filled right-pointing triangles
      tft.fillTriangle(166, 161, 166, 181, 180, 171, ST77XX_WHITE);
      tft.fillTriangle(184, 161, 184, 181, 198, 171, ST77XX_WHITE);
      {
        // RPI source: show RPi metadata
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_CYAN);
        tft.fillRect(0, 77, SCREEN_WIDTH, 10, ST77XX_BLACK);
        tft.setCursor(0, 77);
        tft.print("RPI:");
        tft.print(rpiConnected ? " ONLINE " : " OFFLINE ");
        tft.print(" ");
        tft.print(rpiStateBuf);
        // File/format line uses full width (below the status row).
        tft.fillRect(0, 88, SCREEN_WIDTH, 10, ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);
        tft.setCursor(0, 90);
        {
          char fileDisp[40];
          fmtShorten(rpiFileBuf, 36, fileDisp, sizeof(fileDisp));
          tft.print(fileDisp);
        }
        tft.setTextColor(ST77XX_CYAN);

        updateRpiTitleTicker(true);
        tft.fillRect(0, 123, SCREEN_WIDTH, 20, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, 125);
        {
          char artistDisp[40];
          fmtShorten(rpiArtistBuf, 30, artistDisp, sizeof(artistDisp));
          tft.print(artistDisp);
        }
      }
    } else if (sursa == 4) {
      // PCA: analog input (IN1) - show "PC analogic" overlay.
      tft.fillRect(0, 60, 320, 140, ST77XX_BLACK);
      // No transport buttons on PCA.
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_WHITE);
      const char *msg = "PC analogic";
      int16_t x1, y1;
      uint16_t w, h;
      tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
      const int cx = (SCREEN_WIDTH - (int)w) / 2;
      const int cy = 60 + (140 - (int)h) / 2;
      tft.setCursor(cx, cy);
      tft.print(msg);
      pcAnalogUi = true;
    } else if (sursa == 5) {
      // PCD: foobar metadata/controls.
      pcAnalogUi = false;
      tft.fillRect(0, 60, 320, 140, ST77XX_BLACK);
      tft.fillRect(TRANSPORT_BTN0_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, CUSTOM_DARKGREEN);
      tft.fillRect(TRANSPORT_BTN1_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, ST77XX_ORANGE);
      tft.fillRect(TRANSPORT_BTN2_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, CUSTOM_DARKGREEN);
      tft.setTextColor(ST77XX_WHITE);
      // PREV
      tft.fillTriangle(20, 171, 34, 161, 34, 181, ST77XX_WHITE);
      tft.fillTriangle(38, 171, 52, 161, 52, 181, ST77XX_WHITE);
      updateRpiTransportPlayIcon(true);
      // NEXT
      tft.fillTriangle(166, 161, 166, 181, 180, 171, ST77XX_WHITE);
      tft.fillTriangle(184, 161, 184, 181, 198, 171, ST77XX_WHITE);
      updatePcNowPlayingUi(true);
    } else {
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(3);
      tft.setCursor(60, 135);
      tft.fillRect(0, 120, 320, 70, ST77XX_BLACK);
      tft.print(sourceNames[sursa]);
    }
    lastRenderedSource = sursa;
  }

  void drawSeekButton() {
    tft.fillTriangle(280, 160, 310, 175, 280, 190, CUSTOM_DARKGREEN);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(285, 172);
    tft.print("UP");
    tft.fillTriangle(245, 160, 215, 175, 245, 190, CUSTOM_DARKGREEN);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(230, 172);
    tft.print("DN");
  }

  void updateVolumeDisplay() {
    if (utilMode) return;
    int barWidth = map(currentVolume, MIN_VOLUME, MAX_VOLUME, 0, SCREEN_WIDTH - 60);
    tft.fillRect(barWidth, 200, SCREEN_WIDTH - barWidth - 60, 40, CUSTOM_GREY);
    tft.fillRect(0, 200, barWidth, 40, (isMuted ? ST77XX_RED : ST77XX_BLUE));
    if (!isMuted)
      updateModeDisplay();
  }

  void updateModeDisplay() {
    if (utilMode) return;
    if (sursa == 1) {
      if (isInVolumeMode) {
        tft.fillTriangle(0, 190, 20, 175, 0, 160, ST77XX_BLACK);
        tft.fillTriangle(0, 235, 20, 220, 0, 205, ST77XX_RED);
      } else {
        tft.fillTriangle(0, 190, 20, 175, 0, 160, ST77XX_RED);
        int barWidth = map(currentVolume, MIN_VOLUME, MAX_VOLUME, 0, SCREEN_WIDTH - 60);
        tft.fillTriangle(0, 235, 20, 220, 0, 205, (barWidth > 20 ? ST77XX_BLUE : CUSTOM_GREY));
      }
    } else {
      tft.fillTriangle(0, 190, 20, 175, 0, 160, ST77XX_BLACK);
    }
  }

  void displayRadioText() {
    if (sursa != 1) return;
    tft.fillRect(5, 110, SCREEN_WIDTH - 10, 40, ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    const int maxCharsPerRow = 25;
    int len = strlen(radioText);
    if (len <= maxCharsPerRow) {
      tft.setCursor(5, 120);
      tft.print(radioText);
    } else {
      char row1[maxCharsPerRow + 1] = { 0 };
      char row2[maxCharsPerRow + 1] = { 0 };
      strncpy(row1, radioText, maxCharsPerRow);
      strncpy(row2, radioText + maxCharsPerRow, maxCharsPerRow);
      tft.setCursor(5, 110);
      tft.print(row1);
      tft.setCursor(5, 130);
      tft.print(row2);
    }
  }

  bool buttonPressed(uint8_t pin, bool &lastState, unsigned long &lastPressTime) {
    bool currentState = digitalRead(pin);
    unsigned long now = millis();

    if (currentState == LOW && lastState == HIGH && (now - lastPressTime > DEBOUNCE_DELAY)) {
      lastPressTime = now;
      lastState = currentState;
      return true;
    }
    lastState = currentState;
    return false;
  }

  void handleButtons() {
    readMcp();
    static bool lastNormalState = HIGH;
    static bool lastMuteState = HIGH, lastMode3State = HIGH;
    // In standby: doar MODE (PB5) poate trezi; restul se ignora
    if (inStandby) {
      if (mcpOk && buttonPressedMcp(MCP_PB5_MODE, lastMode3State, lastModePress)) {
        exitStandby();
      }
      return;
    }
    if (mcpOk && buttonPressedMcp(MCP_PB4_MUTE, lastMuteState, lastMutePress))
      toggleMute();
    // Buton MODE (PB5) = standby (temporar)
    if (mcpOk && buttonPressedMcp(MCP_PB5_MODE, lastMode3State, lastModePress)) {
      standbyState = !standbyState;
      if (standbyState) enterStandby();
      else exitStandby();
    }
    if (utilMode == 1) {
      if (eqButtonPressed())
        cycleEqParameter();
    } else {
      if (mcpOk && buttonPressedMcp(MCP_PB2_ENC_SW, lastNormalState, lastModePress) && !isMuted)
        toggleMode();
    }
    if (sursa != 1) return;
    if (utilMode == 1) return;
  }

  void handleEncoder() {
    if (inStandby) return;
    readMcp();
    if (utilMode) {
        static unsigned long lastEncoderUtilMs = 0;
        static int encoderUtilAccum = 0;  // divizare cu 2: 2 pasi encoder = 1 pas valoare
        const unsigned long ENCODER_UTIL_COOLDOWN_MS = 250;
        int32_t encVal = getEncoderCount();
        int step = (encVal > 0) ? 1 : ((encVal < 0) ? -1 : 0);
        if (step != 0) {
            setEncoderCount(0);
            encoderUtilAccum += step;
        }
        int applyStep = 0;
        if (encoderUtilAccum >= 2) { applyStep = 1; encoderUtilAccum -= 2; }
        else if (encoderUtilAccum <= -2) { applyStep = -1; encoderUtilAccum += 2; }
        if (applyStep != 0 && (millis() - lastEncoderUtilMs) >= ENCODER_UTIL_COOLDOWN_MS) {
            lastEncoderUtilMs = millis();
            step = applyStep;
            eqEditMode = true;
            if (eqIndex == 0) {  // Bass
                Bass = constrain(Bass + step, -7, 7);
                tda7439.setSnd(Bass, 1);
            } else if (eqIndex == 1) {  // Mids
                Middle = constrain(Middle + step, -7, 7);
                tda7439.setSnd(Middle, 2);
            } else if (eqIndex == 2) {  // Treble
                Treble = constrain(Treble + step, -7, 7);
                tda7439.setSnd(Treble, 3);
            } else if (eqIndex == 3) {  // Gain
                Gain = constrain(Gain + step, -45, 0);
                tda7439.inputGain((Gain + 45) / 3);
            }
            updateCurrentEqValueDisplay();
            saveSettings();
            flushSettingsIfDue(true);
            lastUtilActivity = millis();
        }
        return;
    }
    if (isMuted) {
        setEncoderCount(currentVolume);
        return;
    }
    if (isInVolumeMode) {
        int32_t encVal = getEncoderCount();
        int32_t diff = encVal - encoderLastEncVal;
        int32_t steps = diff / 2; // 2 pulses per step
        steps = constrain(steps, -1, 1);  // max 1 pas per citire ca sa nu sara volumul
        if (steps != 0) {
            int32_t newVolume = currentVolume + steps;
            if (newVolume < MIN_VOLUME) newVolume = MIN_VOLUME;
            if (newVolume > MAX_VOLUME) newVolume = MAX_VOLUME;
            if (newVolume != currentVolume) {
                currentVolume = newVolume;
                updateVolumeDisplay();
                oldVolume = currentVolume;
                
                Wire.beginTransmission(0x44);
                byte error = Wire.endTransmission();
                if (error == 0) {
                    tda7439.setVolume(currentVolume);
                    delay(10);
                } else {
                    // Serial.print("TDA7439 Error during encoder volume change: ");
                    // Serial.println(error);
                }
                
                saveSettings();
            }
            encoderLastEncVal += steps * 2;
        }
    } else {
        int32_t encoderValue = getEncoderCount();
        float newFreq = constrain(encoderValue * 0.1, MIN_FREQ / 100.0, MAX_FREQ / 100.0);
        if (fabs(newFreq - currentFrequency) >= 0.1) {
            currentFrequency = newFreq;
            if (si4703Powered) radio.setFrequency(currentFrequency * 100);
            clearRDSData();
            updateMainContent();
            updateModeDisplay();
            oldFrequency = currentFrequency;
            lastFrequencyChangeTime = millis();
            saveSettings();
            lastWebFreqChange = millis();
        }
    }
  }

  void readSensors() {
    readMcp();
    if (!mcpOk) return;
    static bool skipNotified = false;
    if (millis() - lastWebFreqChange < 2000) {
      if (!skipNotified) { skipNotified = true; }
      return;
    } else {
      skipNotified = false;
    }
    if (isMuted) return;

    // MEM = statie radio memorata: PA7=MEM1, PA6=MEM2, PA0=MEM3, PA1=MEM4, PA2=MEM5, PA3=MEM6
    static const uint8_t memPin[6] = { MCP_PA7_MEM1, MCP_PA6_MEM2, MCP_PA0_MEM3, MCP_PA1_MEM4, MCP_PA2_MEM5, MCP_PA3_MEM6 };
    bool presetActive = false;
    int presetIndex = -1;
    for (int i = 0; i < 6; i++) {
      if (!(cachedPortA & (1u << memPin[i]))) {
        presetActive = true;
        presetIndex = i;
        clearRDSData();
        break;
      }
    }
    if (presetActive) {
      if (lastMcpPresetIndex != presetIndex) {
        lastMcpPresetPress[presetIndex] = millis();
        lastMcpPresetIndex = presetIndex;
        clearRDSData();
      } else {
        if (millis() - lastMcpPresetPress[presetIndex] >= 3000) {
          sensor1MemFreq[presetIndex] = currentFrequency;
          tft.setCursor(249, 140);
          tft.setTextColor(ST77XX_GREEN);
          tft.setTextSize(2);
          tft.print("Set");
          setMessageActive = true;
          setMessageStart = millis();
          lastMcpPresetPress[presetIndex] = millis() + 10000;
        }
      }
      mcpPresetWasActive = true;
    } else {
      if (mcpPresetWasActive && lastMcpPresetIndex >= 0 && lastMcpPresetIndex < 6) {
        unsigned long duration = millis() - lastMcpPresetPress[lastMcpPresetIndex];
        if (duration < 3000 && millis() - lastPresetActionMs > 500) {
          lastPresetActionMs = millis();
          float newFreq = sensor1MemFreq[lastMcpPresetIndex];
          if (fabs(newFreq - currentFrequency) >= 0.1) {
            currentFrequency = newFreq;
            yield();
            if (si4703Powered) radio.setFrequency(currentFrequency * 100);
            yield();
            updateMainContent();
            yield();
            saveSettings();
          }
        }
      }
      lastMcpPresetIndex = -1;
      mcpPresetWasActive = false;
    }

    // Surse: PB6=TUN(1), PB7=BT(2), PB0=RPI(3), PB1=PC(4) (LOW = pressed)
    int newSource = sursa;
    if (!(cachedPortB & (1u << 6))) newSource = 1;   // TUN
    else if (!(cachedPortB & (1u << 7))) newSource = 2;  // BT
    else if (!(cachedPortB & (1u << 0))) newSource = 3;  // RPI
    else if (!(cachedPortB & (1u << 1))) newSource = 4;  // PC
    if (newSource >= 1 && newSource <= 4 && newSource != sursaVeche) {
      unsigned long now = millis();
      if (now - lastSourceChangeMs < 500) return;
      lastSourceChangeMs = now;
      // Physical source select never enters PCD; PCD is a WebUI-only virtual source (5).
      pcdMode = false;
      sursa = newSource;
      if (sursa == 3) {
        // User explicitly selected RPI on the physical panel; arm the bridge.
        rpiBridgeArmed = true;
      }
      yield();
      Wire.beginTransmission(0x44);
      byte error = Wire.endTransmission();
      if (error == 0) {
        tda7439.setVolume(currentVolume);
        tda7439.setSnd(Bass, 1);
        tda7439.setSnd(Middle, 2);
        tda7439.setSnd(Treble, 3);
        tda7439.inputGain((Gain + 45) / 3);
        setTDA7439InputForSource(sursa);
        // No extra PC audio routing mux.
      }
      yield();
      updateVolumeDisplay();
      yield();
      updateSelectedSourceDisplay();
      yield();
      updateMainContent();
      yield();
      loadEqualizer();
      sursaVeche = sursa;
      yield();
      saveSettings();
    }

    if (presetActive && lastMcpPresetIndex >= 0 && lastMcpPresetIndex == presetIndex && millis() - lastMcpPresetPress[presetIndex] >= 3000) {
      if (millis() - lastPresetActionMs > 400) {
        lastPresetActionMs = millis();
        sensor1MemFreq[presetIndex] = currentFrequency;
        saveSettings();
      }
    }
  }

  void toggleMute() {
    if (utilMode) return;
    isMuted = !isMuted;
    
    // Check TDA7439 before applying mute
    Wire.beginTransmission(0x44);
    byte error = Wire.endTransmission();
    if (error == 0) {
      // Apply mute/unmute to TDA7439
      if (isMuted) {
        tda7439.setVolume(0);
        noteTdaWrite("mute", -1, 0);
        // Serial.println("[TDA7439] Muted");
      } else {
        tda7439.setVolume(currentVolume);
        noteTdaWrite("mute", -1, currentVolume);
        // Serial.print("[TDA7439] Unmuted, restoring volume: ");
        // Serial.println(currentVolume);
      }
    } else {
      // Serial.print("[TDA7439] NACK Error during mute toggle! Error code: ");
      // Serial.println(error);
    }
    
    if (isMuted) {
        setEncoderCount(currentVolume);
    } else {
        setEncoderCount(currentVolume);
        encoderLastEncVal = currentVolume;
    }
    updateVolumeDisplay();
    saveSettings();
  }

  void toggleMode() {
    static unsigned long lastModeToggle = 0;
    const unsigned long debounceTime = 100;

    if (sursa != 1) return;

    unsigned long now = millis();
    if (now - lastModeToggle < debounceTime) return;
    lastModeToggle = now;

    isInVolumeMode = !isInVolumeMode;

    if (!isMuted) {
        if (isInVolumeMode) {
            setEncoderCount(currentVolume);
            encoderLastEncVal = getEncoderCount();
        } else {
            setEncoderCount(currentFrequency * 10);
            lastFrequencyChangeTime = millis();
        }
    }
    updateModeDisplay();
  }

  void seekUp() {
    if (sursa != 1) return;
    // Serial.println("[SEEK] Starting seek up");
    // Preflight: if bus/device not OK, try reinit to avoid freezes
    if (!detectSI4703()) {
      // Serial.println("[SEEK] SI4703 not responding; attempting reinit...");
      if (!reinitSI4703()) {
        // Serial.println("[SEEK] Reinit failed; aborting seek.");
        return;
      }
    }
    if (si4703Powered) {
      radio.seekUp(true);
      delay(100); // Give radio time to seek
      currentFrequency = radio.getFrequency() / 100.0;
      // Serial.print("[SEEK] Found frequency: ");
      // Serial.println(currentFrequency);
      oldFrequency = currentFrequency;
      clearRDSData();
      updateMainContent();
      saveSettings();
      lastWebFreqChange = millis();
    } else {
      // Serial.println("[SEEK] Radio not powered, cannot seek");
    }
  }


  void seekDown() {
    if (sursa != 1) return;
    // Serial.println("[SEEK] Starting seek down");
    // Preflight: if bus/device not OK, try reinit to avoid freezes
    if (!detectSI4703()) {
      // Serial.println("[SEEK] SI4703 not responding; attempting reinit...");
      if (!reinitSI4703()) {
        // Serial.println("[SEEK] Reinit failed; aborting seek.");
        return;
      }
    }
    if (si4703Powered) {
      radio.seekDown(true);
      delay(100); // Give radio time to seek
      currentFrequency = radio.getFrequency() / 100.0;
      // Serial.print("[SEEK] Found frequency: ");
      // Serial.println(currentFrequency);
      oldFrequency = currentFrequency;
      clearRDSData();
      updateMainContent();
      saveSettings();
      lastWebFreqChange = millis();
    } else {
      // Serial.println("[SEEK] Radio not powered, cannot seek");
    }
  }

  void clearRDSData() {
    tft.fillRect(5, 110, SCREEN_WIDTH - 10, 40, ST77XX_BLACK);
    memset(radioText, 0, sizeof(radioText));
    memset(radioTextnow, 0, sizeof(radioTextnow));
    rtUpdated = false;
    rds_b1 = rds_b2 = rds_b3 = rds_b4 = 0;
    strcpy(radioText, " ");
    strcpy(radioTextnow, " ");
  }

  void mapTouch(int rawX, int rawY, int &mappedX, int &mappedY) {
    // Keep mapped coordinates inside display bounds.
    mappedX = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_WIDTH - 1);
    mappedY = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_HEIGHT - 1);
    mappedX = constrain(mappedX, 0, SCREEN_WIDTH - 1);
    mappedY = constrain(mappedY, 0, SCREEN_HEIGHT - 1);
#if TOUCH_MIRROR_X
    mappedX = (SCREEN_WIDTH - 1) - mappedX;
#endif
#if TOUCH_MIRROR_Y
    mappedY = (SCREEN_HEIGHT - 1) - mappedY;
#endif
  }

  void saveSettings() {
    settingsDirty = true;
    settingsDirtySinceMs = millis();
  }

  void flushSettingsIfDue(bool force = false) {
    if (!settingsDirty) return;
    const unsigned long QUIET_MS = 900UL;
    const unsigned long MIN_INTERVAL_MS = 1200UL;
    unsigned long now = millis();
    if (!force) {
      if (now - settingsDirtySinceMs < QUIET_MS) return;
      if (now - lastSettingsWriteMs < MIN_INTERVAL_MS) return;
    }
    saveSettingsNow();
    lastSettingsWriteMs = now;
    settingsDirty = false;
  }

  void saveSettingsNow() {
    // #region agent log: H3 count NVS writes
    profNvsWrites++;
    // #endregion
    preferences.begin("settings", false);
    preferences.putInt("sursa", sursa);
    preferences.putInt("freq", (int)(currentFrequency * 100));
    // Per-source volume (include PCD virtual id=5); keep legacy "volume" for backward compat.
    int sid = sursa;
    preferences.putInt(("vol" + String(sid)).c_str(), currentVolume);
    preferences.putInt("volume", currentVolume);
    // Legacy flag kept for backward compatibility.
    preferences.putBool("pcdMode", (sursa == 5));

    String keyBass = "bass" + String(sid);
    String keyMiddle = "mid" + String(sid);
    String keyTreble = "treble" + String(sid);
    String keyGain = "gain" + String(sid);
    String keyBalance = "bal" + String(sid);
    preferences.putInt(keyBass.c_str(), Bass);
    preferences.putInt(keyMiddle.c_str(), Middle);
    preferences.putInt(keyTreble.c_str(), Treble);
    preferences.putInt(keyGain.c_str(), Gain);
    preferences.putInt(keyBalance.c_str(), Balance);
    for (int i = 0; i < 6; i++) {
      preferences.putFloat(("p" + String(i) + "f").c_str(), sensor1MemFreq[i]);
    }
    preferences.end();
  }

  void loadSettings() {
    preferences.begin("settings", true);
    sursa = preferences.getInt("sursa", 2);
    int freqInt = preferences.getInt("freq", DEFAULT_FREQ);
    currentFrequency = freqInt / 100.0;
    // Migrate legacy pcdMode (old: sursa=3 + pcdMode=true) to real source id=5.
    pcdMode = preferences.getBool("pcdMode", false);
    if (sursa == 3 && pcdMode) {
      sursa = 5;
      pcdMode = false;
    }
    int sid = sursa;
    currentVolume = preferences.getInt(("vol" + String(sid)).c_str(), preferences.getInt("volume", DEFAULT_VOLUME));
    // Do not force "PC analogic" overlay on PCA; keep it off by default.
    pcAnalogUi = false;
    for (int i = 0; i < 6; i++) {
      sensor1MemFreq[i] = preferences.getFloat(("p" + String(i) + "f").c_str(), defaultPresetFreqs[i]);
    }

    String keyBass = "bass" + String(sid);
    String keyMiddle = "mid" + String(sid);
    String keyTreble = "treble" + String(sid);
    String keyGain = "gain" + String(sid);
    String keyBalance = "bal" + String(sid);
    Bass = preferences.getInt(keyBass.c_str(), 0);
    Middle = preferences.getInt(keyMiddle.c_str(), 0);
    Treble = preferences.getInt(keyTreble.c_str(), 0);
    Gain = preferences.getInt(keyGain.c_str(), -30);
    Balance = preferences.getInt(keyBalance.c_str(), 0);

    preferences.end();
  }

  void loadEqualizer() {
    preferences.begin("settings", true);
    int sid = (sursa == 3 && pcdMode) ? 5 : sursa;
    String keyBass = "bass" + String(sid);
    String keyMiddle = "mid" + String(sid);
    String keyTreble = "treble" + String(sid);
    String keyGain = "gain" + String(sid);
    String keyBalance = "bal" + String(sid);
    Bass = preferences.getInt(keyBass.c_str(), 0);
    Middle = preferences.getInt(keyMiddle.c_str(), 0);
    Treble = preferences.getInt(keyTreble.c_str(), 0);
    Gain = preferences.getInt(keyGain.c_str(), -30);
    Balance = preferences.getInt(keyBalance.c_str(), 0);
    preferences.end();
  }

  void processRDSGroup(uint16_t b1, uint16_t b2, uint16_t b3, uint16_t b4) {
    if (sursa != 1) return;
    rds_b1 = b1;
    rds_b2 = b2;
    rds_b3 = b3;
    rds_b4 = b4;
    uint8_t groupType = (b2 >> 12) & 0x0F;
    if (groupType == 2) {
      uint8_t segmentAddress = b2 & 0x0F;
      char segment[5] = { char(b3 >> 8), char(b3 & 0xFF), char(b4 >> 8), char(b4 & 0xFF), '\0' };
      strncpy(&radioText[segmentAddress * 4], segment, 4);
      if (strcmp(radioText, radioTextnow) != 0) {
        strcpy(radioTextnow, " ");
        rtUpdated = true;
        strcpy(radioTextnow, radioText);
      } else {
        rtUpdated = false;
      }
    }
  }

  static void RDS_process(uint16_t b1, uint16_t b2, uint16_t b3, uint16_t b4) {
    if (instance)
      instance->processRDSGroup(b1, b2, b3, b4);
  }

  void update() {
    handleButtons();
    handleEncoder();
    if (!utilMode) {
      if (!isMuted && !isInVolumeMode && (millis() - lastFrequencyChangeTime > AUTO_MODE_SWITCH_DELAY)) {
        toggleMode();
        lastFrequencyChangeTime = millis();
      }
      // Poll RDS only when radio source is active, and throttle to ~100ms
      static unsigned long lastRdsPoll = 0;
      if (si4703Powered && sursa == 1) {
        unsigned long now = millis();
        if (now - lastRdsPoll >= 100) {
          radio.checkRDS();
          lastRdsPoll = now;
        }
        // Sample RSSI in main loop (avoid I2C from HTTP task)
        if (now - lastFmRssiPoll >= 200) {
          RADIO_INFO info;
          radio.getRadioInfo(&info);
          lastFmRssi = (int)info.rssi;
          lastFmRssiPoll = now;
        }
      }
      if (rtUpdated) updateMainContent();
      readSensors();
    }
    if (setMessageActive && millis() - setMessageStart > 3000) {
      tft.fillRect(249, 140, 35, 15, ST77XX_BLACK);
      setMessageActive = false;
    }
    // Auto-exit util mode after 20 seconds of inactivity
    if (utilMode && (millis() - lastUtilActivity > 20000)) {
      utilMode = false;
      toggleUtilMode();
    }
  }
};


//------------------------------------
// Enter/Exit Standby Helpers
//------------------------------------
FMRadioController *FMRadioController::instance = nullptr;
FMRadioController fmRadio;

static void sendStatusJson(WebServer &srv) {
  // Stream response body via sendContent to avoid heap fragmentation.
  // NOTE: caller must have already set headers + sent the 200 with empty body.
  int rssi = (!standbyState && si4703Powered && fmRadio.sursa == 1) ? (int)lastFmRssi : -1;
  int wifi = webStatusWifiRssi;
  const int sid = fmRadio.sursa;

  WebChunkedWriter w(srv);
  w.write("{");
  w.write("\"power\":\""); w.write(standbyState ? "off" : "on"); w.write("\",");
  w.write("\"mute\":\""); w.write(fmRadio.isMuted ? "on" : "off"); w.write("\",");
  w.write("\"volume\":"); w.writeInt((long)fmRadio.currentVolume); w.write(",");
  w.write("\"source\":"); w.writeInt((long)sid); w.write(",");
  w.write("\"freq\":"); w.writeFloat2((double)fmRadio.currentFrequency); w.write(",");
  w.write("\"rssi\":"); w.writeInt((long)rssi); w.write(",");
  w.write("\"wifi\":"); w.writeInt((long)wifi); w.write(",");

  w.write("\"rds\":");
  if (!standbyState && si4703Powered && fmRadio.sursa == 1) {
    w.writeJsonEscaped(fmRadio.radioTextnow);
  } else {
    w.write("\"\"");
  }
  w.write(",");

  w.write("\"rpiOnline\":"); w.write(rpiConnected ? "true" : "false"); w.write(",");
  w.write("\"rpiState\":");  w.writeJsonEscaped(rpiStateBuf);  w.write(",");
  w.write("\"rpiTitle\":");  w.writeJsonEscaped(rpiTitleBuf);  w.write(",");
  w.write("\"rpiArtist\":"); w.writeJsonEscaped(rpiArtistBuf); w.write(",");
  w.write("\"rpiFile\":");   w.writeJsonEscaped(rpiFileBuf);   w.write(",");

  w.write("\"pcOnline\":"); w.write(pcBridgeOk ? "true" : "false"); w.write(",");
  w.write("\"pcHttpCode\":"); w.writeInt((long)lastPcHttpCode); w.write(",");
  w.write("\"pcState\":");  w.writeJsonEscaped(pcStateBuf);  w.write(",");
  w.write("\"pcTitle\":");  w.writeJsonEscaped(pcTitleBuf);  w.write(",");
  w.write("\"pcArtist\":"); w.writeJsonEscaped(pcArtistBuf); w.write(",");
  w.write("\"pcExtra\":");  w.writeJsonEscaped(pcExtraBuf);  w.write(",");

  w.write("\"bass\":"); w.writeInt((long)Bass); w.write(",");
  w.write("\"middle\":"); w.writeInt((long)Middle); w.write(",");
  w.write("\"treble\":"); w.writeInt((long)Treble); w.write(",");
  w.write("\"gain\":"); w.writeInt((long)Gain); w.write(",");
  w.write("\"balance\":"); w.writeInt((long)Balance);
  w.write("}");
  w.flush();
}

// v2: minimal, non-blocking state snapshot for WebUI.
static void sendStateV2Json(WebServer &srv) {
  const int wifi = webStatusWifiRssi;
  const int rssi = (!standbyState && si4703Powered && fmRadio.sursa == 1) ? (int)lastFmRssi : -1;

  WebChunkedWriter w(srv);

  w.write("{");
  w.write("\"power\":"); w.write(standbyState ? "0" : "1"); w.write(",");
  w.write("\"mute\":"); w.write(fmRadio.isMuted ? "1" : "0"); w.write(",");
  w.write("\"volume\":"); w.writeInt((long)fmRadio.currentVolume); w.write(",");
  w.write("\"source\":"); w.writeInt((long)fmRadio.sursa); w.write(",");

  w.write("\"wifiRssi\":"); w.writeInt((long)wifi); w.write(",");

  // Diagnostics
  w.write("\"diag\":{");
  w.write("\"webTaskLastUs\":"); w.writeInt((long)webTaskLastUs); w.write(",");
  w.write("\"webTaskMaxUs\":"); w.writeInt((long)webTaskMaxUs); w.write(",");
  w.write("\"webTaskLoops\":"); w.writeInt((long)webTaskLoops); w.write(",");
  w.write("\"webTaskLastMs\":"); w.writeInt((long)webTaskLastMs); w.write(",");
  w.write("\"pc\":{");
  w.write("\"okCount\":"); w.writeInt((long)pcOkCount); w.write(",");
  w.write("\"failCount\":"); w.writeInt((long)pcFailCount); w.write(",");
  w.write("\"connectFail\":"); w.writeInt((long)pcConnectFailCount); w.write(",");
  w.write("\"readFail\":"); w.writeInt((long)pcReadFailCount); w.write(",");
  w.write("\"lastOkMs\":"); w.writeInt((long)pcLastOkMs); w.write(",");
  w.write("\"lastFailMs\":"); w.writeInt((long)pcLastFailMs); w.write(",");
  w.write("\"lastFailCode\":"); w.writeInt((long)pcLastFailCode); w.write(",");
  w.write("\"lastDtMs\":"); w.writeInt((long)pcLastDtMs);
  w.write("}");
  w.write("},");

  // Audio debug snapshot (last TDA write that could affect sound)
  w.write("\"audio\":{");
  w.write("\"tdaInput\":"); w.writeInt((long)lastTdaInput); w.write(",");
  w.write("\"tdaVolume\":"); w.writeInt((long)lastTdaVolume); w.write(",");
  w.write("\"lastWriteMs\":"); w.writeInt((long)lastTdaWriteMs); w.write(",");
  w.write("\"why\":"); w.writeJsonEscaped(lastTdaWhy);
  w.write("},");

  // Tuner
  w.write("\"tuner\":{");
  w.write("\"freq\":"); w.writeFloat2((double)fmRadio.currentFrequency); w.write(",");
  w.write("\"rssi\":"); w.writeInt((long)rssi); w.write(",");
  w.write("\"rds\":");
  if (!standbyState && si4703Powered && fmRadio.sursa == 1) w.writeJsonEscaped(fmRadio.radioTextnow);
  else w.write("\"\"");
  w.write("},");

  // RPi
  w.write("\"rpi\":{");
  w.write("\"online\":"); w.write(rpiConnected ? "true" : "false"); w.write(",");
  w.write("\"state\":");  w.writeJsonEscaped(rpiStateBuf);  w.write(",");
  w.write("\"title\":");  w.writeJsonEscaped(rpiTitleBuf);  w.write(",");
  w.write("\"artist\":"); w.writeJsonEscaped(rpiArtistBuf); w.write(",");
  w.write("\"file\":");   w.writeJsonEscaped(rpiFileBuf);
  w.write("},");

  // PC / foobar
  w.write("\"pc\":{");
  w.write("\"online\":"); w.write(pcBridgeOk ? "true" : "false"); w.write(",");
  w.write("\"httpCode\":"); w.writeInt((long)lastPcHttpCode); w.write(",");
  w.write("\"state\":");  w.writeJsonEscaped(pcStateBuf);  w.write(",");
  w.write("\"title\":");  w.writeJsonEscaped(pcTitleBuf);  w.write(",");
  w.write("\"artist\":"); w.writeJsonEscaped(pcArtistBuf); w.write(",");
  w.write("\"extra\":");  w.writeJsonEscaped(pcExtraBuf);
  w.write("},");

  // EQ
  w.write("\"eq\":{");
  w.write("\"bass\":"); w.writeInt((long)Bass); w.write(",");
  w.write("\"middle\":"); w.writeInt((long)Middle); w.write(",");
  w.write("\"treble\":"); w.writeInt((long)Treble); w.write(",");
  w.write("\"gain\":"); w.writeInt((long)Gain); w.write(",");
  w.write("\"balance\":"); w.writeInt((long)Balance);
  w.write("}");

  w.write("}");
  w.flush();
}

void updateRpiTitleTicker(bool forceRedraw) {
  static char lastBaseTitle[96] = "";
  static char scrollBuf[200] = ""; // base + separator, doubled in logic (no heap)
  static size_t scrollPeriod = 0;  // strlen(scrollBuf) after rebuild
  static size_t scrollIdx = 0;
  static unsigned long lastTickMs = 0;
  const unsigned long TITLE_SCROLL_STEP_MS = 300UL;

  if (standbyState || fmRadio.utilMode || (fmRadio.sursa != 3)) return;

  char baseTitle[sizeof(rpiTitleBuf)];
  strncpy(baseTitle, rpiTitleBuf, sizeof(baseTitle) - 1);
  baseTitle[sizeof(baseTitle) - 1] = '\0';
  // trim in-place
  char *b = baseTitle;
  while (*b == ' ' || *b == '\t') b++;
  size_t bl = strlen(b);
  while (bl > 0 && (b[bl - 1] == ' ' || b[bl - 1] == '\t')) bl--;
  b[bl] = '\0';
  if (bl == 0) {
    strcpy(b, "-");
    bl = 1;
  }

  if (forceRedraw || strcmp(b, lastBaseTitle) != 0) {
    strncpy(lastBaseTitle, b, sizeof(lastBaseTitle) - 1);
    lastBaseTitle[sizeof(lastBaseTitle) - 1] = '\0';
    // Separator must be non-empty so scrolling shows a pause between repeats.
    snprintf(scrollBuf, sizeof(scrollBuf), "%s  ", lastBaseTitle);
    scrollPeriod = strlen(scrollBuf);
    scrollIdx = 0;
    lastTickMs = 0;
  }

  const size_t windowLen = 30;
  bool longTitle = (strlen(lastBaseTitle) > windowLen);
  unsigned long now = millis();
  if (!longTitle) {
    // Static title: redraw only when forced or when base changed (handled above).
    if (!forceRedraw) return;
  } else {
    if (!forceRedraw) {
      if (now - lastTickMs < TITLE_SCROLL_STEP_MS) return;
    }
    lastTickMs = now;
  }

  char view[40];
  memset(view, ' ', sizeof(view));
  view[windowLen] = '\0';

  if (longTitle) {
    if (scrollPeriod == 0) return;
    if (scrollIdx >= scrollPeriod) scrollIdx = 0;
    for (size_t i = 0; i < windowLen; i++) {
      view[i] = scrollBuf[(scrollIdx + i) % scrollPeriod];
    }
    scrollIdx++;
  } else {
    size_t n = strlen(lastBaseTitle);
    if (n > windowLen) n = windowLen;
    memcpy(view, lastBaseTitle, n);
    for (size_t i = n; i < windowLen; i++) view[i] = ' ';
    view[windowLen] = '\0';
  }

  // #region agent log: H5 ticker redraw spikes
  uint32_t t0 = micros();
  // #endregion
  fmRadio.tft.fillRect(0, 99, SCREEN_WIDTH, 24, ST77XX_BLACK);
  fmRadio.tft.setTextColor(ST77XX_YELLOW);
  fmRadio.tft.setTextSize(2);
  fmRadio.tft.setCursor(0, 101);
  fmRadio.tft.print(view);
  // #region agent log: H5 ticker redraw spikes
  uint32_t dt = micros() - t0;
  if (dt > profTickerUsMax) profTickerUsMax = dt;
  profTickerDraws++;
  // #endregion
}

void updateRpiPlayIndicator(bool forceRedraw) {
  // Dot in the top-right corner of the RPI source button (x: 80..159, y: 0..59).
  const int dotX = 151;
  const int dotY = 13;
  const int dotR = 6;

  static bool lastActive = false;
  static bool lastShown = false;

  // În standby nu atingem TFT-ul aici — altfel fillCircle gri la y~13 suprascrie header-ul ceasului.
  if (standbyState) {
    lastActive = false;
    lastShown = false;
    rpiBlinkOn = false;
    return;
  }

  // Keep indicator off in standby/util screens.
  bool activeScreen = !standbyState && !fmRadio.utilMode;
  bool rpiPlaying = rpiPlayingFromState(rpiStateBuf);
  bool active = activeScreen && rpiConnected && rpiPlaying;

  unsigned long now = millis();
  if (active && (forceRedraw || (now - lastRpiBlinkMs >= 500UL))) {
    rpiBlinkOn = !rpiBlinkOn;
    lastRpiBlinkMs = now;
    forceRedraw = true; // repaint only on blink edge
  }
  if (!active) {
    rpiBlinkOn = false;
  }

  bool showDot = active && rpiBlinkOn;
  if (!forceRedraw && active == lastActive && showDot == lastShown) return;

  // Restore button background where the dot lives, then draw if needed.
  fmRadio.tft.fillCircle(dotX, dotY, dotR + 1, CUSTOM_GREY);
  if (showDot) {
    fmRadio.tft.fillCircle(dotX, dotY, dotR, ST77XX_RED);
  }

  lastActive = active;
  lastShown = showDot;
}

void updateRpiTransportPlayIcon(bool forceRedraw) {
  static bool lastValid = false;
  static bool lastPlaying = false;

  bool activeScreen = !standbyState && !fmRadio.utilMode && (fmRadio.sursa == 3 || fmRadio.sursa == 4 || fmRadio.sursa == 5);
  if (!activeScreen) {
    lastValid = false;
    return;
  }

  const char *st = (fmRadio.sursa == 5) ? pcStateBuf : ((fmRadio.sursa == 4) ? pcStateBuf : rpiStateBuf);
  bool rpiIsPlaying = rpiPlayingFromState(st);
  if (!forceRedraw && lastValid && (lastPlaying == rpiIsPlaying)) return;

  // Middle transport button: orange background + icon that indicates next action.
  fmRadio.tft.fillRect(TRANSPORT_BTN1_X, TRANSPORT_ROW_Y, TRANSPORT_BTN_W, TRANSPORT_ROW_H, ST77XX_ORANGE);
  if (rpiIsPlaying) {
    fmRadio.tft.fillRect(99, 161, 6, 20, ST77XX_WHITE);
    fmRadio.tft.fillRect(111, 161, 6, 20, ST77XX_WHITE);
  } else {
    fmRadio.tft.fillTriangle(100, 161, 100, 181, 118, 171, ST77XX_WHITE);
  }

  lastPlaying = rpiIsPlaying;
  lastValid = true;
}

//------------------------------------
// Enter/Exit Standby Helpers
//------------------------------------

void enterStandby() {
  fmRadio.flushSettingsIfDue(true);
  inStandby = true;
  standbyState = true;
  standbyStartTime = rtc.now();
  standbyStartMs = millis();
  fmRadio.tft.fillScreen(ST77XX_BLACK);
  forceFullTimeRedraw = true;  // Force full redraw on next time update
  standbyFullRedrawPending = true;  // Ensure full standby layout repaint

  // Reset encoder count so movement during standby is ignored
  setEncoderCount(0);
  encoderLastEncVal = 0;

  // Mute TDA7439 + stare UI (fără updateVolumeDisplay — urmează fillScreen în showtimestandBy)
  if (!fmRadio.isMuted) {
    fmRadio.isMuted = true;
    Wire.beginTransmission(0x44);
    if (Wire.endTransmission() == 0) {
      tda7439.setVolume(0);
    }
    setEncoderCount(fmRadio.currentVolume);
    fmRadio.saveSettings();
  } else {
    Wire.beginTransmission(0x44);
    if (Wire.endTransmission() == 0) {
      tda7439.setVolume(0);
    }
  }

  // Serial.println("Entering Standby mode");
  // Immediately render standby clock/date so the screen isn't left blank
  showtimestandBy();
}

void exitStandby() {
  inStandby = false;
  standbyState = false;
  standbyFullRedrawPending = true;
  rtc.begin();
  delay(10);

  delay(500); // Wait for components to stabilize

  // Reinit I2C si initializeaza SI4703 la power ON
  Wire.begin(RADIO_SDAPIN, RADIO_SCLPIN);
  delay(100);
  reinitSI4703();

  // Now reinitialize display and touch so TFT is not left in reset state
  fmRadio.tft.init(SCREEN_HEIGHT, SCREEN_WIDTH);
  fmRadio.tft.setRotation(1);
  fmRadio.tft.fillScreen(ST77XX_BLACK);
  fmRadio.tft.invertDisplay(false);
  fmRadio.ts.begin();

  fmRadio.tft.fillScreen(ST77XX_BLACK);
  fmRadio.drawInitialScreen();
  fmRadio.updateModeDisplay();
  fmRadio.updateSelectedSourceDisplay();
  // Serial.println("Exit standby mode");
  forceFullTimeRedraw = true;  // Force full redraw on next time update

  // Set volume to fixed startup value (ieșire din standby = fără mute pe UI, aliniat cu TDA)
  fmRadio.currentVolume = 10;
  fmRadio.isMuted = false;
  setEncoderCount(fmRadio.currentVolume);
  encoderLastEncVal = fmRadio.currentVolume;
  fmRadio.updateVolumeDisplay();

  // Re-apply saved audio settings to TDA7439 after wake
  fmRadio.loadEqualizer();
  applyTDA7439SettingsForCurrentSource();

  // Reset encoder to current value so no jump occurs
  if (fmRadio.isInVolumeMode)
    setEncoderCount(fmRadio.currentVolume);
  else
    setEncoderCount(fmRadio.currentFrequency * 10);
}

//------------------------------------
// Time Display Function
//------------------------------------

void showTime() {
  if (FMRadioController::instance->utilMode) return;

  DateTime now;
  if (inStandby) {
    uint32_t elapsedSec = (millis() - standbyStartMs) / 1000;
    now = standbyStartTime + TimeSpan(elapsedSec);
    showtimestandBy();
    return;
  }

  // On Raspberry and PC (foobar) sources, do not draw time/date.
  // These sources use the mid-screen area for transport/UI and time can leave artifacts.
  if (FMRadioController::instance->sursa == 3 || FMRadioController::instance->sursa == 4 || FMRadioController::instance->sursa == 5) {
    return;
  }

  now = rtc.now();

  char timeStr[10];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  char dateStr[16];
  sprintf(dateStr, "%s %02d/%02d",
          FMRadioController::instance->days[now.dayOfTheWeek()],
          now.day(), now.month());

  static char prevTimeStr[9] = "";
  static char prevDateStr[16] = "";

  const int tx = 8, ty = 75, cw = 17, ch = 24;  // time layout
  const int dx = 180, dy = 75;                  // date position

  // On forced refresh, clear the entire time/date strip first
  // to avoid leftovers after source/layout transitions.
  if (forceFullTimeRedraw) {
    FMRadioController::instance->tft.fillRect(0, ty, SCREEN_WIDTH, ch, ST77XX_BLACK);
  }

  bool redrawDate = forceFullTimeRedraw || FMRadioController::instance->wasSetup || strcmp(prevDateStr, dateStr) != 0;

  // Update time digits only if changed (clear width +3px on last char to avoid white spill from font)
  for (int i = 0; i < 8; i++) {
    if (forceFullTimeRedraw || FMRadioController::instance->wasSetup || prevTimeStr[i] != timeStr[i]) {
      int x = tx + i * cw;
      int clearW = (i == 7) ? cw + 3 : cw;  // extra pixels right of last digit to remove white artifacts
      FMRadioController::instance->tft.fillRect(x, ty, clearW, ch, ST77XX_BLACK);
      FMRadioController::instance->tft.setCursor(x, ty);
      FMRadioController::instance->tft.setTextColor(ST77XX_YELLOW);
      FMRadioController::instance->tft.setTextSize(3);
      FMRadioController::instance->tft.print(timeStr[i]);
    }
  }

  // Update the date string if needed
  if (redrawDate) {
    FMRadioController::instance->tft.fillRect(dx, dy, 140, ch, ST77XX_BLACK);
    FMRadioController::instance->tft.setCursor(dx, dy);

    // Extract day abbreviation (first 2 chars of dateStr)
    char dayPart[4] = { 0 };
    strncpy(dayPart, dateStr, 2);
    dayPart[2] = '\0';

    FMRadioController::instance->tft.setTextColor(ST77XX_WHITE);
    FMRadioController::instance->tft.print(dayPart);  // e.g., "Vi"
    FMRadioController::instance->tft.setTextColor(ST77XX_YELLOW);
    FMRadioController::instance->tft.print(&dateStr[2]);  // e.g., " 25/04"

    strcpy(prevDateStr, dateStr);
  }


  strcpy(prevTimeStr, timeStr);
  FMRadioController::instance->wasSetup = false;
  forceFullTimeRedraw = false;
}



void showtimestandBy() {
  if (!inStandby) return;

  uint32_t elapsedSec = (millis() - standbyStartMs) / 1000;
  DateTime now = standbyStartTime + TimeSpan(elapsedSec);

  static const char* monthsShort[] = {"Ian", "Feb", "Mar", "Apr", "Mai", "Iun", "Iul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  char timeStr[6];
  sprintf(timeStr, "%02d%c%02d", now.hour(), (elapsedSec % 2 == 0) ? ':' : ' ', now.minute());
  const char* dayStr = fmRadio.daysFull[now.dayOfTheWeek()];
  char dateStr[12];
  sprintf(dateStr, "%d %s", now.day(), monthsShort[now.month() - 1]);

  static char prevTimeStr[6] = "";
  static char prevDayStr[12] = "";
  static char prevDateStr[12] = "";

  const int timeY = 52;
  const int dayDateY = 143;
  const int dayDateLineHeight = 24;
  const int timeZoneH = dayDateY - timeY;
  const int timeSlotW = 52, timeSlotH = 56;
  const int timeX0 = (SCREEN_WIDTH - 5 * timeSlotW) / 2 - 5;

  int16_t x1, y1;
  uint16_t w, h;

  bool anyTimeSlotChanged = false;
  for (int i = 0; i < 5; i++) {
    if (prevTimeStr[i] != timeStr[i]) anyTimeSlotChanged = true;
  }
  bool dayDateChanged = (strcmp(prevDayStr, dayStr) != 0 || strcmp(prevDateStr, dateStr) != 0);
  bool needFullStandbyRedraw = forceFullTimeRedraw || standbyFullRedrawPending;
  const bool standbyDidFullRedraw = needFullStandbyRedraw;
  bool needRedrawClock = needFullStandbyRedraw || anyTimeSlotChanged || dayDateChanged;

  static char prevStandbyIp[24] = "";
  static int prevStandbyRssi = 0;
  const int barY = 202, barH = 38;
  String ipStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "-";
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -999;
  char rssiBuf[16];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", rssi);
  } else {
    strcpy(rssiBuf, "N/A");
  }
  bool rssiChanged = (rssi == -999 && prevStandbyRssi != -999) || (rssi != -999 && abs(rssi - prevStandbyRssi) >= 2);
  bool needRedrawFullBar = needFullStandbyRedraw;
  if (needFullStandbyRedraw) prevStandbyIp[0] = '\0';

  if (needFullStandbyRedraw) {
    fmRadio.tft.fillScreen(ST77XX_BLACK);
    fmRadio.tft.fillRect(0, timeY, SCREEN_WIDTH, timeZoneH, ST77XX_BLACK);
    fmRadio.tft.setTextSize(8);
    fmRadio.tft.setTextColor(CUSTOM_CYAN);
    for (int i = 0; i < 5; i++) {
      int slotX = timeX0 + i * timeSlotW;
      fmRadio.tft.setCursor(slotX, timeY);
      fmRadio.tft.print(timeStr[i]);
    }
    fmRadio.tft.setTextSize(3);
    fmRadio.tft.setTextColor(ST77XX_WHITE);
    fmRadio.tft.setCursor(5, dayDateY);
    fmRadio.tft.print(dayStr);
    fmRadio.tft.setTextColor(ST77XX_BLUE);
    fmRadio.tft.getTextBounds(dateStr, 0, 0, &x1, &y1, &w, &h);
    fmRadio.tft.setCursor(SCREEN_WIDTH - w - 5, dayDateY);
    fmRadio.tft.print(dateStr);
    strcpy(prevTimeStr, timeStr);
    strcpy(prevDayStr, dayStr);
    strcpy(prevDateStr, dateStr);
  } else {
    if (anyTimeSlotChanged) {
      fmRadio.tft.setTextSize(8);
      fmRadio.tft.setTextColor(CUSTOM_CYAN);
      for (int i = 0; i < 5; i++) {
        if (prevTimeStr[i] != timeStr[i]) {
          int slotX = timeX0 + i * timeSlotW;
          fmRadio.tft.fillRect(slotX, timeY, timeSlotW, timeSlotH, ST77XX_BLACK);
          fmRadio.tft.setCursor(slotX, timeY);
          fmRadio.tft.print(timeStr[i]);
        }
      }
      fmRadio.tft.setTextSize(3);
      strcpy(prevTimeStr, timeStr);
    }
    if (dayDateChanged) {
      fmRadio.tft.fillRect(0, dayDateY, SCREEN_WIDTH, dayDateLineHeight, ST77XX_BLACK);
      fmRadio.tft.setTextSize(3);
      fmRadio.tft.setTextColor(ST77XX_WHITE);
      fmRadio.tft.setCursor(5, dayDateY);
      fmRadio.tft.print(dayStr);
      fmRadio.tft.setTextColor(ST77XX_BLUE);
      fmRadio.tft.getTextBounds(dateStr, 0, 0, &x1, &y1, &w, &h);
      fmRadio.tft.setCursor(SCREEN_WIDTH - w - 5, dayDateY);
      fmRadio.tft.print(dateStr);
      strcpy(prevDayStr, dayStr);
      strcpy(prevDateStr, dateStr);
    }
  }

  if (needFullStandbyRedraw) {
    forceFullTimeRedraw = false;
    standbyFullRedrawPending = false;
  }

  if (needRedrawFullBar) {
    if (prevStandbyIp[0] == '\0') {
      strncpy(prevStandbyIp, ipStr.c_str(), sizeof(prevStandbyIp) - 1);
      prevStandbyIp[sizeof(prevStandbyIp) - 1] = '\0';
    }
    fmRadio.tft.fillRect(0, barY, SCREEN_WIDTH, barH, ST77XX_BLACK);
    fmRadio.tft.setTextSize(2);
    fmRadio.tft.setTextColor(CUSTOM_LIGHTGREY);
    fmRadio.tft.setCursor(5, barY + 4);
    fmRadio.tft.print(prevStandbyIp);
    fmRadio.tft.getTextBounds(rssiBuf, 0, 0, &x1, &y1, &w, &h);
    fmRadio.tft.setCursor(SCREEN_WIDTH - w - 5, barY + 4);
    fmRadio.tft.print(rssiBuf);
    prevStandbyRssi = rssi;
  } else if (rssiChanged) {
    const int rssiZoneW = 90;
    fmRadio.tft.fillRect(SCREEN_WIDTH - rssiZoneW, barY, rssiZoneW, barH, ST77XX_BLACK);
    fmRadio.tft.setTextSize(2);
    fmRadio.tft.setTextColor(CUSTOM_LIGHTGREY);
    fmRadio.tft.getTextBounds(rssiBuf, 0, 0, &x1, &y1, &w, &h);
    fmRadio.tft.setCursor(SCREEN_WIDTH - w - 5, barY + 4);
    fmRadio.tft.print(rssiBuf);
    prevStandbyRssi = rssi;
  }

  // RPI / Moode: în play → punct verde care alternează la ~1 s; altfel stare statică
  {
    const int rpiBandY = 0;
    const int rpiBandH = 40;
    const int dotCx = 16;
    const int dotCy = 18;
    const int dotR = 7;
    bool playing = rpiConnected && rpiPlayingFromState(rpiStateBuf);
    unsigned long tsec = millis() / 1000UL;
    bool flashBright = ((tsec % 2UL) == 0UL);

    static unsigned long lastStandbyRpiDrawSec = 0xFFFFFFFFUL;
    static bool lastStandbyRpiConn = false;
    static bool lastStandbyRpiPlay = false;

    bool conn = rpiConnected;
    bool needRpiBand = standbyDidFullRedraw
        || (conn != lastStandbyRpiConn)
        || (playing != lastStandbyRpiPlay)
        || (playing && (tsec != lastStandbyRpiDrawSec));

    if (needRpiBand) {
      if (playing) {
        lastStandbyRpiDrawSec = tsec;
      } else {
        lastStandbyRpiDrawSec = 0xFFFFFFFFUL;
      }
      lastStandbyRpiConn = conn;
      lastStandbyRpiPlay = playing;

      fmRadio.tft.fillRect(0, rpiBandY, SCREEN_WIDTH, rpiBandH, ST77XX_BLACK);
      uint16_t dotCol;
      if (!conn) {
        dotCol = CUSTOM_GREY;
      } else if (playing) {
        dotCol = flashBright ? ST77XX_GREEN : CUSTOM_DARKGREEN;  // ~1 s alternanță
      } else {
        dotCol = ST77XX_CYAN;
      }
      fmRadio.tft.fillCircle(dotCx, dotCy, dotR, dotCol);
      fmRadio.tft.setTextSize(2);
      fmRadio.tft.setTextColor(ST77XX_WHITE);
      fmRadio.tft.setCursor(32, 8);
      fmRadio.tft.print("RPI");
      fmRadio.tft.setTextSize(1);
      fmRadio.tft.setTextColor(CUSTOM_LIGHTGREY);
      fmRadio.tft.setCursor(92, 14);
      if (!conn) {
        fmRadio.tft.print("Moode: --");
      } else if (playing) {
        fmRadio.tft.setTextColor(flashBright ? ST77XX_GREEN : CUSTOM_GREY);
        fmRadio.tft.print("Moode: PLAY");
      } else {
        fmRadio.tft.print("Moode: ");
        fmRadio.tft.print(rpiStateBuf);
      }
    }
  }
}

//------------------------------------
// Touch Handling Function
//------------------------------------
void checkTouch() {
  static bool rpiTouchLatched = false;
  static unsigned long lastTouchDbgMs = 0;
  if (!FMRadioController::instance->ts.touched()) {
    rpiTouchLatched = false;
    return;
  }
  if (FMRadioController::instance->ts.touched()) {
    TS_Point p = FMRadioController::instance->ts.getPoint();
    int mappedX, mappedY;
    FMRadioController::instance->mapTouch(p.x, p.y, mappedX, mappedY);
    // Hit-test coordinate (0,0 = top-left).
    const int ux = mappedX;

  // Touch debug: print raw + mapped coordinates (throttled).
  // Enable by setting TOUCH_DEBUG to 1 (and DEBUG_SERIAL_LOGS to 1).
#ifndef TOUCH_DEBUG
#define TOUCH_DEBUG 0
#endif
#if TOUCH_DEBUG && DEBUG_SERIAL_LOGS
    if (millis() - lastTouchDbgMs > 120) {
      lastTouchDbgMs = millis();
      Serial.print("[TOUCH] raw=(");
      Serial.print(p.x);
      Serial.print(",");
      Serial.print(p.y);
      Serial.print(") z=");
      Serial.print(p.z);
      Serial.print(" mapped=(");
      Serial.print(mappedX);
      Serial.print(",");
      Serial.print(mappedY);
      Serial.print(") sursa=");
      Serial.println(FMRadioController::instance->sursa);
    }
#endif

    if (p.z > 10) {
      if (FMRadioController::instance->utilMode) lastUtilActivity = millis();
      // Util-mode buttons are drawn at the bottom:
      // - RTC: bottom-left (0,200..60x40)
      // - Back: bottom-right (SETUP_BUTTON_*)
      if (FMRadioController::instance->utilMode) {
        // RTC button (bottom-left)
        if (mappedX >= 0 && mappedX <= 60 && mappedY >= 200 && mappedY <= (SCREEN_HEIGHT - 1)) {
          FMRadioController::instance->rtcUpdateSuccess = syncRTCWithNTPRetries();
          FMRadioController::instance->rtcUpdateMsgMillis = millis();
          FMRadioController::instance->drawUtilButton();
          drawsetScreen();
          return;
        }
        // Back button (bottom-right)
        if (mappedX >= SETUP_BUTTON_X_MIN && mappedX <= (SETUP_BUTTON_X_MIN + SETUP_BUTTON_WIDTH - 1) &&
            mappedY >= SETUP_BUTTON_Y_MIN && mappedY <= (SETUP_BUTTON_Y_MIN + SETUP_BUTTON_HEIGHT - 1)) {
          static unsigned long lastUtil = 0;
          if (millis() - lastUtil < 700) return;  // debounce = 700 ms ca in toggleUtilMode
          lastUtil = millis();
          FMRadioController::instance->utilMode = false;
          FMRadioController::instance->toggleUtilMode();
          return;
        }
      } else {
        // Main screen: Util button (bottom-right)
        if (mappedX >= SETUP_BUTTON_X_MIN && mappedX <= (SETUP_BUTTON_X_MIN + SETUP_BUTTON_WIDTH - 1) &&
            mappedY >= SETUP_BUTTON_Y_MIN && mappedY <= (SETUP_BUTTON_Y_MIN + SETUP_BUTTON_HEIGHT - 1)) {
          static unsigned long lastUtil = 0;
          if (millis() - lastUtil < 700) return;  // debounce = 700 ms ca in toggleUtilMode
          lastUtil = millis();
          FMRadioController::instance->utilMode = true;
          FMRadioController::instance->toggleUtilMode();
          return;
        }
      }
      // In util (setup) mode, change sursa by touching the lower part.
      if (mappedY > 180 && FMRadioController::instance->utilMode) {
        int newSursa = 0;
        // Treat PCD as a real source here too (5 segments, same order as top bar).
        const int idx = constrain(ux / 64, 0, 4);
        static const int kSourceByIndex[5] = { 4, 5, 3, 2, 1 };
        newSursa = kSourceByIndex[idx];
        if (newSursa != 0 && newSursa != FMRadioController::instance->sursa) {
          // Save current settings before switching.
          FMRadioController::instance->saveSettings();
          // Update sursa.
          FMRadioController::instance->sursa = newSursa;
          // Keep MCP source logic from overwriting the touch-selected source.
          FMRadioController::instance->sursaVeche = FMRadioController::instance->sursa;
          FMRadioController::instance->lastSourceChangeMs = millis();
          // Load and apply EQ/gain/input/volume for the new source
          FMRadioController::instance->loadEqualizer();
          applyTDA7439SettingsForCurrentSource();
          // Instead of redrawing the full screen, update only the EQ values.
          FMRadioController::instance->updateUtilSursaChange();
          FMRadioController::instance->updateSelectedSourceDisplay();
          // drawsetScreen();
          FMRadioController::instance->saveSettings();
          lastUtilActivity = millis();
        }
      }
      // Process non-util touches.
      if (!FMRadioController::instance->utilMode) {
        // Top source bar: 5 buttons (PCA, PCD, RPI, BT, TUN) across 320px => 64px each.
        if (mappedY >= 0 && mappedY <= 60) {
          int newSursa = 0;
          const int idx = constrain(ux / 64, 0, 4);
          // Display order: PCA, PCD, RPI, BT, TUN
          static const int kSourceByIndex[5] = { 4, 5, 3, 2, 1 };
          newSursa = kSourceByIndex[idx];
          if (newSursa != 0 && newSursa != FMRadioController::instance->sursa) {
            FMRadioController::instance->sursa = newSursa;
            FMRadioController::instance->sursaVeche = FMRadioController::instance->sursa;
            FMRadioController::instance->lastSourceChangeMs = millis();
            // PCA shows "PC analogic" overlay; PCD shows Foobar metadata.
            if (newSursa == 4) pcAnalogUi = true;
            else if (newSursa == 5) pcAnalogUi = false;
            FMRadioController::instance->loadEqualizer();
            applyTDA7439SettingsForCurrentSource();
            FMRadioController::instance->updateSelectedSourceDisplay();
            FMRadioController::instance->updateMainContent();
            FMRadioController::instance->saveSettings();
          }
          return;
        }
        // Transport row band (Prev/Play/Next) - should match draw positions.
        const bool rpiButtonBand =
          (mappedY >= (TRANSPORT_ROW_Y - 5)) &&
          (mappedY <= (TRANSPORT_ROW_Y + TRANSPORT_ROW_H + 5));
        // RPI + PC transport: aceleași dreptunghiuri; Prev/Play/Next = aceeași mapare tx ca la RPi
        // IMPORTANT: toate hitbox-urile sunt în coordonata tx (mappedX + offset),
        // altfel PCA/PCD și Prev/Play/Next se decalibrează relativ unul față de altul.
        if ((FMRadioController::instance->sursa == 3 || FMRadioController::instance->sursa == 5) && rpiButtonBand) {
          int tx = constrain(mappedX + RPI_TOUCH_X_OFFSET, 0, SCREEN_WIDTH - 1);
          int tbtn = -1;
          if (tx <= TRANSPORT_TX_MAX0) tbtn = 0;
          else if (tx <= TRANSPORT_TX_MAX1) tbtn = 1;
          else if (tx <= TRANSPORT_TX_MAX2) tbtn = 2;

          if (tbtn >= 0) {
            if (!rpiTouchLatched) {
              rpiTouchLatched = true;
              if (FMRadioController::instance->sursa == 3) {
                // RPi: Moode controls
                if (tbtn == 0) sendRpiCommand("NEXT"); // swap PREV/NEXT to match on-screen behavior
                else if (tbtn == 1) sendRpiCommand("PLAYPAUSE");
                else sendRpiCommand("PREV");
              } else { // PCA/PCD: foobar controls
                // Foobar: keep natural mapping (left=PREV, right=NEXT).
                if (tbtn == 0) sendPcCommand("PREV");
                else if (tbtn == 1) sendPcCommand("PLAYPAUSE");
                else sendPcCommand("NEXT");
              }
            }
          } else {
            // Allow immediate retrigger if finger slides back over a button.
            rpiTouchLatched = false;
          }
          return;
        }
        if (mappedX >= SEEK_UP_X_MIN && mappedX <= SEEK_UP_X_MAX && mappedY >= SEEK_UP_Y_MIN && mappedY <= SEEK_UP_Y_MAX) {
          FMRadioController::instance->seekUp();
          FMRadioController::instance->updateMainContent();
          FMRadioController::instance->saveSettings();
        }
        if (mappedX >= SEEK_DN_X_MIN && mappedX <= SEEK_DN_X_MAX && mappedY >= SEEK_DN_Y_MIN && mappedY <= SEEK_DN_Y_MAX) {
          FMRadioController::instance->seekDown();
          FMRadioController::instance->updateMainContent();
          FMRadioController::instance->saveSettings();
        } else if (mappedY > 180) {
          int newSursa = 0;
          // Bottom quick source select: 5 segments, same order as top bar.
          const int idx = constrain(ux / 64, 0, 4);
          static const int kSourceByIndex[5] = { 4, 5, 3, 2, 1 };
          newSursa = kSourceByIndex[idx];
          if (newSursa != 0 && newSursa != FMRadioController::instance->sursa) {
            FMRadioController::instance->sursa = newSursa;
            FMRadioController::instance->sursaVeche = FMRadioController::instance->sursa;
            FMRadioController::instance->lastSourceChangeMs = millis();
            // Load and apply EQ/gain/input/volume for the new source
            FMRadioController::instance->loadEqualizer();
            applyTDA7439SettingsForCurrentSource();
            FMRadioController::instance->updateSelectedSourceDisplay();
            FMRadioController::instance->updateMainContent();
            FMRadioController::instance->saveSettings();
          }
        }
      }
    }

    auto &tft = fmRadio.tft;
    if (FMRadioController::instance->utilMode) {
      // Debounce: un singur pas la fiecare 350 ms ca nu merge direct la min/max
      static unsigned long lastEqTouchMs = 0;
      const unsigned long EQ_TOUCH_COOLDOWN_MS = 350;
      auto adjustSetting = [&](int &setting, int delta, int minVal, int maxVal, int cursorX, int cursorY) {
        if (millis() - lastEqTouchMs < EQ_TOUCH_COOLDOWN_MS) return;
        lastEqTouchMs = millis();
        if ((delta > 0 && setting >= maxVal) || (delta < 0 && setting <= minVal)) {
          tft.setCursor(cursorX, cursorY);
          tft.fillRect(cursorX, cursorY, 60, 20, ST77XX_BLACK);
          tft.setTextColor(ST77XX_RED);
          char buf[5];
          sprintf(buf, "%3d", setting);
          tft.print(buf);
          return;
        }
        setting += delta;
        tft.setCursor(cursorX, cursorY);
        tft.fillRect(cursorX, cursorY, 60, 20, ST77XX_BLACK);
        if (setting == maxVal || setting == minVal)
          tft.setTextColor(ST77XX_RED);
        else
          tft.setTextColor(ST77XX_WHITE);
        char buf[5];
        sprintf(buf, "%3d", setting);
        tft.print(buf);
        // Actualizare TDA7439 la fiecare parametru
        if (&setting == &Bass) tda7439.setSnd(Bass, 1);
        else if (&setting == &Middle) tda7439.setSnd(Middle, 2);
        else if (&setting == &Treble) tda7439.setSnd(Treble, 3);
        else if (&setting == &Gain) tda7439.inputGain((Gain + 45) / 3);
        lastUtilActivity = millis();
      };

      // UTIL EQ +/-: same column order and Y bands as drawsetScreen() (Bass..Gain left→right).
      if (mappedX >= 0 && mappedX <= 79) {
        if (mappedY >= UTIL_EQ_PLUS_Y_MIN && mappedY <= UTIL_EQ_PLUS_Y_MAX)
          adjustSetting(Bass, 1, -7, 7, 14, 134);
        else if (mappedY >= UTIL_EQ_MINUS_Y_MIN && mappedY <= UTIL_EQ_MINUS_Y_MAX)
          adjustSetting(Bass, -1, -7, 7, 14, 134);
      } else if (mappedX >= 80 && mappedX <= 159) {
        if (mappedY >= UTIL_EQ_PLUS_Y_MIN && mappedY <= UTIL_EQ_PLUS_Y_MAX)
          adjustSetting(Middle, 1, -7, 7, 94, 134);
        else if (mappedY >= UTIL_EQ_MINUS_Y_MIN && mappedY <= UTIL_EQ_MINUS_Y_MAX)
          adjustSetting(Middle, -1, -7, 7, 94, 134);
      } else if (mappedX >= 160 && mappedX <= 239) {
        if (mappedY >= UTIL_EQ_PLUS_Y_MIN && mappedY <= UTIL_EQ_PLUS_Y_MAX)
          adjustSetting(Treble, 1, -7, 7, 174, 134);
        else if (mappedY >= UTIL_EQ_MINUS_Y_MIN && mappedY <= UTIL_EQ_MINUS_Y_MAX)
          adjustSetting(Treble, -1, -7, 7, 174, 134);
      } else if (mappedX >= 240 && mappedX <= (SCREEN_WIDTH - 1)) {
        if (mappedY >= UTIL_EQ_PLUS_Y_MIN && mappedY <= UTIL_EQ_PLUS_Y_MAX)
          adjustSetting(Gain, 1, -45, 0, 259, 134);
        else if (mappedY >= UTIL_EQ_MINUS_Y_MIN && mappedY <= UTIL_EQ_MINUS_Y_MAX)
          adjustSetting(Gain, -1, -45, 0, 259, 134);
      }
    }
  }
}

//------------------------------------
// Utility Screen Draw
//------------------------------------
void drawsetScreen() {
  fmRadio.drawsetScreen();
  showTime();  // redraw time/date on top
}


// --------------------------------------------------------------------------------------------------
// TIME
// --------------------------------------------------------------------------------------------------

// WiFi credentials (replace with your actual credentials)
const char *ssid = "Turris-Slow";
const char *password = "quickprint";
const unsigned long NTP_SYNC_INTERVAL_MS = 3600000UL; // 1 hour
const uint8_t NTP_SYNC_MAX_RETRIES = 3;
const unsigned long NTP_RETRY_DELAY_MS = 1000UL;
static unsigned long nextNtpSyncDueMs = 0; // first cycle at boot, then hourly

#if defined(ARDUINO_ARCH_ESP32)
static TaskHandle_t ntpSyncTaskHandle = nullptr;
static volatile bool ntpSyncInProgress = false;
static volatile bool ntpSyncRequested = false;

static void ntpSyncTask(void *param) {
  (void)param;
  for (;;) {
    // Wait until loop() schedules a sync
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ntpSyncInProgress = true;
    ntpSyncRequested = false;
    // Serial.println("[NTP] Background sync task running...");
    syncRTCWithNTPRetries();
    ntpSyncInProgress = false;
  }
}
#endif

//------------------------------------
// WiFi + NTP Sync Function
//------------------------------------
void connectToWiFi() {
  // Start WiFi in Station mode
  WiFi.mode(WIFI_STA);
  // Reduce latency caused by WiFi modem sleep
  WiFi.setSleep(false);
  WiFi.setHostname("flo-amp");
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  Serial.print("SSID: ");
  Serial.println(ssid);

  // Wait up to 15 seconds (tighter poll so Web UI / setup can continue sooner when AP is quick).
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    Serial.print(".");
    delay(120);
    yield();
  }

  // Check result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\n❌ Failed to connect.");
    Serial.println("Please check:");
    Serial.println("1. WiFi credentials are correct");
    Serial.println("2. ESP32 is in range of the WiFi network");
    Serial.println("3. WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)");
  }
}

// Dupa multe ore / standby, AP-ul poate deconecta clientul; fara reconectare ramane fara IP si Web UI + mDNS nu raspund.
void maintainWiFiConnection() {
  static wl_status_t prevWifi = WL_IDLE_STATUS;
  static unsigned long lastReconnectMs = 0;
  const wl_status_t st = WiFi.status();

  if (st == WL_CONNECTED) {
    if (prevWifi != WL_CONNECTED && prevWifi != WL_IDLE_STATUS && prevWifi != WL_NO_SHIELD) {
#if defined(ARDUINO_ARCH_ESP32)
      MDNS.end();
      delay(30);
      if (MDNS.begin("flo-amp")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("mDNS OK (dupa reconectare): http://flo-amp.local");
      } else {
        Serial.println("mDNS esuat dupa reconectare — foloseste IP-ul");
      }
#endif
      Serial.print("Web UI:  http://");
      Serial.println(WiFi.localIP());
    }
    prevWifi = st;
    return;
  }

  if (prevWifi == WL_CONNECTED) {
    Serial.println("[WiFi] legatura pierduta — se incearca reconectarea periodica");
  }
  prevWifi = st;

  const unsigned long now = millis();
  if (now - lastReconnectMs < 20000UL) return;
  lastReconnectMs = now;

  WiFi.disconnect(false);
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("flo-amp");
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("[WiFi] WiFi.begin() (mentenanta)");
}


bool syncRTCWithNTP() {
  // Romania timezone with automatic DST (EET/EEST)
  configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.nist.gov");

  // Serial.print("Waiting for NTP time sync");
  const unsigned long ntpTimeoutMs = 12000;  // max 12 s ca setup-ul sa continue si web/mDNS sa porneasca
  unsigned long startMs = millis();
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2 && (millis() - startMs) < ntpTimeoutMs) {
    delay(500);
    // Serial.print(".");
    nowSecs = time(nullptr);
  }
  if ((millis() - startMs) >= ntpTimeoutMs) {
    // Serial.println(" timeout (continuing without NTP).");
    return false;
  }
  // Serial.println(" done.");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    // Serial.println("Failed to obtain time");
    return false;
  }

  // getLocalTime() returns local time according to TZ rules above.
  rtc.adjust(DateTime(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec));

  // Serial.println("RTC updated with NTP time.");
  return true;
}

bool syncRTCWithNTPRetries() {
  if (WiFi.status() != WL_CONNECTED) {
    // Serial.println("[NTP] WiFi not connected, skipping sync cycle.");
    return false;
  }

  for (uint8_t attempt = 1; attempt <= NTP_SYNC_MAX_RETRIES; attempt++) {
    // Serial.print("[NTP] Sync attempt ");
    // Serial.print(attempt);
    // Serial.print("/");
    // Serial.println(NTP_SYNC_MAX_RETRIES);

    if (syncRTCWithNTP()) {
      // Serial.println("[NTP] Sync successful.");
      return true;
    }

    if (attempt < NTP_SYNC_MAX_RETRIES) {
      delay(NTP_RETRY_DELAY_MS);
    }
  }

  // Serial.println("[NTP] 3 failed attempts. Keeping RTC time until next scheduled sync.");
  return false;
}

static bool serial2TryWriteBytes(const uint8_t *data, size_t len) {
  if (!len) return true;
  if (Serial2.availableForWrite() < (int)len) return false;
  return Serial2.write(data, len) == len;
}

static bool rpiUartEnabled = false;
bool rpiBridgeArmed = false;

static bool serial2TryPrint(const char *s) {
  if (!rpiUartEnabled) return false;
  if (!s) return true;
  size_t len = strlen(s);
  return serial2TryWriteBytes((const uint8_t *)s, len);
}

static inline bool isRpiContextActive() {
  // Keep all other sources fully independent from RPi boot/noise.
  // Enable UART only when the active source is actually RPI (not standby, not PCD).
  // NOTE: `rpiBridgeArmed` was previously required to avoid UART noise, but it can leave RPI "offline"
  // after reboot or certain source transitions. For reliability, keep UART active whenever source=RPI.
  return (!standbyState) && (fmRadio.sursa == 3) && (!pcdMode);
}

void initRpiSerial() {
#if defined(ARDUINO_ARCH_ESP32)
  Serial2.setTxBufferSize(2048);
  Serial2.setRxBufferSize(2048);
#endif
  Serial2.begin(RPI_SERIAL_BAUD, SERIAL_8N1, RPI_SERIAL_RX_PIN, RPI_SERIAL_TX_PIN);
  rpiRxLen = 0;
  // Treat "no STAT yet" as quiet since boot — avoids `(millis()-0) >= 1800` looking like infinite RX silence.
  lastRpiRxMs = millis();
  Serial.printf("[RPI] Serial bridge initialized RX=%d TX=%d @%d\n", RPI_SERIAL_RX_PIN, RPI_SERIAL_TX_PIN, RPI_SERIAL_BAUD);
}

static void setRpiUartEnabled(bool enabled) {
  if (enabled == rpiUartEnabled) return;
  rpiUartEnabled = enabled;
  if (enabled) {
    initRpiSerial();
  } else {
    // Fully stop UART interrupts when RPi is irrelevant (prevents boot-noise flooding WiFi/WebUI).
    Serial2.end();
    pinMode(RPI_SERIAL_RX_PIN, INPUT_PULLUP);
    rpiRxLen = 0;
    rpiForceGetAtMs = 0;
    rpiWebCmdNextTryMs = 0;
    rpiWebCmdPending = false;
    rpiConnected = false;
  }
}

bool sendRpiCommand(const char *cmd) {
  unsigned long now = millis();
  if (!cmd) return true;
  if (!rpiUartEnabled) return false;
  // Guard against accidental command floods (especially PLAYPAUSE).
  if (streqLower(cmd, "PLAYPAUSE") && (now - lastRpiCmdTxMs) < 450UL) return true;
  if (lastRpiCmdTxBuf[0] && streqLower(cmd, lastRpiCmdTxBuf) && (now - lastRpiCmdTxMs) < 200UL) return true;

  char line[80];
  int n = snprintf(line, sizeof(line), "CMD|%s\n", cmd);
  if (n <= 0 || n >= (int)sizeof(line)) return true;
  if (Serial2.availableForWrite() < n) return false;

  strncpy(lastRpiCmdTxBuf, cmd, sizeof(lastRpiCmdTxBuf) - 1);
  lastRpiCmdTxBuf[sizeof(lastRpiCmdTxBuf) - 1] = '\0';
  lastRpiCmdTxMs = now;
  Serial2.write((const uint8_t *)line, (size_t)n);

  // Moode/MPD metadata can lag slightly after transport commands.
  // Schedule a short follow-up GET so the UI gets fresh title/artist after PREV/NEXT/PLAYPAUSE.
  if (streqLower(cmd, "PREV") || streqLower(cmd, "NEXT") || streqLower(cmd, "PLAYPAUSE")) {
    rpiForceGetAtMs = now + 220UL;
  }
  return true;
}

bool sendPcCommand(const char *cmd) {
  if (!cmd || !cmd[0]) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!PC_BRIDGE_HOST || !PC_BRIDGE_HOST[0]) return false;

  HTTPClient http;
  String url;
  url.reserve(96);
  url = "http://";
  url += PC_BRIDGE_HOST;
  url += ":";
  url += String(PC_BRIDGE_PORT);
  url += "/cmd?c=";
  url += cmd;

  http.setTimeout(PC_BRIDGE_TIMEOUT_MS);
  if (!http.begin(url)) return false;
  int code = http.GET();
  http.end();
  bool ok = (code >= 200 && code < 300);
  // After transport commands, foobar/beefweb may report new metadata with a short delay.
  // Schedule a couple of fast /state polls so UI doesn't lag one track behind.
  if (ok) {
    if (streqLower(cmd, "NEXT") || streqLower(cmd, "PREV") || streqLower(cmd, "PLAYPAUSE") || streqLower(cmd, "PLAY") || streqLower(cmd, "PAUSE")) {
      unsigned long now = millis();
      pcForcePollAtMs1 = now + 150UL;
      pcForcePollAtMs2 = now + 650UL;
    }
  }
  return ok;
}

void setPcAnalogUi(bool enabled, bool persistPrefs) {
  if (standbyState || fmRadio.utilMode || fmRadio.sursa != 4) return;

  pcAnalogUi = enabled;

  if (pcAnalogUi) {
    // Clear the "now playing" region and show a simple analog path label.
    fmRadio.tft.fillRect(0, 60, SCREEN_WIDTH, 89, ST77XX_BLACK);
    fmRadio.tft.setTextSize(3);
    fmRadio.tft.setTextColor(ST77XX_WHITE);
    const char *msg = "PC analogic";
    int16_t x1, y1;
    uint16_t w, h;
    fmRadio.tft.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    const int cx = (SCREEN_WIDTH - (int)w) / 2;
    const int cy = 60 + (89 - (int)h) / 2;
    fmRadio.tft.setCursor(cx, cy);
    fmRadio.tft.print(msg);
  } else {
    updatePcNowPlayingUi(true);
  }

  updateRpiTransportPlayIcon(true);

  if (persistPrefs) {
    fmRadio.saveSettings();
  }
}

static inline const char *skipSpaces(const char *p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

static void normalizePlayerState(char *st, size_t stSize) {
  (void)stSize;
  if (!st || !st[0]) return;
  // Normalize common variants so UI logic is consistent across bridges.
  if (streqLower(st, "paused")) strcpy(st, "pause");
  else if (streqLower(st, "stopped")) strcpy(st, "stop");
  else if (streqLower(st, "playing")) strcpy(st, "play");
}

static void copyTrim(char *dst, size_t dstSize, const char *start, const char *end) {
  if (dstSize == 0) return;
  start = skipSpaces(start);
  while (end > start && (end[-1] == ' ' || end[-1] == '\t')) end--;
  size_t n = (size_t)(end - start);
  if (n >= dstSize) n = dstSize - 1;
  memcpy(dst, start, n);
  dst[n] = '\0';
}

void updatePcNowPlayingUi(bool forceRedraw) {
  if (standbyState || fmRadio.utilMode) return;
  if (!(fmRadio.sursa == 4 || fmRadio.sursa == 5)) return;
  if (fmRadio.sursa == 4 && pcAnalogUi) return;

  // Status row (same layout anchors as RPi screen)
  fmRadio.tft.setTextSize(1);
  fmRadio.tft.fillRect(0, 77, SCREEN_WIDTH, 10, ST77XX_BLACK);
  fmRadio.tft.setTextColor(ST77XX_CYAN);
  fmRadio.tft.setCursor(0, 77);
  fmRadio.tft.print(fmRadio.sursa == 4 ? "PCA:" : "PCD:");
  fmRadio.tft.print(pcBridgeOk ? " ONLINE " : " OFFLINE ");
  fmRadio.tft.print(" ");
  fmRadio.tft.print(pcStateBuf);

  // File / format line uses full width (below the status row).
  fmRadio.tft.fillRect(0, 88, SCREEN_WIDTH, 10, ST77XX_BLACK);
  fmRadio.tft.setTextSize(1);
  fmRadio.tft.setTextColor(ST77XX_WHITE);
  fmRadio.tft.setCursor(0, 90);
  {
    char fileLine[48];
    fmtShorten(pcExtraBuf, 40, fileLine, sizeof(fileLine));
    fmRadio.tft.print(fileLine);
  }

  // Title ticker uses rpiTitleBuf path; mirror foobar title into rpiTitleBuf while on PC source.
  strncpy(rpiTitleBuf, pcTitleBuf, sizeof(rpiTitleBuf) - 1);
  rpiTitleBuf[sizeof(rpiTitleBuf) - 1] = '\0';
  updateRpiTitleTicker(forceRedraw);

  // Artist line (same band as RPi)
  fmRadio.tft.fillRect(0, 123, SCREEN_WIDTH, 20, ST77XX_BLACK);
  fmRadio.tft.setTextSize(2);
  fmRadio.tft.setTextColor(ST77XX_WHITE);
  fmRadio.tft.setCursor(0, 125);
  {
    char artistDisp[40];
    fmtShorten(pcArtistBuf, 30, artistDisp, sizeof(artistDisp));
    fmRadio.tft.print(artistDisp);
  }
}

void processPcStatLine(const char *line) {
  if (!line) return;
  if (strncmp(line, "STAT|", 5) != 0) return;

  const char *p = line + 5;
  const char *s1 = strchr(p, '|');
  if (!s1) return;
  const char *s2 = strchr(s1 + 1, '|');
  if (!s2) return;
  const char *s3 = strchr(s2 + 1, '|');
  if (!s3) return;

  char newState[16], newTitle[96], newArtist[96], newExtra[96];
  copyTrim(newState, sizeof(newState), p, s1);
  copyTrim(newTitle, sizeof(newTitle), s1 + 1, s2);
  copyTrim(newArtist, sizeof(newArtist), s2 + 1, s3);
  copyTrim(newExtra, sizeof(newExtra), s3 + 1, s3 + 1 + strlen(s3 + 1));

  if (newTitle[0] == '\0') strcpy(newTitle, "-");
  if (newArtist[0] == '\0') strcpy(newArtist, "-");
  if (newExtra[0] == '\0') strcpy(newExtra, "-");
  if (newState[0] == '\0') strcpy(newState, "unknown");
  normalizePlayerState(newState, sizeof(newState));

  bool changed = (strcmp(newState, pcStateBuf) != 0) || (strcmp(newTitle, pcTitleBuf) != 0) || (strcmp(newArtist, pcArtistBuf) != 0) || (strcmp(newExtra, pcExtraBuf) != 0);

  strncpy(pcStateBuf, newState, sizeof(pcStateBuf) - 1);
  pcStateBuf[sizeof(pcStateBuf) - 1] = '\0';
  strncpy(pcTitleBuf, newTitle, sizeof(pcTitleBuf) - 1);
  pcTitleBuf[sizeof(pcTitleBuf) - 1] = '\0';
  strncpy(pcArtistBuf, newArtist, sizeof(pcArtistBuf) - 1);
  pcArtistBuf[sizeof(pcArtistBuf) - 1] = '\0';
  strncpy(pcExtraBuf, newExtra, sizeof(pcExtraBuf) - 1);
  pcExtraBuf[sizeof(pcExtraBuf) - 1] = '\0';

  // NOTE: "offline" here refers to Beefweb/player state, not ESP->PC bridge connectivity.
  // Bridge connectivity is tracked via lastPcHttpCode in pollPcBridge().
  lastPcRxMs = millis();

  if (changed && !standbyState && !fmRadio.utilMode && (fmRadio.sursa == 4 || fmRadio.sursa == 5 || fmRadio.sursa == 3)) {
    if (!pcAnalogUi) {
      updatePcNowPlayingUi(true);
    }
    updateRpiTransportPlayIcon(true);
  }
}

void pollPcBridge() {
  // Foobar bridge active on PCA (source 4) and on PCD mode (source 3 + pcdMode).
  // Do NOT skip while utilMode: same rationale as pcBridgeTask() — otherwise /state never
  // updates, WebUI sticks on Offline, and transport metadata goes stale until UTIL exits.
  if (standbyState) return;
  if (!(fmRadio.sursa == 4 || fmRadio.sursa == 5)) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!PC_BRIDGE_HOST || !PC_BRIDGE_HOST[0]) return;

#if defined(ARDUINO_ARCH_ESP32)
  // When ENABLE_PC_BRIDGE_POLLING is 1, pcBridgeTask() owns /state polling (non-blocking).
  // When it is 0, fall through and poll from loop() so WebUI/metadata still update (at cost of occasional HTTP blocking).
  if (ENABLE_PC_BRIDGE_POLLING) return;
#endif

  unsigned long now = millis();
  // When the PC bridge is offline/unreachable, frequent blocking HTTP GETs can starve WiFi/WebUI/audio.
  // Use a simple backoff to reduce load while offline; keep faster polls when online.
  static uint8_t pcFailStreak = 0;
  static unsigned long pcNextPollDueMs = 0;
  const unsigned long onlinePollMinMs = 900UL;
  const unsigned long offlinePollMinMs = 6000UL;
  if ((long)(now - pcNextPollDueMs) < 0) return;

  bool forced = false;
  if (pcForcePollAtMs1 && (long)(now - pcForcePollAtMs1) >= 0) {
    forced = true;
    pcForcePollAtMs1 = 0;
  } else if (pcForcePollAtMs2 && (long)(now - pcForcePollAtMs2) >= 0) {
    forced = true;
    pcForcePollAtMs2 = 0;
  }
  // Rate-limit: allow forced polls, otherwise use different cadence based on last known bridge status.
  const unsigned long minPoll = pcBridgeOk ? onlinePollMinMs : offlinePollMinMs;
  if (!forced && (now - lastPcPollMs < minPoll)) return;
  lastPcPollMs = now;

  HTTPClient http;
  String url;
  url.reserve(96);
  url = "http://";
  url += PC_BRIDGE_HOST;
  url += ":";
  url += String(PC_BRIDGE_PORT);
  url += "/state";

  http.setTimeout(PC_BRIDGE_TIMEOUT_MS);
  if (!http.begin(url)) {
    pcBridgeOk = false;
    lastPcHttpCode = 0;
    if (pcFailStreak < 20) pcFailStreak++;
    // Backoff grows up to ~60s while offline.
    pcNextPollDueMs = now + min(60000UL, (unsigned long)offlinePollMinMs * (unsigned long)pcFailStreak);
    // Serial spam disabled (PC bridge polling diagnostics).
    return;
  }

  uint32_t t0 = millis();
  int code = http.GET();
  uint32_t dt = millis() - t0;
  lastPcHttpCode = code;
  if (code < 200 || code >= 300) {
    http.end();
    pcBridgeOk = false;
    if (pcFailStreak < 20) pcFailStreak++;
    pcNextPollDueMs = now + min(60000UL, (unsigned long)offlinePollMinMs * (unsigned long)pcFailStreak);
    // Serial spam disabled (PC bridge polling diagnostics).
    return;
  }

  String body = http.getString();
  http.end();

  body.trim();
  pcBridgeOk = true;
  pcFailStreak = 0;
  pcNextPollDueMs = now + onlinePollMinMs;
  // Serial spam disabled (PC bridge polling diagnostics).
  processPcStatLine(body.c_str());
}

void processRpiLine(const char *line) {
  if (strncmp(line, "STAT|", 5) != 0) return;
  // While the UI is in PC/foobar mode, the Raspberry UART bridge may still stream STAT lines.
  // Do not let those updates clobber the RPi metadata buffers (they are reused for PC title ticker).
  // Only block STAT updates while in PCA/PCD screens (they reuse buffers for foobar UI).
  if (!standbyState && (fmRadio.sursa == 4 || fmRadio.sursa == 5)) return;
  // #region agent log: H2 STAT redraw spikes
  profStatCount++;
  // #endregion
  // #region agent log: H5 measure full STAT handling + heap jitter
  uint32_t statT0 = micros();
  #if defined(ARDUINO_ARCH_ESP32)
  profHeapNow = ESP.getFreeHeap();
  uint32_t mh = ESP.getMinFreeHeap();
  if (mh < profHeapMin) profHeapMin = mh;
  #endif
  // #endregion
  const char *p = line + 5;
  const char *s1 = strchr(p, '|'); if (!s1) return;
  const char *s2 = strchr(s1 + 1, '|'); if (!s2) return;
  const char *s3 = strchr(s2 + 1, '|'); if (!s3) return;

  char newStateBuf[16], newTitleBuf[96], newArtistBuf[96], newFileBuf[96];
  copyTrim(newStateBuf, sizeof(newStateBuf), p, s1);
  copyTrim(newTitleBuf, sizeof(newTitleBuf), s1 + 1, s2);
  copyTrim(newArtistBuf, sizeof(newArtistBuf), s2 + 1, s3);
  copyTrim(newFileBuf, sizeof(newFileBuf), s3 + 1, s3 + 1 + strlen(s3 + 1));

  if (newTitleBuf[0] == '\0') strcpy(newTitleBuf, "-");
  if (newArtistBuf[0] == '\0') strcpy(newArtistBuf, "-");
  if (newFileBuf[0] == '\0') strcpy(newFileBuf, "-");
  if (newStateBuf[0] == '\0') strcpy(newStateBuf, "unknown");
  normalizePlayerState(newStateBuf, sizeof(newStateBuf));

  bool stateChanged = (strcmp(newStateBuf, rpiStateBuf) != 0);
  bool titleChanged = (strcmp(newTitleBuf, rpiTitleBuf) != 0);
  bool artistChanged = (strcmp(newArtistBuf, rpiArtistBuf) != 0);
  bool fileChanged = (strcmp(newFileBuf, rpiFileBuf) != 0);
  bool majorChanged = stateChanged || titleChanged || artistChanged;
  strncpy(rpiStateBuf, newStateBuf, sizeof(rpiStateBuf) - 1);
  rpiStateBuf[sizeof(rpiStateBuf) - 1] = '\0';
  strncpy(rpiTitleBuf, newTitleBuf, sizeof(rpiTitleBuf) - 1);
  rpiTitleBuf[sizeof(rpiTitleBuf) - 1] = '\0';
  strncpy(rpiArtistBuf, newArtistBuf, sizeof(rpiArtistBuf) - 1);
  rpiArtistBuf[sizeof(rpiArtistBuf) - 1] = '\0';
  strncpy(rpiFileBuf, newFileBuf, sizeof(rpiFileBuf) - 1);
  rpiFileBuf[sizeof(rpiFileBuf) - 1] = '\0';
  rpiConnected = true;
  lastRpiRxMs = millis();

  // Live UI refresh on RPi screen (throttled) so Prev/Next updates metadata quickly.
  static unsigned long lastUiUpdateMs = 0;
  if (!standbyState && !fmRadio.utilMode && (fmRadio.sursa == 3 && !pcdMode) && (majorChanged || fileChanged)) {
    unsigned long now = millis();
    if (now - lastUiUpdateMs >= 280UL) {
      fmRadio.updateMainContent();
      updateRpiTransportPlayIcon(true);
      updateRpiPlayIndicator(true);
      lastUiUpdateMs = now;
    }
  }
  // #region agent log: H5 measure full STAT handling + heap jitter
  uint32_t statDt = micros() - statT0;
  if (statDt > profStatUsMax) profStatUsMax = statDt;
  // #endregion
}

void pollRpiSerial() {
  if (!rpiUartEnabled) return;
  // Decouple non-RPi sources from the Raspberry bridge state/startup.
  // Outside standby/RPi context we only drain a tiny amount of UART RX and skip GET polling/command retries.
  if (!isRpiContextActive()) {
    for (int iter = 0; iter < 16 && Serial2.available(); iter++) {
      char c = (char)Serial2.read();
      if (c == '\n') rpiRxLen = 0;
      else if (c != '\r' && rpiRxLen < (sizeof(rpiRxLine) - 1)) rpiRxLine[rpiRxLen++] = c;
    }
    if (rpiConnected && (millis() - lastRpiRxMs >= 5000UL)) {
      rpiConnected = false;
    }
    return;
  }
  // #region agent log: H3 measure pollRpiSerial spikes
  uint32_t t0 = micros();
  // #endregion
  // Don't spend unbounded time draining a large UART burst in one loop() tick.
  for (int iter = 0; iter < 48 && Serial2.available(); iter++) {
    if ((iter & 15) == 15) yield();
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rpiRxLen > 0) {
        rpiRxLine[rpiRxLen] = '\0';
        processRpiLine(rpiRxLine);
        rpiRxLen = 0;
      }
      continue;
    }
    if (rpiRxLen < (sizeof(rpiRxLine) - 1)) rpiRxLine[rpiRxLen++] = c;
  }

  unsigned long now = millis();
  if (rpiForceGetAtMs && (long)(now - rpiForceGetAtMs) >= 0) {
    if (serial2TryPrint("GET\n")) {
      lastRpiPollMs = now;
      rpiForceGetAtMs = 0;
    } else {
      rpiForceGetAtMs = now + 150UL;
    }
  }
  // If link is online keep quick refresh; if offline back off retries to reduce UART churn.
  const unsigned long onlinePollMs = 3000UL;
  const unsigned long offlinePollMs = 30000UL;
  const unsigned long pollIntervalMs = rpiConnected ? onlinePollMs : offlinePollMs;
  const unsigned long minSilenceMs = rpiConnected ? 1800UL : 8000UL;
  // Bridge already pushes STAT periodically; request GET only when updates look stale.
  // În standby cerem tot GET ca să vedem dacă Moode e în play (indicator pe ecran).
  if ((standbyState || (!standbyState && fmRadio.sursa == 3 && !pcdMode))
      && (now - lastRpiPollMs >= pollIntervalMs)
      && (now - lastRpiRxMs >= minSilenceMs)) {
    if (serial2TryPrint("GET\n")) {
      lastRpiPollMs = now;
    }
  }
  if (rpiConnected && (now - lastRpiRxMs >= 5000UL)) {
    rpiConnected = false;
    if (!standbyState && !fmRadio.utilMode && fmRadio.sursa == 3) {
      fmRadio.updateMainContent();
    }
  }
  // #region agent log: H3 measure pollRpiSerial spikes
  uint32_t dt = micros() - t0;
  if (dt > profPollUsMax) profPollUsMax = dt;
  // #endregion
}

//------------------------------------
// MCP23017 init: try 0x20..0x27 (A2,A1,A0), all 16 pins INPUT with pull-up
//------------------------------------
void initMcp() {
  mcpOk = false;
  mcpI2cAddr = 0;
  if (mcp.begin_I2C(MCP23017_ADDR)) {
    mcpI2cAddr = MCP23017_ADDR;
    mcpOk = true;
  } else {
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
      if (addr == MCP23017_ADDR) continue;
      if (mcp.begin_I2C(addr)) {
        mcpI2cAddr = addr;
        mcpOk = true;
        break;
      }
      delay(5);
    }
  }
  if (!mcpOk) {
    Serial.println("MCP23017 not found on I2C (tried 0x20..0x27). Check SDA/SCL and A0,A1,A2.");
    return;
  }
  for (uint8_t i = 0; i < 16; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }
  cachedPortA = (uint8_t)mcp.readGPIOAB() & 0xFF;
  cachedPortB = (uint8_t)(mcp.readGPIOAB() >> 8);
  mcpLastEncA = (cachedPortB >> 6) & 1;
  mcpLastEncB = (cachedPortB >> 7) & 1;
  Serial.print("MCP23017 OK @ 0x");
  Serial.println(mcpI2cAddr, HEX);
}

// --------------------------------------------------------------------------------------------------
// End of TIME
//------------------------------------
// Setup & Loop
//------------------------------------
void setup() {
  // IMPORTANT: Keep the DAC visible to Raspberry at boot.
  // If the I2S mux is left on Amanero during Pi boot, the ES9038 HAT may fail to probe
  // (no ALSA card => Moode "Output device" disappears) and won't recover until reboot.
  if (PC_DIGITAL_STATUS_PIN >= 0) {
    pinMode(PC_DIGITAL_STATUS_PIN, OUTPUT);
    // Default to Raspberry (non-PCD) until we load settings and apply the real source.
    const bool wantAmanero = false;
    const bool level = PC_DIGITAL_STATUS_AMANERO_LEVEL_HIGH ? wantAmanero : !wantAmanero;
    digitalWrite(PC_DIGITAL_STATUS_PIN, level ? HIGH : LOW);
  }

  Serial.begin(115200);
  delay(250);
  // Reset diagnostics (helps explain repeated "Starting setup..." loops)
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print("[BOOT] resetReason: ");
  Serial.println((int)esp_reset_reason());
  Serial.print("[BOOT] heap=");
  Serial.println(ESP.getFreeHeap());
#endif
  // Boot-time TDA volume log (10s cadence for 5 minutes).
#if BOOT_TDA_VOLUME_LOG
  bootTdaLogActive = true;
  bootTdaLogNextMs = millis() + 10000UL;
  bootTdaLogUntilMs = millis() + 300000UL;
#endif
  // RPi UART is enabled only when source=RPI/standby; keep it off during boot to avoid RX noise flooding.
  setRpiUartEnabled(false);
  rpiBridgeArmed = false;
  lastSourceSeen = fmRadio.sursa;

  Serial.println("Starting setup...");

  // GPIO33: indicator / alimentare logică — HIGH la pornire; nu se comută la standby
  pinMode(POWER_INDICATOR_PIN, OUTPUT);
  digitalWrite(POWER_INDICATOR_PIN, HIGH);
  pinMode(RADIO_LED_PIN, OUTPUT);
  digitalWrite(RADIO_LED_PIN, LOW);    // Will be set in loop when FM is active

  if (PC_AUDIO_SEL_PIN >= 0) {
    pinMode(PC_AUDIO_SEL_PIN, OUTPUT);
    // Default to PCA (XMOS) on boot.
    const bool level = PC_AUDIO_SEL_ACTIVE_HIGH ? false : true;
    digitalWrite(PC_AUDIO_SEL_PIN, level ? HIGH : LOW);
  }
  // PC_DIGITAL_STATUS_PIN was already initialized at the top of setup() to keep DAC probe stable.

  // Initialize I2C with explicit pins
  Serial.println("Initializing I2C...");
  Wire.begin(RADIO_SDAPIN, RADIO_SCLPIN);
  delay(100); // Give I2C bus time to stabilize

  scanI2CBus();
  
  // Initialize TDA7439
  Serial.println("Initializing TDA7439...");
  // Legacy TDA7439 library: no explicit begin(); rely on Wire.begin() only
  
  // Check if TDA7439 is responding
  Wire.beginTransmission(0x44);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("TDA7439 Error: ");
    Serial.println(error);
    Serial.println("Please check TDA7439 connections:");
    Serial.println("1. SDA and SCL connections");
    Serial.println("2. Power supply to TDA7439");
    Serial.println("3. I2C address (should be 0x44)");
  } else {
    Serial.println("TDA7439 initialized successfully");
  }
  
  // Check I2C bus
  Wire.beginTransmission(SI4703_ADDR);
  error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
    Serial.println("Please check I2C connections:");
    Serial.print("SDA: GPIO");
    Serial.println(RADIO_SDAPIN);
    Serial.print("SCL: GPIO");
    Serial.println(RADIO_SCLPIN);
  } else {
    Serial.println("I2C bus OK");
  }

  Serial.println("SPI.begin()");
  SPI.begin();
  Serial.println("rtc.begin()");
  rtc.begin();
  Serial.println("ts.begin()");
  fmRadio.ts.begin();

  initMcp();
  setEncoderCount(fmRadio.currentVolume);
  encoderLastEncVal = fmRadio.currentVolume;
  Serial.println("MCP23017 and encoder count initialized");

  connectToWiFi();

#if defined(ARDUINO_ARCH_ESP32)
  // Run scheduled (hourly) NTP sync in the background to avoid UI freezes.
  // Manual sync from the touchscreen button intentionally remains blocking.
  if (ntpSyncTaskHandle == nullptr) {
    // Pin NTP sync away from the web server task (which runs on core 0) to reduce chances
    // of WiFi/SNTP work starving HTTP handling during early boot.
    xTaskCreatePinnedToCore(ntpSyncTask, "ntpSyncTask", 4096, nullptr, 1, &ntpSyncTaskHandle, 1);
  }
  // PC bridge polling task (keeps blocking HTTP out of loop()).
  if (ENABLE_PC_BRIDGE_POLLING && pcBridgeTaskHandle == nullptr) {
    // Pin PC bridge polling to the same core as the web/WiFi work (core 0) to avoid
    // intermittent/permanent connect failures seen when running WiFiClient from core 1.
    xTaskCreatePinnedToCore(pcBridgeTask, "pcBridgeTask", 4096, nullptr, 1, &pcBridgeTaskHandle, 0);
  }
#endif

  // First scheduled cycle: delay after boot to avoid WiFi/SNTP churn while Web UI is starting up.
  // (User reported a ~10–15s post-boot freeze window.)
  nextNtpSyncDueMs = millis() + 180000UL; // 3 minutes

  // --- OTA SETUP ---
  ArduinoOTA.setHostname("flo-amp");
  ArduinoOTA.setPassword("quickprint");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  Serial.print("[BUILD] PC_ONLY_FOOBAR=");
  Serial.println((int)PC_ONLY_FOOBAR);

  Serial.println("fmRadio.loadSettings()");
  fmRadio.loadSettings();
  // Restored sursa from NVS must arm the RPi UART bridge; otherwise rpiBridgeArmed stays false from boot
  // and RPI shows offline / never receives STAT until user presses the physical RPI button or POST /source.
  if (fmRadio.sursa == 3) {
    rpiBridgeArmed = true;
    Serial.println("[RPI] Bridge armed from saved source=RPI");
    // Bring UART up immediately so we can receive STAT without requiring another UI action.
#if !PC_ONLY_FOOBAR
    setRpiUartEnabled(true);
    rpiForceGetAtMs = millis() + 220UL;
#endif
  }

  // No PC audio routing mux.

  // Ensure SI4703 is in 2‑wire (I2C) mode and bus is healthy, then detect
  i2cBusRecover();
  resetSI4703Hardware();
  si4703Powered = detectSI4703();
  if (si4703Powered) {
    fmRadio.initRadio();
    fmRadio.radio.setFrequency(fmRadio.currentFrequency * 100);
    fmRadio.radio.setVolume(10);
    fmRadio.radio.attachReceiveRDS(FMRadioController::RDS_process);
    Serial.println("SI4703 detected and initialized in setup().");
  } else {
    Serial.println("SI4703 not detected, running in no-radio mode.");
  }
  
  // Initialize the display AFTER radio init to avoid shared RESET conflicts
  Serial.println("Calling fmRadio.initDisplay()...");
  fmRadio.initDisplay();
  Serial.println("Display (initDisplay) called in setup()");
  fmRadio.initButtons();
  // Apply saved audio settings to TDA7439 for the current source
  applyTDA7439SettingsForCurrentSource();
  // Retry shortly after power-up to ensure gain is latched
  delay(50);
  applyTDA7439SettingsForCurrentSource();
  // Schedule a final post-boot apply a bit later to catch late power-up of TDA7439 rails
  postBootAudioApplyPending = true;
  postBootAudioApplyAt = millis() + 400;
  // Force startup volume to 10 on power-on
  fmRadio.currentVolume = 10;
  if (si4703Powered) {
    fmRadio.radio.setVolume(fmRadio.currentVolume);
  }
  tda7439.setVolume(fmRadio.currentVolume);
  noteTdaWrite("boot", -1, fmRadio.currentVolume);
  delay(5);
  setEncoderCount(fmRadio.currentVolume);
  encoderLastEncVal = fmRadio.currentVolume;
  fmRadio.updateVolumeDisplay();
  fmRadio.drawInitialScreen();
  fmRadio.updateModeDisplay();
  fmRadio.updateSelectedSourceDisplay();
  fmRadio.utilMode = false;
  fmRadio.wasSetup = false;

  // --- Web server setup ---
#if ENABLE_WEBUI
  server.on("/", handleRoot);
  // v2 API
  server.on("/api/v2/state", HTTP_GET, handleApiV2State);
  server.on("/api/v2/power", HTTP_POST, handleApiV2Power);
  server.on("/api/v2/mute", HTTP_POST, handleApiV2Mute);
  server.on("/api/v2/volume", HTTP_POST, handleApiV2Volume);
  server.on("/api/v2/source", HTTP_POST, handleApiV2Source);
  server.on("/api/v2/eq", HTTP_POST, handleApiV2Eq);
  server.on("/api/v2/pc/test", HTTP_GET, handleApiV2PcTest);
  server.on("/api/v2/tuner/seek", HTTP_POST, handleApiV2TunerSeek);
  server.on("/api/v2/rpi/cmd", HTTP_POST, handleApiV2RpiCmd);
  server.on("/api/v2/pc/cmd", HTTP_POST, handleApiV2PcCmd);
  // Back-compat for Standby page and older bookmarks
  server.on("/systemInfo", HTTP_GET, handleSystemInfo);
  // v2 WebUI still uses a few legacy read/write endpoints for tuner presets.
  server.on("/presets", HTTP_GET, handlePresets);
  server.on("/preset", handlePreset);
  server.on("/freq", handleFreq);
  /*
   * === WEBUI_V1_BEGIN ===
   * Legacy endpoints (kept for reference; disabled):
   *   server.on(\"/power\", HTTP_POST, handlePower);
   *   server.on(\"/mute\", handleMute);
   *   server.on(\"/volume\", handleVolume);
   *   server.on(\"/source\", handleSource);
   *   server.on(\"/freq\", handleFreq);
   *   server.on(\"/eq\", handleEQ);
   *   server.on(\"/seek\", HTTP_POST, handleSeek);
   *   server.on(\"/rpi\", HTTP_POST, handleRpi);
   *   server.on(\"/pc\", HTTP_POST, handlePc);
   *   server.on(\"/status\", HTTP_GET, handleStatus);
   *   server.on(\"/systemInfo\", HTTP_GET, handleSystemInfo);
   *   server.on(\"/presets\", handlePresets);
   *   server.on(\"/preset\", handlePreset);
   * === WEBUI_V1_END ===
   */
  server.begin();
#endif
#if defined(ARDUINO_ARCH_ESP32) && ENABLE_WEBUI
  // Start accepting TCP immediately; do not wait for mDNS / I2C cache below (was adding 0.2–1.2s dead air).
  if (webServerTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(webServerTask, "webServerTask", 16384, nullptr, 2, &webServerTaskHandle, 0);
  }
#endif
  // Initializare cache system info (I2C o singura data in setup)
  Wire.beginTransmission(0x44);
  cachedTdaOk = (Wire.endTransmission() == 0);
  Wire.beginTransmission(0x68);
  cachedRtcOk = (Wire.endTransmission() == 0);
  cachedMcpOk = (mcpI2cAddr != 0) || mcpOk;
  Wire.beginTransmission(SI4703_ADDR);
  cachedSi47Ok = (Wire.endTransmission() == 0);
  lastSystemInfoCacheMs = millis();

  // mDNS pentru http://flo-amp.local (daca nu merge, foloseste IP-ul afisat mai jos)
  if (WiFi.status() == WL_CONNECTED) {
    delay(80);
    if (!MDNS.begin("flo-amp")) {
      delay(400);
      if (!MDNS.begin("flo-amp")) {
        Serial.println("mDNS esuat - deschide Web UI cu IP-ul de mai jos");
      } else {
        MDNS.addService("http", "tcp", 80);
        Serial.println("mDNS OK: http://flo-amp.local");
      }
    } else {
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS OK: http://flo-amp.local");
    }
    Serial.println("----------------------------------------");
    Serial.print("Web UI:  http://");
    Serial.println(WiFi.localIP());
    Serial.println("----------------------------------------");
  }

  // Set up IR receiver on pin 12
  IrReceiver.begin(12, ENABLE_LED_FEEDBACK); // Use pin 12 for IR

  // Ensure standby button pin is set up
  pinMode(STANDBY_CTRL_PIN, INPUT_PULLUP);

 // Removed debug direct volume write that overwrote saved state

  tda7439.spkAtt(0); // Set speaker attenuation to middle value at startup
  
  // Run a brief TDA7439 communication test and report to Serial
  testTDA7439Comms();
}

static void tickSourceTransitionsAndBridgeIo() {
  const int prevSource = lastSourceSeen;
#if PC_ONLY_FOOBAR
  setRpiUartEnabled(false);
  rpiBridgeArmed = false;
  const bool rpiContextActive = false;
#else
  const bool rpiContextActive = isRpiContextActive();
  setRpiUartEnabled(rpiContextActive);
#endif

  // Entering RPi source: force an immediate state read (unless in PCD mode).
#if !PC_ONLY_FOOBAR
  if (!standbyState && fmRadio.sursa == 3 && !pcdMode && prevSource != 3) {
    unsigned long t = millis();
    if (serial2TryPrint("GET\n")) {
      lastRpiPollMs = t;
    } else {
      rpiForceGetAtMs = t + 120UL;
    }
  }
#endif

  // Entering PCA/PCD mode: pause Moode so foobar can take over.
  if (!standbyState && (fmRadio.sursa == 4 || fmRadio.sursa == 5) && (prevSource != 4 && prevSource != 5)) {
    // Keep non-RPi sources independent: only attempt PAUSE when bridge is already alive.
#if !PC_ONLY_FOOBAR
    if (rpiConnected) sendRpiCommand("PAUSE");
#endif
  }

  // Leaving PCA/PCD mode: rpiTitleBuf may have been reused as a scratch buffer for the title ticker.
  if (!standbyState && (prevSource == 4 || prevSource == 5) && (fmRadio.sursa != 4 && fmRadio.sursa != 5)) {
    strcpy(rpiTitleBuf, "-");
    pcAnalogUi = false;
  }

  lastSourceSeen = fmRadio.sursa;

  if (rpiContextActive) {
    unsigned long nowm = millis();
    if (rpiWebCmdNextTryMs != 0UL && (long)(nowm - rpiWebCmdNextTryMs) >= 0) {
      rpiWebCmdNextTryMs = 0;
    }
    if (rpiWebCmdNextTryMs == 0UL) {
      char rpiCmdLocal[sizeof(rpiWebCmdBuf)];
      bool haveRpiWeb =
#if defined(ARDUINO_ARCH_ESP32)
          popPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, sizeof(rpiCmdLocal), &rpiWebCmdMux);
#else
          popPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, sizeof(rpiCmdLocal));
#endif
      if (haveRpiWeb && !sendRpiCommand(rpiCmdLocal)) {
        rpiWebCmdNextTryMs = nowm + 55UL;
#if defined(ARDUINO_ARCH_ESP32)
        setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, &rpiWebCmdMux);
#else
        setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal);
#endif
      }
    }
  } else {
    // Outside standby/RPi context, keep UART command retries disabled.
    rpiWebCmdNextTryMs = 0;
    rpiForceGetAtMs = 0;
  }

#if !PC_ONLY_FOOBAR
  pollRpiSerial();
#endif
  // PC bridge: either a background task feeds STAT lines, or we poll from the main loop here.
#if defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_PC_BRIDGE_POLLING) {
    char line[220];
    if (pcPopPendingStatLine(line, sizeof(line))) {
      processPcStatLine(line);
    }
  } else {
    pollPcBridge();
  }
#else
  pollPcBridge();
#endif

  {
    const unsigned long now = millis();
    if ((long)(now - webStatusWifiRssiAtMs) >= 0) {
      webStatusWifiRssiAtMs = now + 1000UL;
      webStatusWifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
    }
  }

  {
    char cmdLocal[sizeof(pcWebCmdBuf)];
    bool havePcCmd =
#if defined(ARDUINO_ARCH_ESP32)
        popPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmdLocal, sizeof(cmdLocal), &pcWebCmdMux);
#else
        popPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmdLocal, sizeof(cmdLocal));
#endif
    if (havePcCmd) sendPcCommand(cmdLocal);
  }

#if !PC_ONLY_FOOBAR
  updateRpiTitleTicker(false);
  updateRpiTransportPlayIcon(false);
  updateRpiPlayIndicator(false);
#endif
  // Coalesce frequent UI-originated preference writes.
  fmRadio.flushSettingsIfDue(false);
}

static void tickPeriodicSchedulers() {
  // #region agent log: H1/H2/H3 periodic freeze profiler summary (~2s cadence)
  if (profWindowStartMs == 0) profWindowStartMs = millis();
  unsigned long nowMs = millis();
  if (!standbyState && fmRadio.sursa == 3 && (nowMs - profWindowStartMs) >= 2000UL) {
    uint32_t p0 = micros();
    if (Serial.availableForWrite() >= 256) {
      profProfSent++;
      static uint32_t profNdjsonSeq = 0;
      profNdjsonSeq++;
      if (Serial.availableForWrite() >= 512) {
        uint32_t tsMs = (uint32_t)millis();
#if defined(ARDUINO_ARCH_ESP32)
        uint32_t heapNow = ESP.getFreeHeap();
#else
        uint32_t heapNow = 0;
#endif
        (void)tsMs;
        (void)heapNow;
      }
    } else {
      profProfSkipped++;
    }
    uint32_t pdt = micros() - p0;
    if (pdt > profPrintUsMax) profPrintUsMax = pdt;
    if (Serial.availableForWrite() >= 64) {
      (void)pdt;
    }
    profWindowStartMs = nowMs;
    profDrawUsMax = 0;
    profMainContentUsMax = 0;
    profLoopUsMax = 0;
    profPollUsMax = 0;
    profSysInfoRuns = 0;
    profNvsWrites = 0;
    profFmUpdateUsMax = 0;
    profTouchUsMax = 0;
    profShowTimeUsMax = 0;
    profStatMiniUsMax = 0;
    profStatUsMax = 0;
    profTickerUsMax = 0;
    profTickerDraws = 0;
    profPrintUsMax = 0;
    profProfSent = 0;
    profProfSkipped = 0;
    profApplyTdaUsMax = 0;
    profStatCount = 0;
  }
  // #endregion

  // Actualizare cache system info la 5 s (I2C doar in main loop, nu in handler web)
  if (millis() - lastSystemInfoCacheMs >= 5000) {
    profSysInfoRuns++;
    lastSystemInfoCacheMs = millis();
    Wire.beginTransmission(0x44);
    cachedTdaOk = (Wire.endTransmission() == 0);
    Wire.beginTransmission(0x68);
    cachedRtcOk = (Wire.endTransmission() == 0);
    cachedMcpOk = (mcpI2cAddr != 0) || mcpOk;
    Wire.beginTransmission(SI4703_ADDR);
    cachedSi47Ok = (Wire.endTransmission() == 0);
  }

  // Re-sync RTC from NTP every hour starting from boot.
  if ((long)(millis() - nextNtpSyncDueMs) >= 0) {
#if defined(ARDUINO_ARCH_ESP32)
    if (!ntpSyncInProgress && !ntpSyncRequested && ntpSyncTaskHandle != nullptr) {
      ntpSyncRequested = true;
      xTaskNotifyGive(ntpSyncTaskHandle);
    }
#else
    syncRTCWithNTPRetries();
#endif
    nextNtpSyncDueMs += NTP_SYNC_INTERVAL_MS;
    if ((long)(millis() - nextNtpSyncDueMs) >= 0) {
      nextNtpSyncDueMs = millis() + NTP_SYNC_INTERVAL_MS;
    }
  }
}

static void tickDeferredAppliesAndActions() {
  if (postBootAudioApplyPending && millis() >= postBootAudioApplyAt && !standbyState) {
    applyTDA7439SettingsForCurrentSource();
    postBootAudioApplyPending = false;
  }

  if (audioApplyPending && millis() >= audioApplyAt && !standbyState) {
    // NVS read belongs in main loop, not in the HTTP handler (was stretching /api/v2/source timeline).
    fmRadio.loadEqualizer();
    applyTDA7439SettingsForCurrentSource();
    audioApplyPending = false;
  }

  if (powerOnPending) {
    powerOnPending = false;
    exitStandby();
  } else if (powerOffPending) {
    powerOffPending = false;
    enterStandby();
  }

  if (buttonPressed(STANDBY_CTRL_PIN, lastStandbyState, lastStandbyPressTime)) {
    standbyState = !standbyState;
    if (standbyState) enterStandby();
    else exitStandby();
  }

  digitalWrite(RADIO_LED_PIN, (!standbyState && fmRadio.sursa == 1) ? HIGH : LOW);

  if (freqApplyPending && !standbyState && fmRadio.sursa == 1) {
    if (!si4703Powered) {
      freqApplyPending = false;
    } else {
      float f = pendingFreq;
      fmRadio.currentFrequency = f;
      fmRadio.radio.setFrequency(f * 100);
      if (!fmRadio.isInVolumeMode) {
        setEncoderCount(f * 10);
      } else {
        setEncoderCount(fmRadio.currentVolume);
      }
      fmRadio.oldFrequency = f;
      fmRadio.lastFrequencyChangeTime = millis();
      fmRadio.updateMainContent();
      fmRadio.saveSettings();
      delay(1);
      float actualFreq = fmRadio.radio.getFrequency() / 100.0;
      if (abs(actualFreq - f) > 0.1) {
        fmRadio.radio.setFrequency(f * 100);
        delay(1);
      }
      freqApplyPending = false;
    }
  }

  if (volumeApplyPending && !standbyState) {
    int v = pendingVolume;
    const int applied = fmRadio.isMuted ? 0 : v;
    tda7439.setVolume(applied);
    noteTdaWrite("vol", -1, applied);
    setEncoderCount(v);
    encoderLastEncVal = v;
    fmRadio.updateVolumeDisplay();
    fmRadio.saveSettings();
    volumeApplyPending = false;
  }

  if (eqApplyPending && !standbyState) {
    int tdaGain = (Gain + 45) / 3; // 0..15
    tda7439.inputGain(tdaGain);
    delay(1);
    tda7439.setSnd(Bass, 1);
    delay(1);
    tda7439.setSnd(Middle, 2);
    delay(1);
    tda7439.setSnd(Treble, 3);
    delay(1);
    setTDA7439Balance((int8_t)Balance);
    delay(1);
    // EQ from Web UI uses debounced saveSettings(); force NVS write now so Gain/EQ survive reboot.
    fmRadio.saveSettings();
    fmRadio.flushSettingsIfDue(true);
    eqApplyPending = false;
  }

  if (pendingSeekDir != 0 && !standbyState && fmRadio.sursa == 1) {
    if (pendingSeekDir > 0) fmRadio.seekUp();
    else fmRadio.seekDown();
    pendingSeekDir = 0;
  }

  if (uiRefreshPending) {
    if (fmRadio.utilMode) {
      drawUtilSourceUnderline(fmRadio.sursa);
      fmRadio.updateUtilSursaChange();
    } else {
      fmRadio.updateSelectedSourceDisplay();
      fmRadio.updateMainContent();
    }
    uiRefreshPending = false;
  }

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      delay(500);
      ESP.restart();
    } else if (c == 't') {
      testTDA7439Comms();
    }
  }
}

void loop() {
  // #region agent log: H3 measure full loop spikes
  uint32_t loopT0 = micros();
  // #endregion
  mainLoopCounter++;

#if defined(ARDUINO_ARCH_ESP32)
  // Update UI state snapshot for background tasks.
  pcUiSource = fmRadio.sursa;
  pcUiUtilMode = fmRadio.utilMode;
#endif
  // HTTP/OTA handled in a separate task (ESP32)
#if !defined(ARDUINO_ARCH_ESP32)
  #if ENABLE_WEBUI
  ArduinoOTA.handle();
  server.handleClient();
  #endif
#endif

  maintainWiFiConnection();

  tickBootTdaVolumeLog(fmRadio.currentVolume, fmRadio.isMuted);

  tickSourceTransitionsAndBridgeIo();
  tickPeriodicSchedulers();
  tickDeferredAppliesAndActions();

#if 0
    const int prevSource = lastSourceSeen;

  // Entering RPi source: force an immediate state read (unless in PCD mode).
  if (!standbyState && fmRadio.sursa == 3 && !pcdMode && prevSource != 3) {
    // Use plain GET (not CMD|GET) to bypass TX flood guards and get immediate metadata.
    unsigned long t = millis();
    if (serial2TryPrint("GET\n")) {
      lastRpiPollMs = t;
    } else {
      rpiForceGetAtMs = t + 120UL;
    }
  }

  // Entering PCA/PCD mode: pause Moode so foobar can take over.
  if (!standbyState && (fmRadio.sursa == 4 || fmRadio.sursa == 5) && (prevSource != 4 && prevSource != 5)) {
    sendRpiCommand("PAUSE");
  }

  // Leaving PCA/PCD mode: rpiTitleBuf may have been reused as a scratch buffer for the title ticker.
  if (!standbyState && (prevSource == 4 || prevSource == 5) && (fmRadio.sursa != 4 && fmRadio.sursa != 5)) {
    strcpy(rpiTitleBuf, "-");
    pcAnalogUi = false;
  }

  lastSourceSeen = fmRadio.sursa;

  {
    unsigned long nowm = millis();
    if (rpiWebCmdNextTryMs != 0UL && (long)(nowm - rpiWebCmdNextTryMs) < 0) {
      // UART TX was full; backoff before retrying same command (prevents 100% CPU spin).
    } else {
      rpiWebCmdNextTryMs = 0;
      char rpiCmdLocal[sizeof(rpiWebCmdBuf)];
      bool haveRpiWeb =
#if defined(ARDUINO_ARCH_ESP32)
          popPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, sizeof(rpiCmdLocal), &rpiWebCmdMux);
#else
          popPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, sizeof(rpiCmdLocal));
#endif
      if (haveRpiWeb) {
        if (!sendRpiCommand(rpiCmdLocal)) {
          rpiWebCmdNextTryMs = nowm + 55UL;
#if defined(ARDUINO_ARCH_ESP32)
          setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal, &rpiWebCmdMux);
#else
          setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), rpiCmdLocal);
#endif
        }
      }
    }
  }

  pollRpiSerial();
  pollPcBridge();
  {
    char cmdLocal[sizeof(pcWebCmdBuf)];
    bool havePcCmd =
#if defined(ARDUINO_ARCH_ESP32)
        popPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmdLocal, sizeof(cmdLocal), &pcWebCmdMux);
#else
        popPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmdLocal, sizeof(cmdLocal));
#endif
    if (havePcCmd) sendPcCommand(cmdLocal);
  }
  updateRpiTitleTicker(false);
  updateRpiTransportPlayIcon(false);
  updateRpiPlayIndicator(false);

  // Coalesce frequent UI-originated preference writes.
  fmRadio.flushSettingsIfDue(false);

  // #region agent log: H1/H2/H3 periodic freeze profiler summary (~2s cadence)
  if (profWindowStartMs == 0) profWindowStartMs = millis();
  unsigned long nowMs = millis();
  if (!standbyState && fmRadio.sursa == 3 && (nowMs - profWindowStartMs) >= 2000UL) {
    // #region agent log: H8 measure Serial.printf blocking
    uint32_t p0 = micros();
    // #endregion
    if (Serial.availableForWrite() >= 256) {
      profProfSent++;
      // Serial.printf("[PROF] win=%lums stat=%lu drawUsMax=%lu mainUsMax=%lu\n",
      //               (unsigned long)(nowMs - profWindowStartMs),
      //               (unsigned long)profStatCount,
      //               (unsigned long)profDrawUsMax,
      //               (unsigned long)profMainContentUsMax);
      // Serial.printf("[PROF2] loopUsMax=%lu pollUsMax=%lu sysInfo=%u nvsWrites=%u\n",
      //               (unsigned long)profLoopUsMax,
      //               (unsigned long)profPollUsMax,
      //               (unsigned)profSysInfoRuns,
      //               (unsigned)profNvsWrites);
      // Serial.printf("[PROF3] fmUpdUsMax=%lu touchUsMax=%lu timeUsMax=%lu statMiniUsMax=%lu\n",
      //               (unsigned long)profFmUpdateUsMax,
      //               (unsigned long)profTouchUsMax,
      //               (unsigned long)profShowTimeUsMax,
      //               (unsigned long)profStatMiniUsMax);
      // Serial.printf("[PROF4] statUsMax=%lu tickerUsMax=%lu tickerDraws=%u heapNow=%lu heapMin=%lu\n",
      //               (unsigned long)profStatUsMax,
      //               (unsigned long)profTickerUsMax,
      //               (unsigned)profTickerDraws,
      //               (unsigned long)profHeapNow,
      //               (unsigned long)profHeapMin);
      // #region agent log: NDJSON perf snapshot for host log capture (session dfc17c)
      static uint32_t profNdjsonSeq = 0;
      profNdjsonSeq++;
      if (Serial.availableForWrite() >= 512) {
        uint32_t tsMs = (uint32_t)millis();
#if defined(ARDUINO_ARCH_ESP32)
        uint32_t heapNow = ESP.getFreeHeap();
#else
        uint32_t heapNow = 0;
#endif
        (void)tsMs;
        (void)heapNow;
        // Serial.printf(
        //     "NDJSON {\"sessionId\":\"dfc17c\",\"id\":\"esp_prof_%lu\",\"timestamp\":%lu,\"location\":\"loop.prof_window\","
        //     "\"message\":\"esp32_prof_snapshot\",\"data\":{\"hypothesisId\":\"H_MULTI\",\"seq\":%lu,\"winMs\":%lu,"
        //     "\"stat\":%lu,\"drawUsMax\":%lu,\"mainUsMax\":%lu,\"loopUsMax\":%lu,\"pollUsMax\":%lu,"
        //     "\"statUsMax\":%lu,\"tickerUsMax\":%lu,\"tickerDraws\":%u,\"applyTdaUsMax\":%lu,\"heapNow\":%lu,\"heapMin\":%lu},"
        //     "\"runId\":\"pre-fix\"}\n",
        //     (unsigned long)profNdjsonSeq,
        //     (unsigned long)tsMs,
        //     (unsigned long)profNdjsonSeq,
        //     (unsigned long)(nowMs - profWindowStartMs),
        //     (unsigned long)profStatCount,
        //     (unsigned long)profDrawUsMax,
        //     (unsigned long)profMainContentUsMax,
        //     (unsigned long)profLoopUsMax,
        //     (unsigned long)profPollUsMax,
        //     (unsigned long)profStatUsMax,
        //     (unsigned long)profTickerUsMax,
        //     (unsigned)profTickerDraws,
        //     (unsigned long)profApplyTdaUsMax,
        //     (unsigned long)heapNow,
        //     (unsigned long)profHeapMin);
      }
      // #endregion
    } else {
      profProfSkipped++;
    }
    // #region agent log: H8 measure Serial.printf blocking
    uint32_t pdt = micros() - p0;
    if (pdt > profPrintUsMax) profPrintUsMax = pdt;
    if (Serial.availableForWrite() >= 64) {
      // Serial.printf("[PROF5] printUs=%lu profSent=%u profSkip=%u applyTdaUsMax=%lu\n",
      //               (unsigned long)pdt,
      //               (unsigned)profProfSent,
      //               (unsigned)profProfSkipped,
      //               (unsigned long)profApplyTdaUsMax);
      (void)pdt;
    }
    // #endregion
    profWindowStartMs = nowMs;
    profDrawUsMax = 0;
    profMainContentUsMax = 0;
    profLoopUsMax = 0;
    profPollUsMax = 0;
    profSysInfoRuns = 0;
    profNvsWrites = 0;
    profFmUpdateUsMax = 0;
    profTouchUsMax = 0;
    profShowTimeUsMax = 0;
    profStatMiniUsMax = 0;
    profStatUsMax = 0;
    profTickerUsMax = 0;
    profTickerDraws = 0;
    profPrintUsMax = 0;
    profProfSent = 0;
    profProfSkipped = 0;
    profApplyTdaUsMax = 0;
    profStatCount = 0;
  }
  // #endregion

  // Actualizare cache system info la 5 s (I2C doar in main loop, nu in handler web)
  if (millis() - lastSystemInfoCacheMs >= 5000) {
    // #region agent log: H3 system info cache periodic work
    profSysInfoRuns++;
    // #endregion
    lastSystemInfoCacheMs = millis();
    Wire.beginTransmission(0x44);
    cachedTdaOk = (Wire.endTransmission() == 0);
    Wire.beginTransmission(0x68);
    cachedRtcOk = (Wire.endTransmission() == 0);
    cachedMcpOk = (mcpI2cAddr != 0) || mcpOk;
    Wire.beginTransmission(SI4703_ADDR);
    cachedSi47Ok = (Wire.endTransmission() == 0);
  }

  // Re-sync RTC from NTP every hour starting from boot.
  // Each cycle: up to 3 tries; if all fail, keep RTC until next scheduled cycle.
  if ((long)(millis() - nextNtpSyncDueMs) >= 0) {
    // Serial.println("[NTP] Scheduled hourly sync cycle...");
#if defined(ARDUINO_ARCH_ESP32)
    // Schedule background sync; skip if one is already running.
    if (!ntpSyncInProgress && !ntpSyncRequested && ntpSyncTaskHandle != nullptr) {
      ntpSyncRequested = true;
      xTaskNotifyGive(ntpSyncTaskHandle);
    } else {
      // Serial.println("[NTP] Background sync already in progress/requested; skipping.");
    }
#else
    syncRTCWithNTPRetries();
#endif
    nextNtpSyncDueMs += NTP_SYNC_INTERVAL_MS;
    // If we were delayed a lot, avoid running many catch-up cycles back-to-back.
    if ((long)(millis() - nextNtpSyncDueMs) >= 0) {
      nextNtpSyncDueMs = millis() + NTP_SYNC_INTERVAL_MS;
    }
  }

  // One-time delayed post-boot audio reapply to ensure gain latches
  if (postBootAudioApplyPending && millis() >= postBootAudioApplyAt && !standbyState) {
    // Serial.println("[TDA7439] Post-boot reapply settings");
    applyTDA7439SettingsForCurrentSource();
    postBootAudioApplyPending = false;
  }
  
  // Short-deferred apply triggered by HTTP handlers (e.g., source change)
  if (audioApplyPending && millis() >= audioApplyAt && !standbyState) {
    // Serial.println("[TDA7439] Deferred apply settings");
    applyTDA7439SettingsForCurrentSource();
    audioApplyPending = false;
  }

  // Handle power transitions requested by HTTP
  if (powerOnPending) {
    powerOnPending = false;
    exitStandby();
  } else if (powerOffPending) {
    powerOffPending = false;
    enterStandby();
  }

  // GPIO32: standby button (toggle on press)
  if (buttonPressed(STANDBY_CTRL_PIN, lastStandbyState, lastStandbyPressTime)) {
    standbyState = !standbyState;
    if (standbyState) enterStandby();
    else exitStandby();
  }

  // GPIO17: radio LED ON when FM source selected and not standby (works even if SI4703 not detected)
  digitalWrite(RADIO_LED_PIN, (!standbyState && fmRadio.sursa == 1) ? HIGH : LOW);

  // Apply pending frequency change (deferred from HTTP handler)
  if (freqApplyPending && !standbyState && fmRadio.sursa == 1) {
    if (!si4703Powered) {
      // Radio hardware not present; ignore pending frequency safely
      freqApplyPending = false;
    } else {
    float f = pendingFreq;
    fmRadio.currentFrequency = f;
    fmRadio.radio.setFrequency(f * 100);
    if (!fmRadio.isInVolumeMode) {
      setEncoderCount(f * 10);
    } else {
      setEncoderCount(fmRadio.currentVolume);
    }
    fmRadio.oldFrequency = f;
    fmRadio.lastFrequencyChangeTime = millis();
    fmRadio.updateMainContent();
    fmRadio.saveSettings();
    // Verify and retune if needed
    delay(1);
    float actualFreq = fmRadio.radio.getFrequency() / 100.0;
    if (abs(actualFreq - f) > 0.1) {
      fmRadio.radio.setFrequency(f * 100);
      delay(1);
    }
    freqApplyPending = false;
    }
  }

  // Apply pending volume change (deferred from HTTP handler)
  if (volumeApplyPending && !standbyState) {
    int v = pendingVolume;
    tda7439.setVolume(v);
    setEncoderCount(v);
    encoderLastEncVal = v;
    fmRadio.updateVolumeDisplay();
    fmRadio.saveSettings();
    volumeApplyPending = false;
  }

  // Apply pending EQ change (deferred from HTTP handler)
  if (eqApplyPending && !standbyState) {
    int tdaGain = (Gain + 45) / 3; // 0..15
    tda7439.inputGain(tdaGain);
    delay(1);
    tda7439.setSnd(Bass, 1);
    delay(1);
    tda7439.setSnd(Middle, 2);
    delay(1);
    tda7439.setSnd(Treble, 3);
    delay(1);
    setTDA7439Balance((int8_t)Balance);
    delay(1);
    fmRadio.saveSettings();
    fmRadio.flushSettingsIfDue(true);
    eqApplyPending = false;
  }

  // Handle pending seek
  if (pendingSeekDir != 0 && !standbyState && fmRadio.sursa == 1) {
    if (pendingSeekDir > 0) fmRadio.seekUp();
    else fmRadio.seekDown();
    pendingSeekDir = 0;
  }

  // Refresh UI requested by handlers without drawing from HTTP task
  if (uiRefreshPending) {
    if (fmRadio.utilMode) {
      drawUtilSourceUnderline(fmRadio.sursa);
      fmRadio.updateUtilSursaChange();
    } else {
      fmRadio.updateSelectedSourceDisplay();
      fmRadio.updateMainContent();
    }
    uiRefreshPending = false;
  }

  // Check for reset command from serial monitor
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      // Serial.println("[SYSTEM] Reset command received. Restarting ESP32...");
      delay(500); // Give time for the message to be sent
      ESP.restart();
    } else if (c == 't') {
      // Serial.println("[SYSTEM] Running TDA7439 communication test...");
      testTDA7439Comms();
    }
  }
#endif

  if (IrReceiver.decode()) {
    uint8_t irCode = IrReceiver.decodedIRData.command;
    // Serial.print("IR code: 0x");
    // Serial.println(irCode, HEX);
    // Serial.println(irCode, HEX); // Only IR code in HEX

    // IR code 0x1B (27 decimal) for power on/off
    if (irCode == 0x1B) {
      standbyState = !standbyState;
      if (standbyState) {
        enterStandby();
      } else {
        exitStandby();
      }
      delay(200); // Debounce
    }

    // IR code 0x1A (26 decimal) for util mode toggle
    if (irCode == 0x1A && !standbyState) {
      fmRadio.utilMode = !fmRadio.utilMode;
      fmRadio.toggleUtilMode();
      delay(200);
    }

    // --- Mute ---
    if (irCode == 0x04 && !standbyState) { // 4 = mute toggle
      fmRadio.toggleMute();
      delay(200);
    }

    // --- Volume and EQ up/down ---
    if (irCode == 0x05) { // 5 = volume up or EQ up
      if (!standbyState) {
        if (fmRadio.utilMode) {
          // In util mode: selected EQ up
          if (fmRadio.eqIndex == 0) { // Bass
            Bass = constrain(Bass + 1, -7, 7);
            tda7439.setSnd(Bass, 1);
          } else if (fmRadio.eqIndex == 1) { // Middle
            Middle = constrain(Middle + 1, -7, 7);
            tda7439.setSnd(Middle, 2);
          } else if (fmRadio.eqIndex == 2) { // Treble
            Treble = constrain(Treble + 1, -7, 7);
            tda7439.setSnd(Treble, 3);
          } else if (fmRadio.eqIndex == 3) { // Gain
            Gain = constrain(Gain + 1, -45, 0);
            tda7439.inputGain((Gain + 45) / 3);
          }
          fmRadio.updateCurrentEqValueDisplay();
          fmRadio.saveSettings();
          fmRadio.flushSettingsIfDue(true);
          // fmRadio.drawsetScreen(); // <-- ensures value color is correct
        } else {
          // Normal mode: volume up
          int32_t nextVolume = min((int32_t)(fmRadio.currentVolume + 1), (int32_t)MAX_VOLUME);
          
          // Check TDA7439 before changing volume
          Wire.beginTransmission(0x44);
          byte error = Wire.endTransmission();
          if (error == 0) {
            fmRadio.currentVolume = nextVolume;
            tda7439.setVolume(fmRadio.currentVolume);
            noteTdaWrite("ir", -1, fmRadio.currentVolume);
            // Serial.print("[TDA7439] Volume increased to: ");
            // Serial.println(fmRadio.currentVolume);
            setEncoderCount(fmRadio.currentVolume);
            encoderLastEncVal = fmRadio.currentVolume;
            fmRadio.updateVolumeDisplay();
            fmRadio.saveSettings();
          } else {
            // Serial.print("[TDA7439] NACK Error during IR volume up! Error code: ");
            // Serial.println(error);
          }
          
          setEncoderCount(fmRadio.currentVolume);
          encoderLastEncVal = fmRadio.currentVolume;
          fmRadio.updateVolumeDisplay();
          fmRadio.saveSettings();
        }
      }
      delay(100);
    }
    if (irCode == 0x00) { // 0 = volume down or EQ down
      if (!standbyState) {
        if (fmRadio.utilMode) {
          // In util mode: selected EQ down
          if (fmRadio.eqIndex == 0) { // Bass
            Bass = constrain(Bass - 1, -7, 7);
            tda7439.setSnd(Bass, 1);
          } else if (fmRadio.eqIndex == 1) { // Middle
            Middle = constrain(Middle - 1, -7, 7);
            tda7439.setSnd(Middle, 2);
          } else if (fmRadio.eqIndex == 2) { // Treble
            Treble = constrain(Treble - 1, -7, 7);
            tda7439.setSnd(Treble, 3);
          } else if (fmRadio.eqIndex == 3) { // Gain
            Gain = constrain(Gain - 1, -45, 0);
            tda7439.inputGain((Gain + 45) / 3);
          }
          fmRadio.updateCurrentEqValueDisplay();
          fmRadio.saveSettings();
          fmRadio.flushSettingsIfDue(true);
          fmRadio.drawsetScreen(); // <-- ensures value color is correct
        } else {
          // Normal mode: volume down
          int32_t nextVolume = max((int32_t)(fmRadio.currentVolume - 1), (int32_t)MIN_VOLUME);
          
          // Check TDA7439 before changing volume
          Wire.beginTransmission(0x44);
          byte error = Wire.endTransmission();
          if (error == 0) {
            fmRadio.currentVolume = nextVolume;
            tda7439.setVolume(fmRadio.currentVolume);
            noteTdaWrite("ir", -1, fmRadio.currentVolume);
            // Serial.print("[TDA7439] Volume decreased to: ");
            // Serial.println(fmRadio.currentVolume);
            setEncoderCount(fmRadio.currentVolume);
            encoderLastEncVal = fmRadio.currentVolume;
            fmRadio.updateVolumeDisplay();
            fmRadio.saveSettings();
          } else {
            // Serial.print("[TDA7439] NACK Error during IR volume down! Error code: ");
            // Serial.println(error);
          }
          
          setEncoderCount(fmRadio.currentVolume);
          encoderLastEncVal = fmRadio.currentVolume;
          fmRadio.updateVolumeDisplay();
          fmRadio.saveSettings();
        }
      }
      delay(100);
    }

    // --- Source/EQ select left/right or Seek up/down ---
    if (irCode == 0x08) { // 8
      if (!standbyState) {
        if (fmRadio.utilMode) {
          // In util mode: previous EQ parameter
          fmRadio.eqIndex = (fmRadio.eqIndex + 3) % 4; // wrap around 0-3
          fmRadio.drawsetScreen(); // redraw all labels/values
          delay(200);
          IrReceiver.resume();
          return; // <--- Prevent further processing!
        }
        if (fmRadio.sursa == 1) {
          // Radio: seek down
          // Serial.println("[IR] Seek down command received");
          fmRadio.seekDown();
        } else {
          // Other sources: NEXT source (reversed)
          fmRadio.sursa++;
          if (fmRadio.sursa > 5) fmRadio.sursa = 1;
          fmRadio.loadEqualizer();
          fmRadio.updateSelectedSourceDisplay();
          fmRadio.updateMainContent();
          fmRadio.saveSettings();
          setTDA7439InputForSource(fmRadio.sursa);
          tda7439.setVolume(fmRadio.currentVolume);
        }
      }
      delay(200);
    }
    if (irCode == 0x01) { // 1
      if (!standbyState) {
        if (fmRadio.utilMode) {
          // In util mode: next EQ parameter
          fmRadio.eqIndex = (fmRadio.eqIndex + 1) % 4;
          fmRadio.drawsetScreen(); // redraw all labels/values
          delay(200);
          IrReceiver.resume();
          return; // <--- Prevent further processing!
        }
        if (fmRadio.sursa == 1) {
          // Radio: seek up
          // Serial.println("[IR] Seek up command received");
          fmRadio.seekUp();
        } else {
          // Other sources: PREVIOUS source (reversed)
          fmRadio.sursa--;
          if (fmRadio.sursa < 1) fmRadio.sursa = 5;
          fmRadio.loadEqualizer();
          fmRadio.updateSelectedSourceDisplay();
          fmRadio.updateMainContent();
          fmRadio.saveSettings();
          setTDA7439InputForSource(fmRadio.sursa);
          tda7439.setVolume(fmRadio.currentVolume);
        }
      }
      delay(200);
    }

    // IR code 0x1F (31 decimal) for switching sources
    if (irCode == 0x1F && !standbyState) {
      if (fmRadio.utilMode) {
        // In util mode: cycle through all sources
        fmRadio.sursa++;
        if (fmRadio.sursa > 5) fmRadio.sursa = 1;
        fmRadio.loadEqualizer();
        drawUtilSourceUnderline(fmRadio.sursa); // Only update the red underline
        fmRadio.updateUtilSursaChange();        // Update EQ values for new source
        fmRadio.saveSettings();
        lastUtilActivity = millis(); // Reset util timer
        delay(5);
        IrReceiver.resume();
        return; // Prevent normal mode code from running
      }
      // Normal mode: previous source (legacy, now reversed)
      fmRadio.sursa--;
      if (fmRadio.sursa < 1) fmRadio.sursa = 5;
      fmRadio.loadEqualizer();
      fmRadio.updateSelectedSourceDisplay();
      fmRadio.updateMainContent();
      fmRadio.saveSettings();
      delay(5); // Debounce
      setTDA7439InputForSource(fmRadio.sursa);
      tda7439.setVolume(fmRadio.currentVolume);
    }

    IrReceiver.resume();
  }

  // In standby: actualizam ceasul si verificam butonul MODE pentru trezire
  if (standbyState) {
    fmRadio.handleButtons();  // citeste MCP; MODE (PB5) apeleaza exitStandby()
    static unsigned long lastStandbyUpdate = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastStandbyUpdate >= 500) {
      showtimestandBy();
      lastStandbyUpdate = nowMs;
    }
    delay(10);
    return;
  }

  // Normal mode
  // #region agent log: H3 measure non-RPi loop work
  { uint32_t t0 = micros(); fmRadio.update(); uint32_t dt = micros() - t0; if (dt > profFmUpdateUsMax) profFmUpdateUsMax = dt; }
  { uint32_t t0 = micros(); checkTouch();    uint32_t dt = micros() - t0; if (dt > profTouchUsMax)    profTouchUsMax = dt; }
  { uint32_t t0 = micros(); showTime();      uint32_t dt = micros() - t0; if (dt > profShowTimeUsMax)  profShowTimeUsMax = dt; }
  // #endregion

  // If utilMode and rtcUpdateSuccess, redraw util button to update/hide message
  if (fmRadio.utilMode && fmRadio.rtcUpdateSuccess && millis() - fmRadio.rtcUpdateMsgMillis > 2000) {
    fmRadio.rtcUpdateSuccess = false;
    fmRadio.drawUtilButton();
    drawsetScreen();
  }

  // #region agent log: H3 measure full loop spikes
  uint32_t loopDt = micros() - loopT0;
  if (loopDt > profLoopUsMax) profLoopUsMax = loopDt;
  // #endregion

  // NOTE: Serial heartbeat/spike logs were useful for diagnosis but can make WebUI sluggish.
  // They are now disabled by default (use DEBUG_SERIAL_LOGS=1 if needed again).
}

// Add this handler function near the top-level functions (after your includes and before setup/loop):
void handleRoot() {
  if (standbyState) {
    webHeadersNoCacheClose(server);
    server.sendHeader("Expires", "0");
    static const char ROOT_STANDBY_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang='en'>
<head>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>FloAmp - Standby</title>
  <style>
    body { font-family: Arial, sans-serif; background: #222; color: #eee; margin: 0; padding: 0; }
    .container { max-width: 420px; margin: 0 auto; padding: 24px 16px; text-align: center; }
    h1 { font-size: 2em; margin-bottom: 8px; letter-spacing: 2px; }
    .power-btn { background: #27ae60; color: #fff; font-size: 1.8em; padding: 24px 0; border: none; border-radius: 16px; width: 100%; margin: 24px 0; box-shadow: 0 4px 24px #0008; transition: background 0.2s; }
    .power-btn:active { background: #219150; }
    .info { text-align: left; background: #333; border-radius: 12px; padding: 16px; margin: 16px 0; font-size: 0.95em; }
    .info h2 { margin: 0 0 12px 0; font-size: 1.1em; color: #f1c40f; }
    .info-row { display: flex; justify-content: space-between; margin: 6px 0; }
    .info-label { color: #aaa; }
    .i2c-ok { color: #2ecc71; }
    .i2c-fail { color: #e74c3c; }
    .wifi-strong { color: #2ecc71; }
    .wifi-weak { color: #e67e22; }
    .wifi-none { color: #e74c3c; }
    #systemInfo { min-height: 120px; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>FloAmp</h1>
    <p style='color:#888; margin:0;'>Standby</p>
    <div id='systemInfo' class='info'>
      <h2>System info</h2>
      <div id='infoContent'>Loading...</div>
    </div>
    <button id='powerBtn' class='power-btn'>Power ON</button>
  </div>
  <script>
    function wifiStrength(rssi) {
      if (rssi <= -127) return { text: 'Disconnected', cls: 'wifi-none' };
      if (rssi >= -50) return { text: rssi + ' dBm (Excellent)', cls: 'wifi-strong' };
      if (rssi >= -60) return { text: rssi + ' dBm (Good)', cls: 'wifi-strong' };
      if (rssi >= -70) return { text: rssi + ' dBm (Fair)', cls: 'wifi-weak' };
      return { text: rssi + ' dBm (Weak)', cls: 'wifi-weak' };
    }
    function formatUptime(sec) {
      var h = Math.floor(sec / 3600), m = Math.floor((sec % 3600) / 60), s = sec % 60;
      if (h > 0) return h + 'h ' + m + 'm';
      if (m > 0) return m + 'm ' + s + 's';
      return s + 's';
    }
    function renderInfo(d) {
      var w = wifiStrength(d.wifiRssi);
      var i2c = d.i2c || {};
      var html = '';
      html += "<div class='info-row'><span class='info-label'>IP</span><span>" + (d.ip || '-') + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>Wi-Fi</span><span class='" + w.cls + "'>" + (d.wifiSsid ? d.wifiSsid + " | " : "") + w.text + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>Uptime</span><span>" + formatUptime(d.uptimeSec || 0) + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>Free heap</span><span>" + (d.freeHeap != null ? (d.freeHeap + " B") : "-") + "</span></div>";
      html += "<div class='info-row' style='margin-top:10px;'><span class='info-label'>I2C</span><span></span></div>";
      html += "<div class='info-row'><span class='info-label'>TDA7439</span><span class='" + (i2c.TDA7439 ? 'i2c-ok' : 'i2c-fail') + "'>" + (i2c.TDA7439 ? "OK" : "-") + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>RTC (DS3231)</span><span class='" + (i2c.RTC_DS3231 ? 'i2c-ok' : 'i2c-fail') + "'>" + (i2c.RTC_DS3231 ? "OK" : "-") + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>MCP23017</span><span class='" + (i2c.MCP23017 ? 'i2c-ok' : 'i2c-fail') + "'>" + (i2c.MCP23017 ? ("OK" + (i2c.mcpAddr && i2c.mcpAddr !== "-" ? " " + i2c.mcpAddr : "")) : "-") + "</span></div>";
      html += "<div class='info-row'><span class='info-label'>SI4703</span><span class='" + (i2c.SI4703 ? 'i2c-ok' : 'i2c-fail') + "'>" + (i2c.SI4703 ? "OK" : "-") + "</span></div>";
      return html;
    }
    function fetchWithTimeout(url, options, timeoutMs) {
      var ms = (timeoutMs !== undefined && timeoutMs !== null) ? timeoutMs : 15000;
      var controller = new AbortController();
      var tid = setTimeout(function() { controller.abort(); }, ms);
      var opts = options ? Object.assign({}, options) : {};
      opts.signal = controller.signal;
      return fetch(url, opts).finally(function() { clearTimeout(tid); });
    }
    function refreshInfo() {
      fetchWithTimeout('/systemInfo', { cache: 'no-store', credentials: 'same-origin' }, 15000)
        .then(function(r) { return r.json(); })
        .then(function(d) {
          document.getElementById('infoContent').innerHTML = renderInfo(d);
        })
        .catch(function() {
          document.getElementById('infoContent').innerHTML = "<span class='i2c-fail'>Failed to load system info.</span>";
        });
    }
    refreshInfo();
    setInterval(refreshInfo, 8000);
    document.getElementById('powerBtn').onclick = function() {
      fetchWithTimeout('/api/v2/power?state=on', { method: 'POST', cache: 'no-store', credentials: 'same-origin' }, 20000)
        .then(function() { location.reload(); })
        .catch(function(err) { alert('Error powering on: ' + err); });
    };
  </script>
</body>
</html>
)rawliteral";
    webSendProgmemChunked(server, "text/html", ROOT_STANDBY_HTML);
    return;
  }
  webHeadersNoCacheClose(server);
  server.sendHeader("Expires", "0");
  static const char ROOT_MAIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang='en'>
<head>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>FloAmp</title>
  <style>
    body { font-family: Arial, sans-serif; background: #222; color: #eee; margin: 0; padding: 0; }
    .container { max-width: 420px; margin: 0 auto; padding: 16px; }
    h1 { text-align: center; margin-bottom: 8px; }
    .status { text-align: center; margin-bottom: 16px; font-size: 1.1em; }
    .row { display: flex; align-items: center; margin: 12px 0; }
    .row label { flex: 1; }
    .row input[type=range] { flex: 2; }
    .row select, .row input[type=number] { flex: 2; font-size: 1em; }
    .btn, button { width: 100%; padding: 16px; font-size: 1.2em; border: none; border-radius: 8px; margin: 8px 0; transition: background 0.2s; }
    .power-on { background: #27ae60; color: #fff; }
    .power-off { background: #c0392b; color: #fff; }
    .mute-on { background: #e67e22; color: #fff; }
    .mute-off { background: #2980b9; color: #fff; }
    .seek-btn { width: 48%; margin: 1%; font-size: 1em; padding: 10px; }
    /* Transport buttons (RPi + PCD) */
    .transport-prevnext { background: #4fc3f7; color: #00263a; }
    .transport-play { color: #fff; }
    .transport-play.playing { background: #27ae60; }
    .transport-play.paused { background: #c0392b; }
    /* Icon-only transport buttons (SVG so no missing glyph boxes) */
    .transport-icon { display: inline-flex; align-items: center; justify-content: center; }
    .transport-svg { width: 2.025em; height: 2.025em; fill: currentColor; display: block; }
    .seek-btn:disabled { opacity: 0.45; filter: grayscale(35%); cursor: not-allowed; }
    .offline-tag { color: #e74c3c; font-weight: 700; }
    .eq-label { width: 60px; display: inline-block; }
    /* Fancy volume slider */
    .volume-row { flex-direction: column; align-items: stretch; margin: 18px 0; }
    .volume-label { text-align: center; font-size: 1.2em; margin-bottom: 6px; }
    .volume-slider {
      -webkit-appearance: none;
      width: 100%;
      height: 32px;
      background: linear-gradient(90deg, #27ae60 0%, #f1c40f 50%, #c0392b 100%);
      outline: none;
      border-radius: 16px;
      margin-bottom: 6px;
      box-shadow: 0 2px 8px #0008;
    }
    .volume-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 36px;
      height: 36px;
      border-radius: 50%;
      background: #fff;
      border: 3px solid #27ae60;
      box-shadow: 0 2px 8px #0008;
      cursor: pointer;
      transition: border 0.2s;
    }
    .volume-slider:active::-webkit-slider-thumb {
      border: 3px solid #c0392b;
    }
    .volume-slider::-moz-range-thumb {
      width: 36px;
      height: 36px;
      border-radius: 50%;
      background: #fff;
      border: 3px solid #27ae60;
      box-shadow: 0 2px 8px #0008;
      cursor: pointer;
      transition: border 0.2s;
    }
    .volume-slider:active::-moz-range-thumb {
      border: 3px solid #c0392b;
    }
    .volume-value {
      text-align: center;
      font-size: 1.5em;
      font-weight: bold;
      color: #f1c40f;
      margin-bottom: 8px;
    }
    /* Source buttons */
    .source-row { display: flex; justify-content: space-between; margin: 18px 0; }
    .source-btn {
      flex: 1;
      margin: 0 4px;
      padding: 18px 0;
      font-size: 1.1em;
      border: none;
      border-radius: 10px;
      color: #fff;
      background: #444;
      transition: background 0.2s, box-shadow 0.2s;
      box-shadow: 0 2px 8px #0006;
      cursor: pointer;
    }
    .source-btn.selected { box-shadow: 0 0 0 3px #f1c40f; background: #27ae60; color: #fff; }
    .source-btn.tun { background: #2980b9; }
    .source-btn.tun.selected { background: #2980b9; }
    .source-btn.bt { background: #8e44ad; }
    .source-btn.bt.selected { background: #8e44ad; }
    .source-btn.rpi { background: #e67e22; }
    .source-btn.rpi.selected { background: #e67e22; }
    .source-btn.pc { background: #c0392b; }
    .source-btn.pc.selected { background: #c0392b; }
    @media (max-width: 500px) {
      .container { padding: 4px; }
      .btn, button { padding: 12px; font-size: 1em; }
      .row label { font-size: 0.95em; }
      .source-btn { font-size: 0.95em; padding: 12px 0; }
      .volume-slider { height: 24px; }
      .volume-slider::-webkit-slider-thumb, .volume-slider::-moz-range-thumb { width: 28px; height: 28px; }
    }
  </style>
</head>
<body>
  <div class='container'>
    <h1>FloAmp</h1>
    <div class='status' id='status'></div>
    <button id='powerBtn' class='btn'>Power</button>
    <button id='muteBtn' class='btn'>Mute</button>
    <div class='source-row'>
      <button class='source-btn pc' id='src4'>PCA</button>
      <button class='source-btn pc' id='src5'>PCD</button>
      <button class='source-btn rpi' id='src3'>Rpi</button>
      <button class='source-btn bt' id='src2'>BT</button>
      <button class='source-btn tun' id='src1'>TUN</button>
    </div>
    <div class='row' id='freqRow' style='display:none;'>
      <label for='freq'>FM Freq (MHz)</label>
      <input type='number' min='87.5' max='108.0' step='0.1' id='freq'>
    </div>
    <div class='row' id='rdsRow' style='display:none;'>
      <label>RDS</label>
      <div id='rdsVal' style='white-space: nowrap; overflow: hidden; text-overflow: ellipsis; color: #f1c40f; font-size: 1.2em; font-weight: 600;'>-</div>
    </div>
    <div class='row' id='presetRow' style='display:none; flex-wrap: wrap; justify-content: space-between;'></div>
    <div class='row' id='seekRow' style='display:none;'>
      <button class='seek-btn' onclick='seek("down")'>&lt;&lt; Seek Down</button>
      <button class='seek-btn' onclick='seek("up")'>Seek Up &gt;&gt;</button>
    </div>
    <div class='row' id='rpiMetaRow' style='display:none; flex-direction:column; align-items:flex-start; gap:4px;'>
      <div><b>RPi:</b> <span id='rpiStateVal'>-</span> | <span id='rpiOnlineVal'>offline</span></div>
      <div><b>Title:</b> <span id='rpiTitleVal'>-</span></div>
      <div><b>Artist:</b> <span id='rpiArtistVal'>-</span></div>
      <div><b>File:</b> <span id='rpiFileVal'>-</span></div>
    </div>
    <div class='row' id='pcMetaRow' style='display:none; flex-direction:column; align-items:flex-start; gap:4px;'>
      <div><b>Foobar:</b> <span id='pcStateVal'>-</span> | <span id='pcOnlineVal'>offline</span></div>
      <div><b>Title:</b> <span id='pcTitleVal'>-</span></div>
      <div><b>Artist:</b> <span id='pcArtistVal'>-</span></div>
      <div><b>File:</b> <span id='pcExtraVal'>-</span></div>
    </div>
    <div class='row' id='pcCtrlRow' style='display:none; gap:8px;'>
      <button class='seek-btn transport-prevnext' id='pcPrevBtn' aria-label='Previous' title='Previous'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M7 6h2v12H7V6zm3.5 6 9 6V6l-9 6z'/></svg></span></button>
      <button class='seek-btn transport-play paused' id='pcPlayBtn' aria-label='Play/Pause' title='Play/Pause'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M8 5v14l11-7L8 5z'/></svg></span></button>
      <button class='seek-btn transport-prevnext' id='pcNextBtn' aria-label='Next' title='Next'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M15 6h2v12h-2V6zM4.5 18l9-6-9-6v12z'/></svg></span></button>
    </div>
    <div class='row' id='rpiCtrlRow' style='display:none; gap:8px;'>
      <button class='seek-btn transport-prevnext' id='rpiPrevBtn' aria-label='Previous' title='Previous'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M7 6h2v12H7V6zm3.5 6 9 6V6l-9 6z'/></svg></span></button>
      <button class='seek-btn transport-play paused' id='rpiPlayBtn' aria-label='Play/Pause' title='Play/Pause'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M8 5v14l11-7L8 5z'/></svg></span></button>
      <button class='seek-btn transport-prevnext' id='rpiNextBtn' aria-label='Next' title='Next'><span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M15 6h2v12h-2V6zM4.5 18l9-6-9-6v12z'/></svg></span></button>
    </div>
    <div class='volume-row'>
      <div class='volume-label'>Volume</div>
      <input type='range' min='0' max='48' id='volume' class='volume-slider'>
      <div class='volume-value' id='volumeVal'></div>
    </div>
    <div style='margin:16px 0 8px 0; text-align:center;'>
      <b>Equalizer</b>
    </div>
    <div class='row'><span class='eq-label'>Bass</span><input type='range' min='-7' max='7' id='bass'><span id='bassVal'></span></div>
    <div class='row'><span class='eq-label'>Middle</span><input type='range' min='-7' max='7' id='middle'><span id='middleVal'></span></div>
    <div class='row'><span class='eq-label'>Treble</span><input type='range' min='-7' max='7' id='treble'><span id='trebleVal'></span></div>
    <div class='row'><span class='eq-label'>Gain</span><input type='range' min='-45' max='0' id='gain'><span id='gainVal'></span></div>
    <div class='row'><span class='eq-label'>Balance</span><input type='range' min='-15' max='15' id='balance'><span id='balanceVal'></span></div>
    <div class='row' id='rssiRow' style='display:none;'>
      <label>Signal</label>
      <div style='display:flex; gap:12px; align-items:center;'>
        <div id='rssiVal'>-</div>
        <div id='wifiVal' style='opacity:0.8; font-size:0.95em;'>WiFi: - dBm</div>
      </div>
    </div>
  </div>
  <script>
    var state = {};
    var presetPressTimer = null;
    var isLongPress = false;

    /** Două cozi: GET (/status, /presets) nu stau în spatele POST-urilor /rpi — evită UI înghețat la transport. */
    var readTail = Promise.resolve();
    var cmdTail = Promise.resolve();
    function readThen(task) {
      var p = readTail.then(function() { return task(); }, function() { return task(); });
      readTail = p.catch(function() {});
      return p;
    }
    function cmdThen(task) {
      var p = cmdTail.then(function() { return task(); }, function() { return task(); });
      cmdTail = p.catch(function() {});
      return p;
    }

    function delay(ms) {
      return new Promise(function(resolve) { setTimeout(resolve, ms); });
    }

    function fetchWithTimeout(url, options, timeoutMs) {
      var ms = (timeoutMs !== undefined && timeoutMs !== null) ? timeoutMs : 10000;
      var controller = new AbortController();
      var tid = setTimeout(function() { controller.abort(); }, ms);
      var opts = options ? Object.assign({}, options) : {};
      opts.signal = controller.signal;
      return fetch(url, opts).finally(function() { clearTimeout(tid); });
    }

    /** GET JSON fără coadă readThen — folosit rar (ex. preseturi imediat după TUN) ca să nu stea după zeci de /state blocate. */
    function httpGetJsonBare(url, timeoutMs) {
      var u = url + (url.indexOf('?') >= 0 ? '&' : '?') + 't=' + Date.now();
      var to = (timeoutMs !== undefined && timeoutMs !== null) ? timeoutMs : 9000;
      return fetchWithTimeout(u, {
        cache: 'no-store',
        credentials: 'same-origin',
        headers: { 'Accept': 'application/json' }
      }, to).then(function(r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      });
    }

    function httpGetJson(url) {
      return readThen(function() {
        return httpGetJsonBare(url, 9000);
      });
    }

    function normalizeV2(s) {
      // Convert v2 API shape into the legacy `state` fields the UI code expects.
      if (!s || typeof s !== 'object') return {};
      var out = {};
      out.power = (s.power ? 'on' : 'off');
      out.mute = (s.mute ? 'on' : 'off');
      out.volume = (typeof s.volume === 'number') ? s.volume : 0;
      out.source = (typeof s.source === 'number') ? s.source : 1;
      out.wifi = (typeof s.wifiRssi === 'number') ? s.wifiRssi : -127;

      var t = s.tuner || {};
      out.freq = (typeof t.freq === 'number') ? t.freq : 0;
      out.rssi = (typeof t.rssi === 'number') ? t.rssi : -1;
      out.rds = (typeof t.rds === 'string') ? t.rds : '';

      var r = s.rpi || {};
      out.rpiOnline = !!r.online;
      out.rpiState = r.state || '';
      out.rpiTitle = r.title || '';
      out.rpiArtist = r.artist || '';
      out.rpiFile = r.file || '';

      var p = s.pc || {};
      out.pcOnline = !!p.online;
      out.pcHttpCode = (typeof p.httpCode === 'number') ? p.httpCode : 0;
      out.pcState = p.state || '';
      out.pcTitle = p.title || '';
      out.pcArtist = p.artist || '';
      out.pcExtra = p.extra || '';

      var eq = s.eq || {};
      out.bass = (typeof eq.bass === 'number') ? eq.bass : 0;
      out.middle = (typeof eq.middle === 'number') ? eq.middle : 0;
      out.treble = (typeof eq.treble === 'number') ? eq.treble : 0;
      out.gain = (typeof eq.gain === 'number') ? eq.gain : 0;
      out.balance = (typeof eq.balance === 'number') ? eq.balance : 0;
      return out;
    }

    var stateRefreshBusy = false;
    var stateRefreshAgain = false;

    function refreshState() {
      if (stateRefreshBusy) {
        stateRefreshAgain = true;
        return Promise.resolve();
      }
      stateRefreshBusy = true;
      return httpGetJson('/api/v2/state')
        .then(function(s) {
          state = normalizeV2(s);
          updateUI();
          if (state.source === 1 && !presetsLoaded) return fetchPresets(false);
        })
        .catch(function(err) { console.error('refreshState:', err); })
        .finally(function() {
          stateRefreshBusy = false;
          if (stateRefreshAgain) {
            stateRefreshAgain = false;
            refreshState();
          }
        });
    }

    function applyStatusSnapshot(s) {
      if (!s || typeof s !== 'object') return;
      var snap = normalizeV2(s);
      state = Object.assign({}, state, snap);
      if (state.source === 1 && !presetsLoaded) fetchPresets(true);
      updateUI();
    }

    /** După POST: dacă endpoint-ul întoarce JSON status, îl aplicăm imediat. */
    function kickSync(resp) {
      if (resp && typeof resp === 'object') {
        applyStatusSnapshot(resp);
        return Promise.resolve(resp);
      }
      return refreshState();
    }

    function postCmd(url) {
      return cmdThen(function() {
        return fetchWithTimeout(url, { method: 'POST', cache: 'no-store', credentials: 'same-origin' }, 7000)
          .then(function(r) {
            if (!r.ok) throw new Error('HTTP ' + r.status);
            var ct = (r.headers.get('content-type') || '').toLowerCase();
            if (ct.indexOf('application/json') >= 0) return r.json();
            return r.text();
          });
      });
    }

    /** După schimbare sursă / power: un singur refresh întârziat (fără dublu GET). */
    function kickSyncSlow() {
      return delay(400).then(function() { return refreshState(); });
    }

    var transportRefreshTimer = null;
    function isFullStatusPayload(obj) {
      return obj && typeof obj === 'object' && ('power' in obj || 'volume' in obj || 'source' in obj);
    }
    function kickSyncTransport(resp) {
      if (isFullStatusPayload(resp)) applyStatusSnapshot(resp);
      if (transportRefreshTimer) clearTimeout(transportRefreshTimer);
      transportRefreshTimer = setTimeout(function() {
        transportRefreshTimer = null;
        refreshState();
      }, 750);
      return Promise.resolve();
    }

    var eqSyncTimer = null;
    var eqPostInFlight = false;
    var volumePending = false;

    function buildEqQuery() {
      const b = document.getElementById('bass').value;
      const m = document.getElementById('middle').value;
      const t = document.getElementById('treble').value;
      const g = document.getElementById('gain').value;
      const bal = document.getElementById('balance').value;
      return 'bass=' + encodeURIComponent(b) + '&middle=' + encodeURIComponent(m) + '&treble=' + encodeURIComponent(t)
        + '&gain=' + encodeURIComponent(g) + '&balance=' + encodeURIComponent(bal);
    }

    function scheduleEqSync() {
      clearTimeout(eqSyncTimer);
      eqSyncTimer = setTimeout(function() {
        eqSyncTimer = null;
        eqPostInFlight = true;
        postCmd('/api/v2/eq?' + buildEqQuery())
          .then(function() { return kickSync(); })
          .catch(function(err) { console.error('Error setting EQ:', err); })
          .finally(function() { eqPostInFlight = false; });
      }, 200);
    }
    function onEqSliderInput(valElId) {
      return function(e) {
        document.getElementById(valElId).textContent = e.target.value;
        scheduleEqSync();
      };
    }
    let presets = [];
    let presetsLoaded = false;
    let lastRenderedFreq = null;
    let lastPresetClick = 0;
    let lastPresetId = -1;
    
    // Add preset handling functions
    function storePreset(id, name) {
      console.log('Storing preset', id, 'with frequency', state.freq);
      const params = new URLSearchParams({
        id: id + 1,
        freq: state.freq,
        name: name || `Preset ${id + 1}`
      });
      
      postCmd('/preset?' + params.toString())
        .then(function() {
          console.log('Preset stored successfully');
          return fetchPresets(true);
        })
        .then(function() { return kickSync(); })
        .catch(err => console.error('Error storing preset:', err));
    }

    function recallPreset(id, preset) {
      console.log('Recalling preset', id, 'with frequency', preset.freq);
      if (preset.freq > 0) {
        // Update the UI immediately to show the change
        document.getElementById('freq').value = preset.freq;
        state.freq = preset.freq; // Update local state
        
        // Make the API call with retry logic
        let retryCount = 0;
        const maxRetries = 3;
        
        function attemptRecall() {
          postCmd('/freq?value=' + preset.freq)
            .then(function() {
              console.log('Preset recalled successfully');
              return kickSync();
            })
            .catch(err => {
              console.error('Error recalling preset:', err);
              if (retryCount < maxRetries) {
                retryCount++;
                console.log(`Retrying preset recall (attempt ${retryCount})...`);
                setTimeout(attemptRecall, 1000); // Wait 1 second before retrying
              } else {
                console.error('Failed to recall preset after', maxRetries, 'attempts');
                // Show error to user
                const statusDiv = document.getElementById('status');
                statusDiv.textContent = 'Error: Could not set frequency. Please try again.';
                statusDiv.style.color = '#ff4444';
                setTimeout(() => {
                  statusDiv.textContent = '';
                  refreshState();
                }, 3000);
              }
            });
        }
        
        attemptRecall();
      }
    }

    function handlePresetClick(id, preset, event) {
      event.preventDefault(); // Prevent default behavior
      const now = Date.now();
      
      // Handle touch events
      if (event.type === 'touchstart') {
        isLongPress = false;
        presetPressTimer = setTimeout(() => {
          isLongPress = true;
          console.log('Long press detected - storing preset');
          storePreset(id);
        }, 500); // 500ms for long press
        return;
      }
      
      // Handle touch end
      if (event.type === 'touchend') {
        clearTimeout(presetPressTimer);
        if (!isLongPress) {
          // Normal tap
          if (preset.freq > 0) {
            recallPreset(id, preset);
          }
        }
        return;
      }
      
      // Check for right click
      if (event.type === 'contextmenu') {
        console.log('Right click detected - storing preset');
        storePreset(id);
        return;
      }
      
      // Check for double click
      if (id === lastPresetId && now - lastPresetClick < 500) {
        console.log('Double click detected - renaming preset');
        const newName = prompt('Preset name:', preset.name || `Preset ${id + 1}`);
        if (newName !== null) {
          storePreset(id, newName);
        }
        return;
      }
      
      // Single click - recall preset
      console.log('Single click detected - recalling preset');
      if (preset.freq > 0) {
        recallPreset(id, preset);
      }
      
      lastPresetClick = now;
      lastPresetId = id;
    }

    function renderPresets() {
      const row = document.getElementById('presetRow');
      row.innerHTML = '';
      presets.forEach((p, i) => {
        const btn = document.createElement('button');
        btn.className = 'btn';
        btn.style.flex = '1 0 30%';
        btn.style.margin = '4px';
        btn.style.minWidth = '70px';
        btn.style.padding = '8px 4px';
        btn.style.display = 'flex';
        btn.style.flexDirection = 'column';
        btn.style.alignItems = 'center';
        btn.style.justifyContent = 'center';
        btn.style.gap = '4px';
        
        // Create name span
        const nameSpan = document.createElement('span');
        nameSpan.textContent = p.name || `Preset ${i+1}`;
        nameSpan.style.fontSize = '14px';
        nameSpan.style.fontWeight = 'bold';
        
        // Create frequency span
        const freqSpan = document.createElement('span');
        freqSpan.style.fontSize = '12px';
        freqSpan.style.opacity = '0.8';
        
        // Set button color and frequency text based on state
        if (p.freq > 0) {
          if (Math.abs(p.freq - state.freq) < 0.1) {
            btn.style.background = '#f1c40f';
            btn.style.color = '#000';
            freqSpan.textContent = `${p.freq.toFixed(1)} MHz`;
            freqSpan.style.fontWeight = 'bold';
          } else {
            btn.style.background = '#27ae60';
            btn.style.color = '#fff';
            freqSpan.textContent = `${p.freq.toFixed(1)} MHz`;
          }
        } else {
          btn.style.background = '#444';
          btn.style.color = '#fff';
          freqSpan.textContent = 'Empty';
          freqSpan.style.fontStyle = 'italic';
        }
        
        // Add hover effect
        btn.style.transition = 'all 0.2s ease';
        btn.onmouseover = function() {
          this.style.transform = 'scale(1.05)';
          this.style.boxShadow = '0 0 10px rgba(0,0,0,0.3)';
        };
        btn.onmouseout = function() {
          this.style.transform = 'scale(1)';
          this.style.boxShadow = 'none';
        };
        
        // Add event listeners
        btn.addEventListener('click', (e) => handlePresetClick(i, p, e));
        btn.addEventListener('contextmenu', (e) => handlePresetClick(i, p, e));
        btn.addEventListener('touchstart', (e) => handlePresetClick(i, p, e));
        btn.addEventListener('touchend', (e) => handlePresetClick(i, p, e));
        
        // Add spans to button
        btn.appendChild(nameSpan);
        btn.appendChild(freqSpan);
        
        row.appendChild(btn);
      });
    }

    function fetchPresets(urgent) {
      var run = function() {
        return httpGetJsonBare('/presets', 8000)
          .then(function(arr) {
            presets = arr;
            presetsLoaded = true;
            renderPresets();
          })
          .catch(function(err) { console.error('Error fetching presets:', err); });
      };
      if (urgent) return run();
      return readThen(run);
    }

    function updateUI() {
      // Update power button
      const powerBtn = document.getElementById('powerBtn');
      powerBtn.className = 'btn ' + (state.power === 'on' ? 'power-on' : 'power-off');
      powerBtn.textContent = state.power === 'on' ? 'Power OFF' : 'Power ON';

      // Update mute button
      const muteBtn = document.getElementById('muteBtn');
      muteBtn.className = 'btn ' + (state.mute === 'on' ? 'mute-on' : 'mute-off');
      muteBtn.textContent = state.mute === 'on' ? 'Unmute' : 'Mute';

      // Update volume (nu suprascrie în timpul tragerii / până vine răspunsul de la POST)
      const volumeSlider = document.getElementById('volume');
      const volumeVal = document.getElementById('volumeVal');
      if (!volumePending && document.activeElement !== volumeSlider) {
        volumeSlider.value = state.volume;
        volumeVal.textContent = state.volume;
      }

      // Update source buttons
      document.querySelectorAll('.source-btn').forEach(btn => {
        btn.classList.remove('selected');
      });
      const selectedSource = document.getElementById('src' + state.source);
      if (selectedSource) selectedSource.classList.add('selected');

      // Show/hide frequency controls based on source
      const freqRow = document.getElementById('freqRow');
      const rssiRow = document.getElementById('rssiRow');
      const rdsRow = document.getElementById('rdsRow');
      const seekRow = document.getElementById('seekRow');
      const presetRow = document.getElementById('presetRow');
      const rpiMetaRow = document.getElementById('rpiMetaRow');
      const rpiCtrlRow = document.getElementById('rpiCtrlRow');
      const pcMetaRow = document.getElementById('pcMetaRow');
      const pcCtrlRow = document.getElementById('pcCtrlRow');
      const rpiTransportBtns = [
        document.getElementById('rpiPrevBtn'),
        document.getElementById('rpiPlayBtn'),
        document.getElementById('rpiNextBtn')
      ];
      const pcTransportBtns = [
        document.getElementById('pcPrevBtn'),
        document.getElementById('pcPlayBtn'),
        document.getElementById('pcNextBtn')
      ];
      function setTransportEnabled(btns, enabled) {
        btns.forEach(function(b) { if (b) b.disabled = !enabled; });
      }
      function normalizePlaybackState(rawState) {
        const s = (rawState || '').toLowerCase();
        if (s === 'playing') return 'play';
        if (s === 'paused') return 'pause';
        if (s === 'stopped') return 'stop';
        return s;
      }
      function applyTransportPanel(cfg) {
        function svgPlay() {
          return "<span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M8 5v14l11-7L8 5z'/></svg></span>";
        }
        function svgPause() {
          return "<span class='transport-icon'><svg class='transport-svg' viewBox='0 0 24 24' aria-hidden='true'><path d='M6 5h4v14H6V5zm8 0h4v14h-4V5z'/></svg></span>";
        }
        if (!cfg.online) {
          cfg.onlineEl.textContent = 'Offline';
          cfg.onlineEl.className = 'offline-tag';
          cfg.onlineEl.style.display = 'inline';
          cfg.stateEl.textContent = 'Offline';
          cfg.titleEl.textContent = 'Offline';
          cfg.artistEl.textContent = 'Offline';
          cfg.extraEl.textContent = 'Offline';
          cfg.playBtn.innerHTML = svgPlay();
          cfg.playBtn.classList.remove('playing');
          cfg.playBtn.classList.add('paused');
          setTransportEnabled(cfg.btns, false);
          return;
        }
        cfg.onlineEl.textContent = '';
        cfg.onlineEl.className = '';
        cfg.onlineEl.style.display = 'none';
        cfg.stateEl.textContent = cfg.stateValue || '-';
        cfg.titleEl.textContent = cfg.titleValue || '-';
        cfg.artistEl.textContent = cfg.artistValue || '-';
        cfg.extraEl.textContent = cfg.extraValue || '-';
        const norm = normalizePlaybackState(cfg.stateValue);
        const isPlaying = (norm === 'play');
        cfg.playBtn.innerHTML = isPlaying ? svgPause() : svgPlay();
        cfg.playBtn.classList.toggle('playing', isPlaying);
        cfg.playBtn.classList.toggle('paused', !isPlaying);
        setTransportEnabled(cfg.btns, true);
      }
      if (state.source === 1) { // TUN
        freqRow.style.display = 'flex';
        rssiRow.style.display = 'flex';
        rdsRow.style.display = 'flex';
        seekRow.style.display = 'flex';
        presetRow.style.display = 'flex';
        rpiMetaRow.style.display = 'none';
        rpiCtrlRow.style.display = 'none';
        pcMetaRow.style.display = 'none';
        pcCtrlRow.style.display = 'none';
        setTransportEnabled(rpiTransportBtns, false);
        setTransportEnabled(pcTransportBtns, false);
        
        // Only update frequency input if it's not focused
        const freqInput = document.getElementById('freq');
        if (document.activeElement !== freqInput) {
          freqInput.value = state.freq;
        }
        // Update RSSI value (simple numeric; could be turned into bars)
        const rssiEl = document.getElementById('rssiVal');
        if (typeof state.rssi === 'number' && state.rssi >= 0) {
          rssiEl.textContent = 'FM: ' + state.rssi.toString();
          rssiEl.style.opacity = '1';
          // Color code FM RSSI (SI4703 scale 0..127). Tune thresholds as needed.
          let fmColor = '#e74c3c'; // red (bad)
          if (state.rssi >= 25) fmColor = '#27ae60'; // green (good)
          else if (state.rssi >= 12) fmColor = '#f1c40f'; // yellow (medium)
          rssiEl.style.color = fmColor;
        } else {
          rssiEl.textContent = 'FM: -';
          rssiEl.style.opacity = '0.6';
          rssiEl.style.color = '#bbb';
        }
        // Update WiFi RSSI (dBm)
        const wifiEl = document.getElementById('wifiVal');
        if (typeof state.wifi === 'number' && state.wifi > -127) {
          wifiEl.textContent = 'WiFi: ' + state.wifi.toString() + ' dBm';
          wifiEl.style.opacity = '1';
          // Color code WiFi RSSI (dBm). Typical: >= -60 good, -75..-60 medium, < -75 bad.
          let wifiColor = '#e74c3c'; // red
          if (state.wifi >= -60) wifiColor = '#27ae60'; // green
          else if (state.wifi >= -75) wifiColor = '#f1c40f'; // yellow
          wifiEl.style.color = wifiColor;
        } else {
          wifiEl.textContent = 'WiFi: - dBm';
          wifiEl.style.opacity = '0.6';
          wifiEl.style.color = '#bbb';
        }
        // Update RDS text below frequency
        const rdsEl = document.getElementById('rdsVal');
        if (state.rds && state.rds.length > 0 && state.rds.trim() !== '') {
          rdsEl.textContent = state.rds;
          rdsEl.style.opacity = '1';
        } else {
          rdsEl.textContent = '-';
          rdsEl.style.opacity = '0.6';
        }
      } else {
        freqRow.style.display = 'none';
        // Show WiFi signal on all sources
        rssiRow.style.display = 'flex';
        rdsRow.style.display = 'none';
        seekRow.style.display = 'none';
        presetRow.style.display = 'none';
        // FM not applicable for non-TUN
        const rssiEl = document.getElementById('rssiVal');
        rssiEl.textContent = 'FM: -';
        rssiEl.style.opacity = '0.6';
        rssiEl.style.color = '#bbb';
        if (state.source === 3) { // RPi
          rpiMetaRow.style.display = 'flex';
          rpiCtrlRow.style.display = 'flex';
          pcMetaRow.style.display = 'none';
          pcCtrlRow.style.display = 'none';
        applyTransportPanel({
          online: !!state.rpiOnline,
          onlineEl: document.getElementById('rpiOnlineVal'),
          stateEl: document.getElementById('rpiStateVal'),
          titleEl: document.getElementById('rpiTitleVal'),
          artistEl: document.getElementById('rpiArtistVal'),
          extraEl: document.getElementById('rpiFileVal'),
          stateValue: state.rpiState,
          titleValue: state.rpiTitle,
          artistValue: state.rpiArtist,
          extraValue: state.rpiFile,
          playBtn: document.getElementById('rpiPlayBtn'),
          btns: rpiTransportBtns
        });
          setTransportEnabled(pcTransportBtns, false);
        } else if (state.source === 5) { // PCD (foobar on RPi input)
          rpiMetaRow.style.display = 'none';
          rpiCtrlRow.style.display = 'none';
          pcMetaRow.style.display = 'flex';
          pcCtrlRow.style.display = 'flex';
        applyTransportPanel({
          online: !!state.pcOnline,
          onlineEl: document.getElementById('pcOnlineVal'),
          stateEl: document.getElementById('pcStateVal'),
          titleEl: document.getElementById('pcTitleVal'),
          artistEl: document.getElementById('pcArtistVal'),
          extraEl: document.getElementById('pcExtraVal'),
          stateValue: state.pcState,
          titleValue: state.pcTitle,
          artistValue: state.pcArtist,
          extraValue: state.pcExtra,
          playBtn: document.getElementById('pcPlayBtn'),
          btns: pcTransportBtns
        });
          setTransportEnabled(rpiTransportBtns, false);
        } else {
          rpiMetaRow.style.display = 'none';
          rpiCtrlRow.style.display = 'none';
          pcMetaRow.style.display = 'none';
          pcCtrlRow.style.display = 'none';
          setTransportEnabled(rpiTransportBtns, false);
          setTransportEnabled(pcTransportBtns, false);
        }
        // Update WiFi RSSI (dBm)
        const wifiEl = document.getElementById('wifiVal');
        if (typeof state.wifi === 'number' && state.wifi > -127) {
          wifiEl.textContent = 'WiFi: ' + state.wifi.toString() + ' dBm';
          wifiEl.style.opacity = '1';
          // Color code WiFi RSSI (dBm). Typical: >= -60 good, -75..-60 medium, < -75 bad.
          let wifiColor = '#e74c3c'; // red
          if (state.wifi >= -60) wifiColor = '#27ae60'; // green
          else if (state.wifi >= -75) wifiColor = '#f1c40f'; // yellow
          wifiEl.style.color = wifiColor;
        } else {
          wifiEl.textContent = 'WiFi: - dBm';
          wifiEl.style.opacity = '0.6';
          wifiEl.style.color = '#bbb';
        }
      }

      // Update EQ — nu suprascrie cât e debounce sau POST în curs sau slider focalizat (evită „arată una, e alta”)
      const eqIds = ['bass', 'middle', 'treble', 'gain', 'balance'];
      const eqKeys = ['bass', 'middle', 'treble', 'gain', 'balance'];
      let eqUiLocked = (eqSyncTimer !== null) || eqPostInFlight;
      if (!eqUiLocked) {
        for (var ei = 0; ei < eqIds.length; ei++) {
          if (document.activeElement === document.getElementById(eqIds[ei])) {
            eqUiLocked = true;
            break;
          }
        }
      }
      if (!eqUiLocked) {
        for (var ej = 0; ej < eqIds.length; ej++) {
          var el = document.getElementById(eqIds[ej]);
          el.value = state[eqKeys[ej]];
          document.getElementById(eqIds[ej] + 'Val').textContent = state[eqKeys[ej]];
        }
      }
      
      // Update presets only when frequency actually changed
      if (state.source === 1 && state.freq !== lastRenderedFreq) {
        lastRenderedFreq = state.freq;
        renderPresets();
      }
    }

    // Event handlers
    document.getElementById('powerBtn').onclick = function() {
      var goingToStandby = (state.power === 'on');
      postCmd('/api/v2/power?state=' + (goingToStandby ? 'off' : 'on'))
        .then(function(resp) {
          kickSync(resp);
          if (goingToStandby) {
            setTimeout(function() { location.reload(); }, 300);
          } else {
            return kickSyncSlow();
          }
        })
        .catch(function(err) { console.error('Error toggling power:', err); });
    };

    document.getElementById('muteBtn').onclick = function() {
      postCmd('/api/v2/mute?state=' + (state.mute === 'on' ? 'off' : 'on'))
        .then(function(resp) { return kickSync(resp); })
        .catch(function(err) { console.error('Error toggling mute:', err); });
    };

    var volumeInputTimeout;
    document.getElementById('volume').addEventListener('input', function(e) {
      volumePending = true;
      document.getElementById('volumeVal').textContent = e.target.value;
      clearTimeout(volumeInputTimeout);
      var value = e.target.value;
      volumeInputTimeout = setTimeout(function() {
        postCmd('/api/v2/volume?value=' + value)
          .then(function(resp) { return kickSync(resp); })
          .then(function() { volumePending = false; })
          .catch(function(err) {
            volumePending = false;
            console.error('Error setting volume:', err);
          });
      }, 150);
    });

    document.querySelectorAll('.source-btn').forEach(function(btn) {
      btn.onclick = function(e) {
        var source = parseInt(e.target.id.replace('src', ''), 10);
        postCmd('/api/v2/source?value=' + source)
          .then(function(j) {
            if (j && typeof j === 'object') applyStatusSnapshot(j);
            return kickSyncSlow();
          })
          .catch(function(err) { console.error('Error setting source:', err); });
      };
    });

    document.getElementById('freq').onchange = function(e) {
      postCmd('/freq?value=' + e.target.value)
        .then(function() { return kickSync(); })
        .catch(function(err) { console.error('Error setting frequency:', err); });
    };

    window.seek = function(dir) {
      postCmd('/api/v2/tuner/seek?dir=' + dir)
        .then(function() { return kickSync(); })
        .catch(function(err) { console.error('Error seeking:', err); });
    };

    window.rpiCmd = function(cmd) {
      if (!state.rpiOnline) return Promise.resolve();
      postCmd('/api/v2/rpi/cmd?cmd=' + encodeURIComponent(cmd))
        .then(function(resp) { return kickSyncTransport(resp); })
        .catch(function(err) { console.error('Error sending RPI command:', err); });
    };

    window.pcCmd = function(cmd) {
      if (!state.pcOnline) return Promise.resolve();
      postCmd('/api/v2/pc/cmd?cmd=' + encodeURIComponent(cmd))
        .then(function(resp) { return kickSyncTransport(resp); })
        .catch(function(err) { console.error('Error sending PC command:', err); });
    };

    // EQ handlers (debounced + un singur POST pentru toate canalele)
    document.getElementById('bass').oninput = onEqSliderInput('bassVal');
    document.getElementById('middle').oninput = onEqSliderInput('middleVal');
    document.getElementById('treble').oninput = onEqSliderInput('trebleVal');
    document.getElementById('gain').oninput = onEqSliderInput('gainVal');
    document.getElementById('balance').oninput = onEqSliderInput('balanceVal');

    document.getElementById('rpiPrevBtn').onclick = function() { rpiCmd('PREV'); };
    document.getElementById('rpiPlayBtn').onclick = function() { rpiCmd('PLAYPAUSE'); };
    document.getElementById('rpiNextBtn').onclick = function() { rpiCmd('NEXT'); };

    document.getElementById('pcPrevBtn').onclick = function() { pcCmd('PREV'); };
    document.getElementById('pcPlayBtn').onclick = function() { pcCmd('PLAYPAUSE'); };
    document.getElementById('pcNextBtn').onclick = function() { pcCmd('NEXT'); };

    setTimeout(function() {
      refreshState();
      setInterval(function() { if (!document.hidden) refreshState(); }, 20000);
    }, 500);
    document.addEventListener('visibilitychange', function() {
      if (!document.hidden) refreshState();
    });

    var freqInputTimeout;
    document.getElementById('freq').addEventListener('input', function(e) {
      clearTimeout(freqInputTimeout);
      freqInputTimeout = setTimeout(function() {
        var value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 87.5 && value <= 108.0) {
          postCmd('/freq?value=' + value)
            .then(function() { return kickSync(); })
            .catch(function(err) { console.error('Error setting frequency:', err); });
        }
      }, 500);
    });

    document.getElementById('freq').addEventListener('blur', function(e) {
      var value = parseFloat(e.target.value);
      if (!isNaN(value) && value >= 87.5 && value <= 108.0) {
        postCmd('/freq?value=' + value)
          .then(function() { return kickSync(); })
          .catch(function(err) { console.error('Error setting frequency:', err); });
      }
    });
  </script>
</body>
</html>
)rawliteral";
  webSendProgmemChunked(server, "text/html", ROOT_MAIN_HTML);
}

// Add these web handlers near the top-level functions (after your includes and before setup/loop):
void handlePower() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      // Respond immediately, perform heavy work in main loop
      if (standbyState) {
        standbyState = false;
        powerOnPending = true;
      }
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else if (state == "off") {
      // Respond immediately, perform work in main loop
      if (!standbyState) {
        standbyState = true;
        powerOffPending = true;
      }      
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else {
      server.send(400, "text/plain", "Invalid state");
    }
  } else {
    server.send(400, "text/plain", "Missing state argument");
  }
}

void handlePowerState() {
  server.send(200, "text/plain", standbyState ? "off" : "on");
}

// --------------------------------------------------------------------------------------------------
// Web API v2 (non-blocking, per-source)
// --------------------------------------------------------------------------------------------------

static void handleApiV2State() {
  if (webStatusBusy) {
    webSendText(server, 503, "busy");
    return;
  }
  webStatusBusy = true;
  webSendStateV2(server);
  webStatusBusy = false;
}

static void handleApiV2Power() {
  if (server.method() != HTTP_POST || !server.hasArg("state")) {
    webSendBadRequest(server, "Missing state");
    return;
  }
  String state = server.arg("state");
  if (state == "on") {
    if (standbyState) {
      standbyState = false;
      powerOnPending = true;
    }
    webSendStateV2(server);
    return;
  }
  if (state == "off") {
    if (!standbyState) {
      standbyState = true;
      powerOffPending = true;
    }
    webSendStateV2(server);
    return;
  }
  webSendBadRequest(server, "Invalid state");
}

static void handleApiV2Mute() {
  if (webRejectIfStandby(server)) return;
  if (server.method() != HTTP_POST || !server.hasArg("state")) {
    webSendBadRequest(server, "Missing state");
    return;
  }
  String state = server.arg("state");
  if (state == "on") {
    if (!fmRadio.isMuted) fmRadio.toggleMute();
  } else if (state == "off") {
    if (fmRadio.isMuted) fmRadio.toggleMute();
  } else {
    webSendBadRequest(server, "Invalid state");
    return;
  }
  webSendStateV2(server);
}

static void handleApiV2Volume() {
  if (webRejectIfStandby(server)) return;
  if (server.method() != HTTP_POST || !server.hasArg("value")) {
    webSendBadRequest(server, "Missing value");
    return;
  }
  int v = server.arg("value").toInt();
  v = constrain(v, MIN_VOLUME, MAX_VOLUME);
  if (fmRadio.currentVolume != v) {
    fmRadio.currentVolume = v;
    pendingVolume = v;
    volumeApplyPending = true;
  }
  webSendStateV2(server);
}

static void handleApiV2Source() {
  if (webRejectIfStandby(server)) return;
  if (server.method() != HTTP_POST || !server.hasArg("value")) {
    webSendBadRequest(server, "Missing value");
    return;
  }
  int s = server.arg("value").toInt();
  if (s < 1 || s > 5) {
    webSendBadRequest(server, "Invalid source");
    return;
  }
  fmRadio.sursa = s;
  fmRadio.sursaVeche = fmRadio.sursa;
  fmRadio.lastSourceChangeMs = millis();
  if (s == 3) {
    rpiBridgeArmed = true;
  }
  audioApplyPending = true;
  audioApplyAt = millis();
  uiRefreshPending = true;
  fmRadio.saveSettings();
  webSendStateV2(server);
}

static void handleApiV2Eq() {
  if (webRejectIfStandby(server)) return;
  bool changed = false;
  if (server.hasArg("bass")) {
    int bass = constrain(server.arg("bass").toInt(), -7, 7);
    if (Bass != bass) { Bass = bass; changed = true; }
  }
  if (server.hasArg("middle")) {
    int middle = constrain(server.arg("middle").toInt(), -7, 7);
    if (Middle != middle) { Middle = middle; changed = true; }
  }
  if (server.hasArg("treble")) {
    int treble = constrain(server.arg("treble").toInt(), -7, 7);
    if (Treble != treble) { Treble = treble; changed = true; }
  }
  if (server.hasArg("gain")) {
    int gain = constrain(server.arg("gain").toInt(), -45, 0);
    if (Gain != gain) { Gain = gain; changed = true; }
  }
  if (server.hasArg("balance")) {
    int balance = constrain(server.arg("balance").toInt(), -15, 15);
    if (Balance != balance) { Balance = balance; changed = true; }
  }
  if (changed) eqApplyPending = true;
  webSendStateV2(server);
}

// PC bridge connectivity self-test (runs on ESP32).
static void handleApiV2PcTest() {
  webHeadersNoCacheClose(server);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  WebChunkedWriter w(server);
  w.write("{");

  w.write("\"esp\":{");
  w.write("\"ip\":"); w.writeJsonEscaped(WiFi.localIP().toString().c_str()); w.write(",");
  w.write("\"gw\":"); w.writeJsonEscaped(WiFi.gatewayIP().toString().c_str()); w.write(",");
  w.write("\"mask\":"); w.writeJsonEscaped(WiFi.subnetMask().toString().c_str());
  w.write("},");

  w.write("\"target\":{");
  w.write("\"host\":"); w.writeJsonEscaped(PC_BRIDGE_HOST); w.write(",");
  w.write("\"port\":"); w.writeInt((long)PC_BRIDGE_PORT);
  w.write("},");

  int code = -1;
  uint32_t dt = 0;
  bool ok = false;
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient c;
    const uint32_t t0 = millis();
    IPAddress ip;
    const bool isIp = ip.fromString(PC_BRIDGE_HOST);
    bool connected = false;
    while ((millis() - t0) <= PC_BRIDGE_TIMEOUT_MS) {
      if (isIp) connected = c.connect(ip, PC_BRIDGE_PORT);
      else connected = c.connect(PC_BRIDGE_HOST, PC_BRIDGE_PORT);
      if (connected) break;
      delay(10);
    }
    dt = millis() - t0;
    if (connected) {
      ok = true;
      code = 200;
      c.stop();
    } else {
      ok = false;
      code = -11;
    }
  } else {
    ok = false;
    code = -1;
  }

  w.write("\"result\":{");
  w.write("\"ok\":"); w.write(ok ? "true" : "false"); w.write(",");
  w.write("\"code\":"); w.writeInt((long)code); w.write(",");
  w.write("\"dtMs\":"); w.writeInt((long)dt);
  w.write("}");

  w.write("}");
  w.flush();
  webEndStream(server);
}

static void handleApiV2TunerSeek() {
  if (webRejectIfStandby(server)) return;
  if (server.method() != HTTP_POST || !server.hasArg("dir")) {
    webSendBadRequest(server, "Missing dir");
    return;
  }
  if (fmRadio.sursa != 1) {
    webSendConflict(server, "Source is not TUN");
    return;
  }
  String dir = server.arg("dir");
  if (dir == "up") pendingSeekDir = 1;
  else if (dir == "down") pendingSeekDir = -1;
  else { webSendBadRequest(server, "Invalid dir"); return; }
  webSendStateV2(server);
}

static void handleApiV2RpiCmd() {
  if (webRejectIfStandby(server)) return;
#if PC_ONLY_FOOBAR
  webSendConflict(server, "RPi bridge disabled (PC-only mode)");
  return;
#endif
  if (server.method() != HTTP_POST || !server.hasArg("cmd")) {
    webSendBadRequest(server, "Missing cmd");
    return;
  }
  if (fmRadio.sursa != 3) {
    webSendConflict(server, "RPi controls only on RPI");
    return;
  }
  rpiBridgeArmed = true;
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  if (cmd == "PREV" || cmd == "NEXT" || cmd == "PLAYPAUSE" || cmd == "GET") {
#if defined(ARDUINO_ARCH_ESP32)
    setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), cmd.c_str(), &rpiWebCmdMux);
#else
    setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), cmd.c_str());
#endif
    rpiWebCmdNextTryMs = 0;
    webSendStateV2(server);
    return;
  }
  webSendBadRequest(server, "Invalid cmd");
}

static void handleApiV2PcCmd() {
  if (webRejectIfStandby(server)) return;
  if (server.method() != HTTP_POST || !server.hasArg("cmd")) {
    webSendBadRequest(server, "Missing cmd");
    return;
  }
  if (fmRadio.sursa != 5) {
    webSendConflict(server, "PC controls only on PCD");
    return;
  }
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  if (cmd == "PREV" || cmd == "NEXT" || cmd == "PLAYPAUSE" || cmd == "PLAY" || cmd == "PAUSE" || cmd == "STOP") {
#if defined(ARDUINO_ARCH_ESP32)
    setPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmd.c_str(), &pcWebCmdMux);
#else
    setPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmd.c_str());
#endif
    webSendStateV2(server);
    return;
  }
  webSendBadRequest(server, "Invalid cmd");
}

// --- Web Control Endpoints ---
// Mute
void handleMute() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      if (!fmRadio.isMuted) fmRadio.toggleMute();
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else if (state == "off") {
      if (fmRadio.isMuted) fmRadio.toggleMute();
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else {
      server.send(400, "text/plain", "Invalid state");
    }
  } else if (server.method() == HTTP_GET) {
    server.send(200, "text/plain", fmRadio.isMuted ? "on" : "off");
  } else {
    server.send(400, "text/plain", "Missing state argument");
  }
}

// Volume
void handleVolume() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("value")) {
    int v = server.arg("value").toInt();
    v = constrain(v, MIN_VOLUME, MAX_VOLUME);
    
    if (fmRadio.currentVolume != v) {
      fmRadio.currentVolume = v;
      pendingVolume = v;
      volumeApplyPending = true;
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else {
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    }
    return;
  } else if (server.method() == HTTP_GET) {
    server.send(200, "text/plain", String(fmRadio.currentVolume));
  } else {
    server.send(400, "text/plain", "Missing value argument");
  }
}

// Source
void handleSource() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("value")) {
    int s = server.arg("value").toInt();
    // Mapping:
    // 1=TUN, 2=Bluetooth, 3=Raspberry PI, 4=PCA (PC input), 5=PCD (RPi input + foobar controls)
    // Serial.print("[WEB SRC] Received source value: ");
    // Serial.println(s);
    if (s >= 1 && s <= 5) {
      if (s == 5) {
        fmRadio.sursa = 5;
        pcdMode = false;
        pcAnalogUi = false;
      } else if (s == 3) {
        fmRadio.sursa = 3;
        pcdMode = false;
        rpiBridgeArmed = true;
      } else {
        fmRadio.sursa = s;
        pcdMode = false;
        if (s == 4) pcAnalogUi = true;
      }
      // Keep MCP "physical source" logic from immediately overwriting a WebUI change.
      // `readSensors()` compares `newSource` with `sursaVeche`; if `sursaVeche` is stale (e.g. 4 while WebUI set 3),
      // it will treat it as a "physical change" even with no buttons pressed and will force `pcdMode=false`.
      fmRadio.sursaVeche = fmRadio.sursa;
      fmRadio.lastSourceChangeMs = millis();
      // Defer NVS EQ load + I2C apply to main loop to keep HTTP responsive
      audioApplyPending = true;
      audioApplyAt = millis();
      // Serial.print("[WEB SRC] fmRadio.sursa is now ");
      // Serial.println(fmRadio.sursa);
      uiRefreshPending = true;
      fmRadio.saveSettings();
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
      return;
    } else {
      server.send(400, "text/plain", "Invalid source");
    }
  } else if (server.method() == HTTP_GET) {
    server.send(200, "text/plain", String(fmRadio.sursa));
  } else {
    server.send(400, "text/plain", "Missing value argument");
  }
}

// Frequency
void handleFreq() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("value")) {
    float f = server.arg("value").toFloat();

    if (fmRadio.sursa != 1) {
      server.send(400, "text/plain", "Source is not TUN");
      return;
    }

    if (f >= MIN_FREQ / 100.0 && f <= MAX_FREQ / 100.0) {
      fmRadio.currentFrequency = f;
      pendingFreq = f;
      freqApplyPending = true;
      lastWebFreqChange = millis();
      // Same shape as /api/v2/* — lighter than sendStatusJson (smaller body, less sendContent work).
      webSendStateV2(server);
    } else {
      server.send(400, "text/plain", "Invalid frequency");
    }
  } else if (server.method() == HTTP_GET) {
    server.send(200, "text/plain", String(fmRadio.currentFrequency, 2));
  } else {
    server.send(400, "text/plain", "Missing value argument");
  }
}

// EQ
void handleEQ() {
  if (webRejectIfStandby(server)) return;
  bool changed = false;
  if (server.hasArg("bass")) {
    int bass = server.arg("bass").toInt();
    bass = constrain(bass, -7, 7);
    if (Bass != bass) { Bass = bass; changed = true; }
  }
  if (server.hasArg("middle")) {
    int middle = server.arg("middle").toInt();
    middle = constrain(middle, -7, 7);
    if (Middle != middle) { Middle = middle; changed = true; }
  }
  if (server.hasArg("treble")) {
    int treble = server.arg("treble").toInt();
    treble = constrain(treble, -7, 7);
    if (Treble != treble) { Treble = treble; changed = true; }
  }
  if (server.hasArg("gain")) {
    int gain = server.arg("gain").toInt();
    gain = constrain(gain, -45, 0);
    if (Gain != gain) { Gain = gain; changed = true; }
  }
  if (server.hasArg("balance")) {
    int balance = server.arg("balance").toInt();
    balance = constrain(balance, -15, 15);
    if (Balance != balance) { Balance = balance; changed = true; }
  }
  if (changed) {
    eqApplyPending = true;
  }
  webBeginJsonStream(server);
  sendStatusJson(server);
  webEndStream(server);
}

// Seek
void handleSeek() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "up") {
      pendingSeekDir = 1;
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else if (dir == "down") {
      pendingSeekDir = -1;
      webBeginJsonStream(server);
      sendStatusJson(server);
      webEndStream(server);
    } else {
      server.send(400, "text/plain", "Invalid dir");
    }
  } else {
    server.send(400, "text/plain", "Missing dir argument");
  }
}

// Raspberry PI serial commands
void handleRpi() {
  if (webRejectIfStandby(server)) return;
  if (!(fmRadio.sursa == 3 && !pcdMode)) {
    webSendConflict(server, "RPi controls are only available in RPI source");
    return;
  }
  // A direct command implies user intent: arm bridge even if it was kept off after boot.
  rpiBridgeArmed = true;
  if (server.method() != HTTP_POST || !server.hasArg("cmd")) {
    webSendBadRequest(server, "Missing cmd argument");
    return;
  }

  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  if (cmd == "PREV" || cmd == "NEXT" || cmd == "PLAYPAUSE" || cmd == "GET") {
#if defined(ARDUINO_ARCH_ESP32)
    setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), cmd.c_str(), &rpiWebCmdMux);
#else
    setPendingCommand(rpiWebCmdPending, rpiWebCmdBuf, sizeof(rpiWebCmdBuf), cmd.c_str());
#endif
    rpiWebCmdNextTryMs = 0;
    webSendJsonOk(server);
    return;
  }

  webSendBadRequest(server, "Invalid cmd");
}

// PC (foobar) LAN commands
void handlePc() {
  if (webRejectIfStandby(server)) return;
  if (!(fmRadio.sursa == 4 || fmRadio.sursa == 5)) {
    webSendConflict(server, "PC controls are only available in PCA/PCD");
    return;
  }
  if (server.method() != HTTP_POST || !server.hasArg("cmd")) {
    webSendBadRequest(server, "Missing cmd argument");
    return;
  }
  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  if (cmd == "PREV" || cmd == "NEXT" || cmd == "PLAYPAUSE" || cmd == "PLAY" || cmd == "PAUSE" || cmd == "STOP") {
#if defined(ARDUINO_ARCH_ESP32)
    setPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmd.c_str(), &pcWebCmdMux);
#else
    setPendingCommand(pcWebCmdPending, pcWebCmdBuf, sizeof(pcWebCmdBuf), cmd.c_str());
#endif
    webSendJsonOk(server);
    return;
  }
  webSendBadRequest(server, "Invalid cmd");
}

// Status (all info)
void handleStatus() {
  // Prevent overlapping /status requests from wedging lwIP/WebServer.
  if (webStatusBusy) {
    webSendText(server, 503, "busy");
    return;
  }
  webStatusBusy = true;
  // Always clear the busy flag even if client disconnects mid-stream.
  webBeginJsonStream(server);
  sendStatusJson(server);
  webEndStream(server);
  webStatusBusy = false;
}

// System info pentru standby UI - foloseste doar cache (fara I2C in handler ca sa nu blocheze web UI)
void handleSystemInfo() {
  if (webSystemInfoBusy) {
    webSendText(server, 503, "busy");
    return;
  }
  webSystemInfoBusy = true;
  int wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
  String wifiSsid = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : "";
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "-";
  unsigned long uptimeSec = millis() / 1000;
  uint32_t freeHeap = ESP.getFreeHeap();

  webBeginJsonStream(server);

  server.sendContent("{");
  server.sendContent("\"ip\":"); sendJsonEscapedContent(server, ip.c_str()); server.sendContent(",");
  server.sendContent("\"wifiRssi\":"); sendContentInt(server, (long)wifiRssi); server.sendContent(",");
  server.sendContent("\"wifiSsid\":"); sendJsonEscapedContent(server, wifiSsid.c_str()); server.sendContent(",");
  server.sendContent("\"i2c\":{");
  server.sendContent("\"TDA7439\":"); server.sendContent(cachedTdaOk ? "true" : "false"); server.sendContent(",");
  server.sendContent("\"RTC_DS3231\":"); server.sendContent(cachedRtcOk ? "true" : "false"); server.sendContent(",");
  server.sendContent("\"MCP23017\":"); server.sendContent(cachedMcpOk ? "true" : "false"); server.sendContent(",");
  server.sendContent("\"mcpAddr\":");
  if (cachedMcpOk && mcpI2cAddr != 0) {
    char buf[8];
    snprintf(buf, sizeof(buf), "0x%02X", (unsigned)mcpI2cAddr);
    sendJsonEscapedContent(server, buf);
  } else {
    server.sendContent("\"-\"");
  }
  server.sendContent(",");
  server.sendContent("\"SI4703\":"); server.sendContent(cachedSi47Ok ? "true" : "false");
  server.sendContent("},");
  server.sendContent("\"freeHeap\":"); sendContentInt(server, (long)freeHeap); server.sendContent(",");
  server.sendContent("\"uptimeSec\":"); sendContentInt(server, (long)uptimeSec);
  server.sendContent("}");
  webEndStream(server);
  webSystemInfoBusy = false;
}

// Draw a red underline under the selected source button in util mode
void drawUtilSourceUnderline(int sursa) {
  // Always use the same color for each source button (like drawInitialScreen)
  fmRadio.tft.fillRect(0, 0, 80, 60, CUSTOM_DARKGREEN);  // PC
  fmRadio.tft.fillRect(80, 0, 80, 60, CUSTOM_GREY);      // RPI
  fmRadio.tft.fillRect(160, 0, 80, 60, ST77XX_BLUE);     // BT
  fmRadio.tft.fillRect(240, 0, 80, 60, ST77XX_ORANGE);   // TUN

  // Redraw the source names
  fmRadio.tft.setTextColor(ST77XX_WHITE);
  fmRadio.tft.setTextSize(2);
  fmRadio.tft.setCursor(24, 20);
  fmRadio.tft.print("PC");
  fmRadio.tft.setCursor(104, 20);
  fmRadio.tft.print("RPI");
  fmRadio.tft.setCursor(186, 20);
  fmRadio.tft.print("BT");
  fmRadio.tft.setCursor(262, 20);
  fmRadio.tft.print("TUN");

  // Draw the red selection band at the same position as touch selection
  fmRadio.tft.fillRect(0, 50, 320, 15, ST77XX_BLACK);
  int x = (4 - sursa) * 80;  // Mirror for web/display order: PC (4) left, TUN (1) right
  fmRadio.tft.fillRect(x, 50, 80, 15, ST77XX_RED);
}

// --- Radio Preset Endpoints ---
// GET /presets: returns all 10 presets as JSON
// POST /preset?id=1&freq=99.5&name=RockFM: sets preset 1 to 99.5 MHz and name 'RockFM'
// GET /preset?id=1: returns {"freq":..., "name":...} for preset 1
void handlePresets() {
  if (webRejectIfStandby(server)) return;
  if (webPresetsBusy) {
    webSendText(server, 503, "busy");
    return;
  }
  webPresetsBusy = true;
  preferences.begin("settings", true);
  webBeginJsonStream(server);
  server.sendContent("[");
  for (int i = 1; i <= 10; ++i) {
    if (i > 1) server.sendContent(",");
    char keyFreq[24], keyName[24];
    snprintf(keyFreq, sizeof(keyFreq), "preset%d_freq", i);
    snprintf(keyName, sizeof(keyName), "preset%d_name", i);

    // Back-compat: MEM1..MEM6 were historically stored as p0f..p5f (float MHz).
    // Prefer the new `preset*_freq` if present; otherwise fall back to `p*f`.
    float freq = preferences.getFloat(keyFreq, 0.0f);
    if (freq <= 0.0f && i >= 1 && i <= 6) {
      const int idx = i - 1;
      String legacyKey = "p" + String(idx) + "f";
      freq = preferences.getFloat(legacyKey.c_str(), 0.0f);
    }

    String name = preferences.getString(keyName, "");
    if (name.length() == 0 && i >= 1 && i <= 6) {
      name = "MEM" + String(i);
    }
    server.sendContent("{\"freq\":");
    sendContentFloat2(server, (double)freq);
    server.sendContent(",\"name\":");
    sendJsonEscapedContent(server, name.c_str());
    server.sendContent("}");
  }
  preferences.end();
  server.sendContent("]");
  webEndStream(server);
  webPresetsBusy = false;
}

void handlePreset() {
  if (webRejectIfStandby(server)) return;
  if (server.method() == HTTP_POST && server.hasArg("id") && server.hasArg("freq") && server.hasArg("name")) {
    int id = server.arg("id").toInt();
    float freq = server.arg("freq").toFloat();
    String name = server.arg("name");
    if (id < 1 || id > 10) {
      server.send(400, "text/plain", "Invalid preset id");
      return;
    }
    String keyFreq = "preset" + String(id) + "_freq";
    String keyName = "preset" + String(id) + "_name";
    preferences.begin("settings", false);
    preferences.putFloat(keyFreq.c_str(), freq);
    preferences.putString(keyName.c_str(), name);

    // Back-compat: mirror preset1..6 into legacy MEM storage (p0f..p5f) used by MCP buttons.
    if (id >= 1 && id <= 6) {
      const int idx = id - 1;
      String legacyKey = "p" + String(idx) + "f";
      preferences.putFloat(legacyKey.c_str(), freq);
      // Keep runtime behavior consistent without requiring reboot:
      // update the in-RAM preset array used by MCP short-press recall.
      fmRadio.sensor1MemFreq[idx] = freq;
    }
    preferences.end();
    server.send(200, "text/plain", "OK");
  } else if (server.method() == HTTP_GET && server.hasArg("id")) {
    int id = server.arg("id").toInt();
    if (id < 1 || id > 10) {
      server.send(400, "text/plain", "Invalid preset id");
      return;
    }
    String keyFreq = "preset" + String(id) + "_freq";
    String keyName = "preset" + String(id) + "_name";
    preferences.begin("settings", true);
    float freq = preferences.getFloat(keyFreq.c_str(), 0.0);
    if (freq <= 0.0f && id >= 1 && id <= 6) {
      const int idx = id - 1;
      String legacyKey = "p" + String(idx) + "f";
      freq = preferences.getFloat(legacyKey.c_str(), 0.0f);
    }
    String name = preferences.getString(keyName.c_str(), "");
    if (name.length() == 0 && id >= 1 && id <= 6) {
      name = "MEM" + String(id);
    }
    preferences.end();
    String json = "{\"freq\":" + String(freq, 2) + ",\"name\":\"" + name + "\"}";
    server.send(200, "application/json", json);
  } else {
    server.send(400, "text/plain", "Missing or invalid arguments");
  }
}

bool detectSI4703() {
  Wire.beginTransmission(SI4703_ADDR);
  return (Wire.endTransmission() == 0);
}

// Attempt to recover a stuck I2C bus by toggling SCL when SDA is held LOW.
void i2cBusRecover() {
  // Release I2C first
  #if defined(ARDUINO_ARCH_ESP32)
    Wire.end();
  #endif
  pinMode(RADIO_SDAPIN, INPUT_PULLUP);
  pinMode(RADIO_SCLPIN, INPUT_PULLUP);
  // If SDA is stuck LOW, toggle SCL up to 9 times
  if (digitalRead(RADIO_SDAPIN) == LOW) {
    pinMode(RADIO_SCLPIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
      digitalWrite(RADIO_SCLPIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(RADIO_SCLPIN, LOW);
      delayMicroseconds(5);
    }
  }
  // Re-init I2C
  Wire.begin(RADIO_SDAPIN, RADIO_SCLPIN);
  delay(2);
}

// Hardware reset sequence that forces the SI4703 into 2‑wire (I2C) mode.
bool resetSI4703Hardware() {
  Serial.println("[SI4703] Hardware reset sequence...");
  // Hold RST low
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(5);
  // Force 2-wire mode selection
  pinMode(RADIO_SDAPIN, OUTPUT);
  digitalWrite(RADIO_SDAPIN, HIGH); // SDIO high
  pinMode(RADIO_SCLPIN, OUTPUT);
  digitalWrite(RADIO_SCLPIN, LOW);  // SCLK low
  // Release reset
  digitalWrite(RESET_PIN, HIGH);
  delay(25); // allow internal startup
  // Release I2C pins
  pinMode(RADIO_SDAPIN, INPUT_PULLUP);
  pinMode(RADIO_SCLPIN, INPUT_PULLUP);
  delay(2);
  // Small settle, then probe
  delay(10);
  bool ok = detectSI4703();
  Serial.println(ok ? "[SI4703] Reset OK, device detected" : "[SI4703] Reset done, device NOT detected");
  return ok;
}

// Full re-init that recovers the I2C bus, hardware‑resets the SI4703 and reconfigures the radio object.
bool reinitSI4703() {
  // Serial.println("[SI4703] Reinitializing after power cycle...");
  i2cBusRecover();
  // Perform hardware reset to guarantee 2-wire mode after rail power-up
  resetSI4703Hardware();
  // Verify presence
  si4703Powered = detectSI4703();
  if (!si4703Powered) {
    // Serial.println("[SI4703] Not detected after reset; skipping radio init.");
    return false;
  }
  // Re-run library init/config
  fmRadio.initRadio();
  fmRadio.radio.setFrequency(fmRadio.currentFrequency * 100);
  fmRadio.radio.setVolume(fmRadio.currentVolume);
  fmRadio.radio.attachReceiveRDS(FMRadioController::RDS_process);
  // Serial.println("[SI4703] Reinit complete.");
  return true;
}

// I2C scanner function
void scanI2CBus() {
  Serial.println("\n[I2C SCAN] Scanning for I2C devices...");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("[I2C SCAN] Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (dec: ");
      Serial.print(address);
      Serial.print(")");
      if (address == 0x44) Serial.print(" <== TDA7439");
      if (address == 0x10) Serial.print(" <== SI4703");
      Serial.println();
      count++;
    }
  }
  if (count == 0) {
    Serial.println("[I2C SCAN] No I2C devices found.");
  } else {
    Serial.print("[I2C SCAN] Done. Devices found: ");
    Serial.println(count);
  }
}

// Simple communication test for TDA7439. Prints results to Serial Monitor.
void testTDA7439Comms() {
  Serial.println("\n[TDA7439 TEST] Starting...");
  // Probe multiple times to check basic address ACK
  for (int i = 0; i < 5; ++i) {
    Wire.beginTransmission(TDA7439_ADDRESS);
    byte err = Wire.endTransmission();
    Serial.print("[TDA7439 TEST] Probe ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (err == 0) {
      Serial.println("ACK (OK)");
    } else {
      Serial.print("Error ");
      Serial.println(err);
    }
    delay(20);
  }

  // Attempt a simple write (register + value) to verify data-byte ACK.
  // Note: This does not rely on readback (chip is typically write-only).
  Serial.println("[TDA7439 TEST] Writing reg 0x01 with value 0x10 (test)...");
  Wire.beginTransmission(TDA7439_ADDRESS);
  Wire.write(0x01); // Commonly volume register in many TDA7439 libs
  Wire.write(0x10); // Mid value for testing
  byte werr = Wire.endTransmission();
  if (werr == 0) {
    Serial.println("[TDA7439 TEST] Data write ACK (OK)");
  } else {
    Serial.print("[TDA7439 TEST] Data write error: ");
    Serial.println(werr);
  }

  Serial.println("[TDA7439 TEST] Finished.\n");
}

// Helper to set TDA7439 input based on sursa
void setTDA7439InputForSource(int sursa) {
  int tdaInput = 1;
  switch (sursa) {
    case 4:
      // PCA (PC) = IN1
      tdaInput = 1;
      break;
    case 5:
      // PCD (Foobar) = IN3 (legacy PC analog path on TDA7439)
      tdaInput = 3;
      break;
    case 2: tdaInput = 2; break; // BT -> IN2
    case 1: tdaInput = 4; break; // TUN (unchanged)
    case 3:
      // RPi = IN3
      tdaInput = 3;
      break;
    default: tdaInput = 1; break;
  }
  // Optional external mux/relay for PC routing (XMOS vs analog).
  if (PC_AUDIO_SEL_PIN >= 0) {
    const bool wantAnalog = (sursa == 5); // PCD = analog path
    const bool level = PC_AUDIO_SEL_ACTIVE_HIGH ? wantAnalog : !wantAnalog;
    digitalWrite(PC_AUDIO_SEL_PIN, level ? HIGH : LOW);
  }
  if (PC_DIGITAL_STATUS_PIN >= 0) {
    const bool wantAmanero = (sursa == 5); // PCD => Amanero, everything else (incl. RPI) => Raspberry
    const bool level = PC_DIGITAL_STATUS_AMANERO_LEVEL_HIGH ? wantAmanero : !wantAmanero;
    digitalWrite(PC_DIGITAL_STATUS_PIN, level ? HIGH : LOW);
  }
  tda7439.setInput(tdaInput);
  noteTdaWrite("input", tdaInput, -1);
}

// Apply all persisted audio settings to TDA7439 for current source
void applyTDA7439SettingsForCurrentSource() {
  // #region agent log: H9 measure TDA7439 apply cost (includes deliberate delays)
  uint32_t applyT0 = micros();
  // #endregion
  Wire.beginTransmission(0x44);
  byte error = Wire.endTransmission();
  if (error != 0) {
    // if (Serial.availableForWrite() >= 64) {
    //   Serial.print("[TDA7439] Apply settings error: ");
    //   Serial.println(error);
    // }
    return;
  }
  setTDA7439InputForSource(fmRadio.sursa);
  delay(2);
  // Apply gain first, then tone, then volume
  int tdaGain = (Gain + 45) / 3; // 0..15
  tda7439.inputGain(tdaGain);
  // if (Serial.availableForWrite() >= 96) {
  //   Serial.print("[TDA7439] Applying input gain: ");
  //   Serial.println(tdaGain);
  // }
  delay(2);
  tda7439.setSnd(Bass, 1);
  delay(2);
  tda7439.setSnd(Middle, 2);
  delay(2);
  tda7439.setSnd(Treble, 3);
  delay(2);
  // Respect UI mute state: re-apply should not accidentally "unmute" audio.
  {
    const int v = fmRadio.isMuted ? 0 : (int)fmRadio.currentVolume;
    tda7439.setVolume(v);
    noteTdaWrite("apply", -1, v);
  }
  delay(2);
  tda7439.spkAtt(0);
  delay(2);
  setTDA7439Balance((int8_t)Balance);
  // if (Serial.availableForWrite() >= 96) {
  //   Serial.println("[TDA7439] Applied saved input/volume/EQ/gain/balance");
  // }
  // #region agent log: H9 measure TDA7439 apply cost (includes deliberate delays)
  uint32_t applyDt = micros() - applyT0;
  if (applyDt > profApplyTdaUsMax) profApplyTdaUsMax = applyDt;
  // #endregion
}


// --- TDA7439 control functions added ---

void setTDA7439Volume(uint8_t volum) {
  if (volum > 31) volum = 31;
  tda7439Send(0x01, volum); // 0x01 = volum master
}


void setTDA7439Bass(int8_t bass) {
  if (bass < -7) bass = -7;
  if (bass > 7)  bass = 7;
  uint8_t val = (bass < 0) ? (abs(bass) | 0x08) : bass;
  tda7439Send(0x02, val & 0x0F); // 0x02 = bass
}


void setTDA7439Treble(int8_t treble) {
  if (treble < -7) treble = -7;
  if (treble > 7)  treble = 7;
  uint8_t val = (treble < 0) ? (abs(treble) | 0x08) : treble;
  tda7439Send(0x03, val & 0x0F); // 0x03 = treblein continuare
}


void setTDA7439Gain(uint8_t gain) {
  if (gain > 3) gain = 3;
  tda7439Send(0x04, gain & 0x03); // 0x04 = gain
}


void setTDA7439Balance(int8_t bal) {
  if (bal < -15) bal = -15;
  if (bal > 15)  bal = 15;
  // TDA7439 has separate speaker attenuators:
  // 0x06 = right, 0x07 = left (0..78 dB attenuation).
  // Positive balance -> attenuate LEFT (sound shifts to right).
  uint8_t rightAtt = 0;
  uint8_t leftAtt = 0;
  if (bal > 0) {
    leftAtt = (uint8_t)bal;
  } else if (bal < 0) {
    rightAtt = (uint8_t)(-bal);
  }
  tda7439Send(0x06, rightAtt);
  tda7439Send(0x07, leftAtt);
}


void tda7439Send(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TDA7439_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delay(2);
}
db,nfiu
