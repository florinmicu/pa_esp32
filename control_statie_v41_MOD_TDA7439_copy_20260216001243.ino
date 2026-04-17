#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Arduino.h>
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
#endif


#define TDA7439_ADDRESS 0x44

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
void setTDA7439Balance(int8_t bal);
void initRpiSerial();
void pollRpiSerial();
void sendRpiCommand(const char *cmd);
void processRpiLine(const String &line);
void updateRpiTitleTicker(bool forceRedraw);
void drawRpiBargraph(bool forceRedraw);
void updateRpiTransportPlayIcon(bool forceRedraw);
void updateRpiPlayIndicator(bool forceRedraw);
String shortenText(const String &s, size_t maxLen);

String shortenText(const String &s, size_t maxLen) {
  if (s.length() <= maxLen) return s;
  if (maxLen <= 3) return s.substring(0, maxLen);
  return s.substring(0, maxLen - 3) + "...";
}

// Standby control pins
#define POWER_INDICATOR_PIN 33  // OUTPUT: HIGH when ESP32 is powered
#define STANDBY_CTRL_PIN 32     // GPIO button to toggle standby
#define STANDBY_OUT_PIN 33      // Standby state output
#define RADIO_LED_PIN 17        // OUTPUT: HIGH = FM radio active, LOW = radio off
#define STANDBY_DEBOUNCE_MS 50  // Debounce interval (ms)
#define RPI_SERIAL_RX_PIN 34    // ESP32 RX <- Raspberry TX
#define RPI_SERIAL_TX_PIN 27    // ESP32 TX -> Raspberry RX
#define RPI_SERIAL_BAUD 115200
// Touch compensation for RPI control row (positive=to right, negative=to left)
#define RPI_TOUCH_X_OFFSET -72

bool inStandby = false;  // track our current mode

#if defined(ARDUINO_ARCH_ESP32)
// Forward declaration for global web server instance
extern WebServer server;
static TaskHandle_t webServerTaskHandle = nullptr;
static void webServerTask(void *param) {
  for (;;) {
    ArduinoOTA.handle();
    server.handleClient();
    vTaskDelay(2);
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

#define CUSTOM_GREY 0x7BEF
#define CUSTOM_LIGHTGREY 0xC618
#define CUSTOM_DARKGREEN 0x03E0
#define CUSTOM_LIGHTBLUE 0x7DDF
#define CUSTOM_CYAN 0x07FF

#define DEBOUNCE_DELAY 50  // Debounce delay in milliseconds
constexpr unsigned long AUTO_MODE_SWITCH_DELAY = 10000;

#define SEEK_BUTTON_X_MIN 0
#define SEEK_BUTTON_X_MAX 60
#define SEEK_BUTTON_Y_MIN 40
#define SEEK_BUTTON_Y_MAX 100
#define SEEK_BUTTON_Z_MIN 80
#define SEEK_BUTTON_Z_MAX 120
#define SEEK_BUTTON_W_MIN 40
#define SEEK_BUTTON_W_MAX 100

#define SETUP_BUTTON_X_MIN 260
#define SETUP_BUTTON_Y_MIN 200
#define SETUP_BUTTON_WIDTH 60
#define SETUP_BUTTON_HEIGHT 40

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
static String rpiState = "unknown";
static String rpiTitle = "-";
static String rpiArtist = "-";
static String rpiFile = "-";
static unsigned long lastRpiCmdTxMs = 0;
static String lastRpiCmdTx = "";
static uint8_t rpiFftL[12] = {0};
static uint8_t rpiFftR[12] = {0};
static bool rpiFftDirty = false;
static unsigned long lastRpiFftMs = 0;
static unsigned long lastRpiBarDrawMs = 0;
static unsigned long lastRpiBlinkMs = 0;
static bool rpiBlinkOn = false;

// Short-deferred TDA apply to avoid blocking HTTP handlers
static bool audioApplyPending = false;
static unsigned long audioApplyAt = 0;

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

const char *sourceNames[] = { "", "TUN", "Bluetooth", "Raspberry PI", "PC" };

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
      Serial.println("EQ mode activated: Bass");
    } else {
      eqIndex = (eqIndex + 1) % 4;
      if (eqIndex == 0)
        Serial.println("Switched to Bass");
      else if (eqIndex == 1)
        Serial.println("Switched to Mids");
      else if (eqIndex == 2)
        Serial.println("Switched to Treble");
      else if (eqIndex == 3)
        Serial.println("Switched to Gain");
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
    tft.fillRect(0, 0, 80, 60, CUSTOM_DARKGREEN);
    tft.fillRect(80, 0, 80, 60, CUSTOM_GREY);
    tft.fillRect(160, 0, 80, 60, ST77XX_BLUE);
    tft.fillRect(240, 0, 80, 60, ST77XX_ORANGE);
    tft.fillRect(0, 50, 80, 10, ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(24, 20);
    tft.print("PC");
    tft.setCursor(104, 20);
    tft.print("RPI");
    tft.setCursor(186, 20);
    tft.print("BT");
    tft.setCursor(262, 20);
    tft.print("TUN");
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
        Serial.print("Device IP address: ");
        Serial.println(ipStr);
    }
  }

  void updateSelectedSourceDisplay() {
    static const uint16_t zoneColors[4][4] = {
      { CUSTOM_DARKGREEN, CUSTOM_GREY, ST77XX_BLUE, ST77XX_RED },
      { CUSTOM_DARKGREEN, CUSTOM_GREY, ST77XX_RED, ST77XX_ORANGE },
      { CUSTOM_DARKGREEN, ST77XX_RED, ST77XX_BLUE, ST77XX_ORANGE },
      { ST77XX_RED, CUSTOM_GREY, ST77XX_BLUE, ST77XX_ORANGE }
    };
    int src = sursa - 1;
    if (src < 0 || src > 3) return;
    for (int i = 0; i < 4; i++) {
      tft.fillRect(i * 80, 50, 80, 15, zoneColors[src][i]);
    }
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
      const int rpiLeftAreaW = 214;  // keep right 1/3 free for future bargraph
      tft.fillRect(0, 60, 320, 140, ST77XX_BLACK);
      tft.fillRect(0, 110, rpiLeftAreaW, 90, ST77XX_BLACK);
      tft.fillRect(rpiLeftAreaW, 60, 320 - rpiLeftAreaW, 140, ST77XX_BLACK);
      // Keep RPI buttons in their original bottom row, +50% taller.
      tft.fillRect(0, 149, 70, 45, CUSTOM_DARKGREEN);
      tft.fillRect(72, 149, 70, 45, ST77XX_ORANGE);
      tft.fillRect(144, 149, 70, 45, CUSTOM_DARKGREEN);
      tft.setTextColor(ST77XX_WHITE);
      // PREV: two filled left-pointing triangles
      tft.fillTriangle(20, 171, 34, 161, 34, 181, ST77XX_WHITE);
      tft.fillTriangle(38, 171, 52, 161, 52, 181, ST77XX_WHITE);
      updateRpiTransportPlayIcon(true);
      // NEXT: two filled right-pointing triangles
      tft.fillTriangle(166, 161, 166, 181, 180, 171, ST77XX_WHITE);
      tft.fillTriangle(184, 161, 184, 181, 198, 171, ST77XX_WHITE);

      tft.setTextSize(1);
      tft.setTextColor(ST77XX_CYAN);
      tft.setCursor(5, 77);
      tft.print("RPI:");
      tft.print(rpiConnected ? " ONLINE " : " OFFLINE ");
      tft.print(" ");
      tft.print(rpiState);
      // Right side of status row: compact stream format info from bridge (e.g. "44.1k 16b AAC")
      tft.fillRect(147, 74, 88, 10, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(147, 77);
      tft.print(shortenText(rpiFile, 14));
      tft.setTextColor(ST77XX_CYAN);

      auto shorten = [](const String &s, size_t maxLen) -> String {
        if (s.length() <= maxLen) return s;
        return s.substring(0, maxLen - 3) + "...";
      };

      updateRpiTitleTicker(true);
      tft.fillRect(5, 123, 206, 20, ST77XX_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(5, 125);
      tft.print(shorten(rpiArtist, 18));
      drawRpiBargraph(true);
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
                    Serial.print("TDA7439 Error during encoder volume change: ");
                    Serial.println(error);
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
      sursa = newSource;
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
        Serial.println("[TDA7439] Muted");
      } else {
        tda7439.setVolume(currentVolume);
        Serial.print("[TDA7439] Unmuted, restoring volume: ");
        Serial.println(currentVolume);
      }
    } else {
      Serial.print("[TDA7439] NACK Error during mute toggle! Error code: ");
      Serial.println(error);
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
    Serial.println("[SEEK] Starting seek up");
    // Preflight: if bus/device not OK, try reinit to avoid freezes
    if (!detectSI4703()) {
      Serial.println("[SEEK] SI4703 not responding; attempting reinit...");
      if (!reinitSI4703()) {
        Serial.println("[SEEK] Reinit failed; aborting seek.");
        return;
      }
    }
    if (si4703Powered) {
      radio.seekUp(true);
      delay(100); // Give radio time to seek
      currentFrequency = radio.getFrequency() / 100.0;
      Serial.print("[SEEK] Found frequency: ");
      Serial.println(currentFrequency);
      oldFrequency = currentFrequency;
      clearRDSData();
      updateMainContent();
      saveSettings();
      lastWebFreqChange = millis();
    } else {
      Serial.println("[SEEK] Radio not powered, cannot seek");
    }
  }


  void seekDown() {
    if (sursa != 1) return;
    Serial.println("[SEEK] Starting seek down");
    // Preflight: if bus/device not OK, try reinit to avoid freezes
    if (!detectSI4703()) {
      Serial.println("[SEEK] SI4703 not responding; attempting reinit...");
      if (!reinitSI4703()) {
        Serial.println("[SEEK] Reinit failed; aborting seek.");
        return;
      }
    }
    if (si4703Powered) {
      radio.seekDown(true);
      delay(100); // Give radio time to seek
      currentFrequency = radio.getFrequency() / 100.0;
      Serial.print("[SEEK] Found frequency: ");
      Serial.println(currentFrequency);
      oldFrequency = currentFrequency;
      clearRDSData();
      updateMainContent();
      saveSettings();
      lastWebFreqChange = millis();
    } else {
      Serial.println("[SEEK] Radio not powered, cannot seek");
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
    mappedX = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_WIDTH);
    mappedY = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_HEIGHT);
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
    preferences.begin("settings", false);
    preferences.putInt("sursa", sursa);
    preferences.putInt("freq", (int)(currentFrequency * 100));
    preferences.putInt("volume", currentVolume);

    String keyBass = "bass" + String(sursa);
    String keyMiddle = "mid" + String(sursa);
    String keyTreble = "treble" + String(sursa);
    String keyGain = "gain" + String(sursa);
    String keyBalance = "bal" + String(sursa);
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
    currentVolume = preferences.getInt("volume", DEFAULT_VOLUME);
    for (int i = 0; i < 6; i++) {
      sensor1MemFreq[i] = preferences.getFloat(("p" + String(i) + "f").c_str(), defaultPresetFreqs[i]);
    }

    String keyBass = "bass" + String(sursa);
    String keyMiddle = "mid" + String(sursa);
    String keyTreble = "treble" + String(sursa);
    String keyGain = "gain" + String(sursa);
    String keyBalance = "bal" + String(sursa);
    Bass = preferences.getInt(keyBass.c_str(), 0);
    Middle = preferences.getInt(keyMiddle.c_str(), 0);
    Treble = preferences.getInt(keyTreble.c_str(), 0);
    Gain = preferences.getInt(keyGain.c_str(), -30);
    Balance = preferences.getInt(keyBalance.c_str(), 0);

    preferences.end();
  }

  void loadEqualizer() {
    preferences.begin("settings", true);
    String keyBass = "bass" + String(sursa);
    String keyMiddle = "mid" + String(sursa);
    String keyTreble = "treble" + String(sursa);
    String keyGain = "gain" + String(sursa);
    String keyBalance = "bal" + String(sursa);
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

void updateRpiTitleTicker(bool forceRedraw) {
  static String lastBaseTitle = "";
  static String scrollBase = "";
  static size_t scrollIdx = 0;
  static unsigned long lastTickMs = 0;
  const unsigned long TITLE_SCROLL_STEP_MS = 300UL;

  if (standbyState || fmRadio.utilMode || fmRadio.sursa != 3) return;

  String baseTitle = rpiTitle;
  baseTitle.trim();
  if (baseTitle.length() == 0) baseTitle = "-";

  if (forceRedraw || baseTitle != lastBaseTitle) {
    lastBaseTitle = baseTitle;
    scrollBase = baseTitle + "  ";
    scrollIdx = 0;
    lastTickMs = 0;
  }

  const size_t windowLen = 18;
  bool longTitle = (baseTitle.length() > windowLen);
  unsigned long now = millis();
  if (!forceRedraw) {
    if (!longTitle) return;  // static if it fits
    if (now - lastTickMs < TITLE_SCROLL_STEP_MS) return;
  }
  lastTickMs = now;

  String view;
  if (longTitle) {
    String doubled = scrollBase + scrollBase;
    if (scrollIdx >= scrollBase.length()) scrollIdx = 0;
    view = doubled.substring(scrollIdx, scrollIdx + windowLen);
    scrollIdx++;
  } else {
    view = baseTitle;
  }

  fmRadio.tft.fillRect(5, 99, 206, 24, ST77XX_BLACK);
  fmRadio.tft.setTextColor(ST77XX_YELLOW);
  fmRadio.tft.setTextSize(2);
  fmRadio.tft.setCursor(5, 101);
  fmRadio.tft.print(view);
}

void drawRpiBargraph(bool forceRedraw) {
  if (standbyState || fmRadio.utilMode || fmRadio.sursa != 3) return;

  const int graphX = 222;
  const int barW = 7;
  const int gap = 1;
  const int bars = 12;
  const int topY = 74;
  const int midY = 128;
  const int bottomY = 190;
  const int topH = midY - topY - 2;
  const int bottomH = bottomY - 136;
  const int maxH = (topH < bottomH) ? topH : bottomH;

  static int prevL[12];
  static int prevR[12];
  static bool init = false;
  if (!init || forceRedraw) {
    for (int i = 0; i < bars; i++) {
      prevL[i] = -1;
      prevR[i] = -1;
    }
    init = true;
    fmRadio.tft.fillRect(214, 60, 106, 140, ST77XX_BLACK);
    fmRadio.tft.setTextSize(1);
    fmRadio.tft.setTextColor(ST77XX_CYAN);
    fmRadio.tft.setCursor(216, 64);
    fmRadio.tft.print("L");
    fmRadio.tft.setCursor(216, 132);
    fmRadio.tft.print("R");
  }

  for (int i = 0; i < bars; i++) {
    int x = graphX + i * (barW + gap);
    int hL = map((int)rpiFftL[i], 0, 255, 0, maxH);
    int hR = map((int)rpiFftR[i], 0, 255, 0, maxH);

    if (prevL[i] != hL || forceRedraw) {
      fmRadio.tft.fillRect(x, midY - maxH, barW, maxH, ST77XX_BLACK);
      if (hL > 0) fmRadio.tft.fillRect(x, midY - hL, barW, hL, ST77XX_WHITE);
      prevL[i] = hL;
    }
    if (prevR[i] != hR || forceRedraw) {
      fmRadio.tft.fillRect(x, bottomY - maxH, barW, maxH, ST77XX_BLACK);
      if (hR > 0) fmRadio.tft.fillRect(x, bottomY - hR, barW, hR, ST77XX_WHITE);
      prevR[i] = hR;
    }
  }
}

void updateRpiPlayIndicator(bool forceRedraw) {
  // Dot in the top-right corner of the RPI source button (x: 80..159, y: 0..59).
  const int dotX = 151;
  const int dotY = 13;
  const int dotR = 6;

  static bool lastActive = false;
  static bool lastShown = false;

  // Keep indicator off in standby/util screens.
  bool activeScreen = !standbyState && !fmRadio.utilMode;
  bool rpiPlaying = rpiState.equalsIgnoreCase("play") || rpiState.equalsIgnoreCase("playing");
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

  bool activeScreen = !standbyState && !fmRadio.utilMode && fmRadio.sursa == 3;
  if (!activeScreen) {
    lastValid = false;
    return;
  }

  bool rpiIsPlaying = rpiState.equalsIgnoreCase("play") || rpiState.equalsIgnoreCase("playing");
  if (!forceRedraw && lastValid && (lastPlaying == rpiIsPlaying)) return;

  // Middle RPI control button: orange background + icon that indicates next action.
  fmRadio.tft.fillRect(72, 149, 70, 45, ST77XX_ORANGE);
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
  
  // Drive standby output LOW when entering standby (standby = power OFF)
  digitalWrite(STANDBY_OUT_PIN, LOW);

  tda7439.setVolume(0);

  Serial.println("Entering Standby mode");
  // Immediately render standby clock/date so the screen isn't left blank
  showtimestandBy();
}

void exitStandby() {
  inStandby = false;
  standbyState = false;
  standbyFullRedrawPending = true;
  rtc.begin();
  delay(10);

  // Drive standby output HIGH when exiting standby (active = power ON)
  digitalWrite(STANDBY_OUT_PIN, HIGH);
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
  Serial.println("Exit standby mode");
  forceFullTimeRedraw = true;  // Force full redraw on next time update

  // Set volume to fixed startup value
  fmRadio.currentVolume = 10;
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

  // On Raspberry source, do not draw time/date.
  if (FMRadioController::instance->sursa == 3) {
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
}

//------------------------------------
// Touch Handling Function
//------------------------------------
void checkTouch() {
  static bool rpiTouchLatched = false;
  if (!FMRadioController::instance->ts.touched()) {
    rpiTouchLatched = false;
    return;
  }
  if (FMRadioController::instance->ts.touched()) {
    TS_Point p = FMRadioController::instance->ts.getPoint();
    int mappedX, mappedY;
    FMRadioController::instance->mapTouch(p.x, p.y, mappedX, mappedY);

    if (p.z > 10) {
      if (FMRadioController::instance->utilMode) lastUtilActivity = millis();
      // RTC button (bottom left)
      if (FMRadioController::instance->utilMode && mappedX >= 260 && mappedX <= 320 && mappedY >= 0 && mappedY <= 40) {
        FMRadioController::instance->rtcUpdateSuccess = syncRTCWithNTPRetries();
        FMRadioController::instance->rtcUpdateMsgMillis = millis();
        FMRadioController::instance->drawUtilButton();
        drawsetScreen();
        return;
      }
      // Check if touch is within the Util/Back button area (top left)
      if (mappedX >= 0 && mappedX <= 60 && mappedY >= 0 && mappedY <= 40) {
        static unsigned long lastUtil = 0;
        if (millis() - lastUtil < 700) return;  // debounce = 700 ms ca in toggleUtilMode
        lastUtil = millis();
        FMRadioController::instance->utilMode = !FMRadioController::instance->utilMode;
        FMRadioController::instance->toggleUtilMode();
        Serial.println(FMRadioController::instance->utilMode);
        return;
      }
      // In util (setup) mode, change sursa by touching the lower part.
      if (mappedY > 180 && FMRadioController::instance->utilMode) {
        int newSursa = 0;
        if (mappedX < 80) newSursa = 1;
        else if (mappedX < 160) newSursa = 2;
        else if (mappedX < 240) newSursa = 3;
        else if (mappedX < 320) newSursa = 4;
        if (newSursa != 0 && newSursa != FMRadioController::instance->sursa) {
          // Save current settings before switching.
          FMRadioController::instance->saveSettings();
          // Update sursa.
          FMRadioController::instance->sursa = newSursa;
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
        // RPI controls: include both normal and Y-inverted calibration bands
        bool rpiButtonBand = ((mappedY >= 140 && mappedY <= 202) || (mappedY >= 38 && mappedY <= 100));
        if (FMRadioController::instance->sursa == 3 && rpiButtonBand) {
          int rpiTouchX = constrain(mappedX + RPI_TOUCH_X_OFFSET, 0, SCREEN_WIDTH - 1);
          int rpiButton = -1;
          if (rpiTouchX <= 71) rpiButton = 0;         // left button
          else if (rpiTouchX <= 143) rpiButton = 1;   // center button
          else if (rpiTouchX <= 214) rpiButton = 2;   // right button

          if (rpiButton >= 0) {
            if (!rpiTouchLatched) {
              rpiTouchLatched = true;
              if (rpiButton == 0) {
                sendRpiCommand("NEXT");      // swap PREV/NEXT to match on-screen behavior
              } else if (rpiButton == 1) {
                sendRpiCommand("PLAYPAUSE");
              } else {
                sendRpiCommand("PREV");
              }
            }
          } else {
            // Allow immediate retrigger if finger slides back over a button.
            rpiTouchLatched = false;
          }
          return;
        }
        if (mappedX >= SEEK_BUTTON_X_MIN && mappedX <= SEEK_BUTTON_X_MAX && mappedY >= SEEK_BUTTON_Y_MIN && mappedY <= SEEK_BUTTON_Y_MAX) {
          FMRadioController::instance->seekUp();
          FMRadioController::instance->updateMainContent();
          FMRadioController::instance->saveSettings();
        }
        if (mappedX >= SEEK_BUTTON_Z_MIN && mappedX <= SEEK_BUTTON_Z_MAX && mappedY >= SEEK_BUTTON_W_MIN && mappedY <= SEEK_BUTTON_W_MAX) {
          FMRadioController::instance->seekDown();
          FMRadioController::instance->updateMainContent();
          FMRadioController::instance->saveSettings();
        } else if (mappedY > 180) {
          int newSursa = 0;
          if (mappedX < 80) newSursa = 1;
          else if (mappedX < 160) newSursa = 2;
          else if (mappedX < 240) newSursa = 3;
          else if (mappedX < 320) newSursa = 4;
          if (newSursa != 0 && newSursa != FMRadioController::instance->sursa) {
            FMRadioController::instance->sursa = newSursa;
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

      // Zone touch EQ +/- (Y conform calibrarii touch-ului tau - sursa/RTC/Back merg bine)
      if (mappedX >= 240 && mappedX <= 320) {
        if (mappedY >= 128 && mappedY <= 168)
          adjustSetting(Bass, 1, -7, 7, 14, 134);
        else if (mappedY >= 44 && mappedY <= 84)
          adjustSetting(Bass, -1, -7, 7, 14, 134);
      } else if (mappedX >= 160 && mappedX <= 239) {
        if (mappedY >= 128 && mappedY <= 168)
          adjustSetting(Middle, 1, -7, 7, 94, 134);
        else if (mappedY >= 44 && mappedY <= 84)
          adjustSetting(Middle, -1, -7, 7, 94, 134);
      } else if (mappedX >= 80 && mappedX <= 159) {
        if (mappedY >= 128 && mappedY <= 168)
          adjustSetting(Treble, 1, -7, 7, 174, 134);
        else if (mappedY >= 44 && mappedY <= 84)
          adjustSetting(Treble, -1, -7, 7, 174, 134);
      } else if (mappedX >= 0 && mappedX <= 79) {
        if (mappedY >= 128 && mappedY <= 168)
          adjustSetting(Gain, 1, -45, 0, 259, 134);
        else if (mappedY >= 44 && mappedY <= 84)
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

//------------------------------------
// WiFi + NTP Sync Function
//------------------------------------
void connectToWiFi() {
  // Start WiFi in Station mode
  WiFi.mode(WIFI_STA);
  // Reduce latency caused by WiFi modem sleep
  WiFi.setSleep(false);
  WiFi.setHostname("flo-amp");
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  Serial.print("SSID: ");
  Serial.println(ssid);

  // Wait up to 20 seconds for connection
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000) {
    Serial.print(".");
    delay(500);
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


bool syncRTCWithNTP() {
  // Romania timezone with automatic DST (EET/EEST)
  configTzTime("EET-2EEST,M3.5.0/3,M10.5.0/4", "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync");
  const unsigned long ntpTimeoutMs = 12000;  // max 12 s ca setup-ul sa continue si web/mDNS sa porneasca
  unsigned long startMs = millis();
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2 && (millis() - startMs) < ntpTimeoutMs) {
    delay(500);
    Serial.print(".");
    nowSecs = time(nullptr);
  }
  if ((millis() - startMs) >= ntpTimeoutMs) {
    Serial.println(" timeout (continuing without NTP).");
    return false;
  }
  Serial.println(" done.");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
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

  Serial.println("RTC updated with NTP time.");
  return true;
}

bool syncRTCWithNTPRetries() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NTP] WiFi not connected, skipping sync cycle.");
    return false;
  }

  for (uint8_t attempt = 1; attempt <= NTP_SYNC_MAX_RETRIES; attempt++) {
    Serial.print("[NTP] Sync attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(NTP_SYNC_MAX_RETRIES);

    if (syncRTCWithNTP()) {
      Serial.println("[NTP] Sync successful.");
      return true;
    }

    if (attempt < NTP_SYNC_MAX_RETRIES) {
      delay(NTP_RETRY_DELAY_MS);
    }
  }

  Serial.println("[NTP] 3 failed attempts. Keeping RTC time until next scheduled sync.");
  return false;
}

void initRpiSerial() {
  Serial2.begin(RPI_SERIAL_BAUD, SERIAL_8N1, RPI_SERIAL_RX_PIN, RPI_SERIAL_TX_PIN);
  Serial.printf("[RPI] Serial bridge initialized RX=%d TX=%d @%d\n", RPI_SERIAL_RX_PIN, RPI_SERIAL_TX_PIN, RPI_SERIAL_BAUD);
}

void sendRpiCommand(const char *cmd) {
  unsigned long now = millis();
  String cmdStr = String(cmd);
  // Guard against accidental command floods (especially PLAYPAUSE).
  if (cmdStr == "PLAYPAUSE" && (now - lastRpiCmdTxMs) < 450UL) return;
  if (cmdStr == lastRpiCmdTx && (now - lastRpiCmdTxMs) < 200UL) return;
  lastRpiCmdTx = cmdStr;
  lastRpiCmdTxMs = now;
  Serial.printf("[RPI] TX CMD: %s\n", cmd);
  Serial2.print("CMD|");
  Serial2.print(cmd);
  Serial2.print('\n');
}

static inline const char *skipSpaces(const char *p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
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

static const char *parseCsv12(const char *p, char endDelim, uint8_t *outVals) {
  for (int i = 0; i < 12; i++) {
    p = skipSpaces(p);
    if (*p == endDelim || *p == '\0') {
      for (int j = i; j < 12; j++) outVals[j] = 0;
      return p;
    }
    char *endp = nullptr;
    long v = strtol(p, &endp, 10);
    if (endp == p) {
      // Not a number; skip until next delimiter and treat as 0.
      outVals[i] = 0;
      while (*p && *p != ',' && *p != endDelim) p++;
    } else {
      if (v < 0) v = 0;
      if (v > 255) v = 255;
      outVals[i] = (uint8_t)v;
      p = endp;
    }
    p = skipSpaces(p);
    if (*p == ',') p++;
    else if (*p == endDelim || *p == '\0') {
      // OK: end of this CSV section
    } else {
      // Unknown separator; advance defensively
      while (*p && *p != ',' && *p != endDelim) p++;
      if (*p == ',') p++;
    }
  }
  return p;
}

void processRpiLine(const char *line) {
  if (strncmp(line, "FFT|", 4) == 0) {
    const char *p = line + 4;
    p = parseCsv12(p, '|', rpiFftL);
    if (*p == '|') p++;
    parseCsv12(p, '\0', rpiFftR);
    // Mark bridge link alive on FFT frames too; otherwise periodic GET probes can
    // preempt serial bandwidth and create visible micro-stutter.
    rpiConnected = true;
    lastRpiRxMs = millis();
    lastRpiFftMs = millis();
    rpiFftDirty = true;
    return;
  }
  if (strncmp(line, "STAT|", 5) != 0) return;
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

  String newState = String(newStateBuf);
  String newTitle = String(newTitleBuf);
  String newArtist = String(newArtistBuf);
  String newFile = String(newFileBuf);

  bool stateChanged = (newState != rpiState);
  bool titleChanged = (newTitle != rpiTitle);
  bool artistChanged = (newArtist != rpiArtist);
  bool fileChanged = (newFile != rpiFile);
  bool majorChanged = stateChanged || titleChanged || artistChanged;
  rpiState = newState;
  rpiTitle = newTitle;
  rpiArtist = newArtist;
  rpiFile = newFile;
  rpiConnected = true;
  lastRpiRxMs = millis();

  if (!fmRadio.utilMode && fmRadio.sursa == 3) {
    if (majorChanged) {
      fmRadio.updateMainContent();
    } else if (fileChanged) {
      // Avoid full-layout redraw for format-only changes; update just the right status segment.
      fmRadio.tft.setTextSize(1);
      fmRadio.tft.fillRect(147, 74, 88, 10, ST77XX_BLACK);
      fmRadio.tft.setTextColor(ST77XX_WHITE);
      fmRadio.tft.setCursor(147, 77);
      fmRadio.tft.print(shortenText(rpiFile, 14));
      fmRadio.tft.setTextColor(ST77XX_CYAN);
    }
  }
}

void pollRpiSerial() {
  while (Serial2.available()) {
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
  // If link is online keep quick refresh; if offline back off retries to reduce UART churn.
  const unsigned long onlinePollMs = 3000UL;
  const unsigned long offlinePollMs = 12000UL;
  const unsigned long pollIntervalMs = rpiConnected ? onlinePollMs : offlinePollMs;
  // Bridge already pushes STAT periodically; request GET only when updates look stale.
  if (!standbyState && fmRadio.sursa == 3
      && (now - lastRpiPollMs >= pollIntervalMs)
      && (now - lastRpiRxMs >= 1800UL)) {
    Serial2.print("GET\n");
    lastRpiPollMs = now;
  }
  if (rpiConnected && (now - lastRpiRxMs >= 5000UL)) {
    rpiConnected = false;
    if (!fmRadio.utilMode && fmRadio.sursa == 3) {
      fmRadio.updateMainContent();
    }
  }
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
  Serial.begin(115200);
  delay(1000);
  initRpiSerial();

  Serial.println("Starting setup...");

  // Ensure power rails (relay) are ON before any I2C/touch/display init
  pinMode(POWER_INDICATOR_PIN, OUTPUT);
  digitalWrite(POWER_INDICATOR_PIN, HIGH);  // GPIO33 = 1 when ESP32 powered
  pinMode(STANDBY_OUT_PIN, OUTPUT);
  digitalWrite(STANDBY_OUT_PIN, HIGH);  // Active mode = power ON
  pinMode(RADIO_LED_PIN, OUTPUT);
  digitalWrite(RADIO_LED_PIN, LOW);    // Will be set in loop when FM is active
  Serial.println("Standby pin set HIGH (power ON)");

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

  // Standby pin already initialized at start; keep rails ON after OTA init
  digitalWrite(STANDBY_OUT_PIN, HIGH);

  Serial.println("fmRadio.loadSettings()");
  fmRadio.loadSettings();

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
  server.on("/", handleRoot);
  server.on("/power", HTTP_POST, handlePower);
  server.on("/powerState", HTTP_GET, handlePowerState);
  server.on("/mute", handleMute);
  server.on("/muteState", HTTP_GET, handleMute);
  server.on("/volume", handleVolume);
  server.on("/source", handleSource);
  server.on("/freq", handleFreq);
  server.on("/eq", handleEQ);
  server.on("/seek", HTTP_POST, handleSeek);
  server.on("/rpi", HTTP_POST, handleRpi);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/systemInfo", HTTP_GET, handleSystemInfo);
  server.on("/presets", handlePresets);
  server.on("/preset", handlePreset);
  server.begin();
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
    delay(200);
    if (!MDNS.begin("flo-amp")) {
      delay(1000);
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

#if defined(ARDUINO_ARCH_ESP32)
  // Run HTTP/OTA handling on core 0 to keep UI loop responsive
  xTaskCreatePinnedToCore(webServerTask, "webServerTask", 12288, nullptr, 2, &webServerTaskHandle, 0);
#endif

  // Set up IR receiver on pin 12
  IrReceiver.begin(12, ENABLE_LED_FEEDBACK); // Use pin 12 for IR

  // Ensure standby button pin is set up
  pinMode(STANDBY_CTRL_PIN, INPUT_PULLUP);

 // Removed debug direct volume write that overwrote saved state

  tda7439.spkAtt(0); // Set speaker attenuation to middle value at startup
  
  // Run a brief TDA7439 communication test and report to Serial
  testTDA7439Comms();
}

void loop() {
  static int lastSourceSeen = -1;
  mainLoopCounter++;
  // HTTP/OTA handled in a separate task (ESP32)
#if !defined(ARDUINO_ARCH_ESP32)
  ArduinoOTA.handle();
  server.handleClient();
#endif

  // Force an immediate RPi state read when entering RPI source (or at boot on RPI source).
  if (!standbyState && fmRadio.sursa == 3 && lastSourceSeen != 3) {
    sendRpiCommand("GET");
    lastRpiPollMs = millis();
  }
  lastSourceSeen = fmRadio.sursa;

  pollRpiSerial();
  if (!standbyState && fmRadio.sursa == 3) {
    // Keep FFT drawing first to reduce occasional UI micro-freeze.
    if (rpiFftDirty && (millis() - lastRpiBarDrawMs >= 35UL)) {
      drawRpiBargraph(false);
      lastRpiBarDrawMs = millis();
      rpiFftDirty = false;
    }
    if (lastRpiFftMs > 0 && (millis() - lastRpiFftMs > 1500UL)) {
      for (int i = 0; i < 12; i++) {
        rpiFftL[i] = 0;
        rpiFftR[i] = 0;
      }
      lastRpiFftMs = millis();
      rpiFftDirty = true;
    }
  }
  updateRpiTitleTicker(false);
  updateRpiTransportPlayIcon(false);
  updateRpiPlayIndicator(false);

  // Coalesce frequent UI-originated preference writes.
  fmRadio.flushSettingsIfDue(false);

  // Actualizare cache system info la 5 s (I2C doar in main loop, nu in handler web)
  if (millis() - lastSystemInfoCacheMs >= 5000) {
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
    Serial.println("[NTP] Scheduled hourly sync cycle...");
    syncRTCWithNTPRetries();
    nextNtpSyncDueMs += NTP_SYNC_INTERVAL_MS;
    // If we were delayed a lot, avoid running many catch-up cycles back-to-back.
    if ((long)(millis() - nextNtpSyncDueMs) >= 0) {
      nextNtpSyncDueMs = millis() + NTP_SYNC_INTERVAL_MS;
    }
  }

  // One-time delayed post-boot audio reapply to ensure gain latches
  if (postBootAudioApplyPending && millis() >= postBootAudioApplyAt && !standbyState) {
    Serial.println("[TDA7439] Post-boot reapply settings");
    applyTDA7439SettingsForCurrentSource();
    postBootAudioApplyPending = false;
  }
  
  // Short-deferred apply triggered by HTTP handlers (e.g., source change)
  if (audioApplyPending && millis() >= audioApplyAt && !standbyState) {
    Serial.println("[TDA7439] Deferred apply settings");
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

  // Keep GPIO33 (LED/standby output) in sync with standbyState every loop
  digitalWrite(STANDBY_OUT_PIN, standbyState ? LOW : HIGH);

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
      Serial.println("[SYSTEM] Reset command received. Restarting ESP32...");
      delay(500); // Give time for the message to be sent
      ESP.restart();
    } else if (c == 't') {
      Serial.println("[SYSTEM] Running TDA7439 communication test...");
      testTDA7439Comms();
    }
  }

  if (IrReceiver.decode()) {
    uint8_t irCode = IrReceiver.decodedIRData.command;
    Serial.print("IR code: 0x");
    Serial.println(irCode, HEX);
    Serial.println(irCode, HEX); // Only IR code in HEX

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
          // fmRadio.drawsetScreen(); // <-- ensures value color is correct
        } else {
          // Normal mode: volume up
          fmRadio.currentVolume = min((int32_t)(fmRadio.currentVolume + 1), (int32_t)MAX_VOLUME);
          
          // Check TDA7439 before changing volume
          Wire.beginTransmission(0x44);
          byte error = Wire.endTransmission();
          if (error == 0) {
            fmRadio.currentVolume = fmRadio.currentVolume + 1;
            tda7439.setVolume(fmRadio.currentVolume);
            Serial.print("[TDA7439] Volume increased to: ");
            Serial.println(fmRadio.currentVolume);
            setEncoderCount(fmRadio.currentVolume);
            encoderLastEncVal = fmRadio.currentVolume;
            fmRadio.updateVolumeDisplay();
            fmRadio.saveSettings();
          } else {
            Serial.print("[TDA7439] NACK Error during IR volume up! Error code: ");
            Serial.println(error);
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
          fmRadio.drawsetScreen(); // <-- ensures value color is correct
        } else {
          // Normal mode: volume down
          fmRadio.currentVolume = max((int32_t)(fmRadio.currentVolume - 1), (int32_t)MIN_VOLUME);
          
          // Check TDA7439 before changing volume
          Wire.beginTransmission(0x44);
          byte error = Wire.endTransmission();
          if (error == 0) {
            fmRadio.currentVolume = fmRadio.currentVolume - 1;
            tda7439.setVolume(fmRadio.currentVolume);
            Serial.print("[TDA7439] Volume decreased to: ");
            Serial.println(fmRadio.currentVolume);
            setEncoderCount(fmRadio.currentVolume);
            encoderLastEncVal = fmRadio.currentVolume;
            fmRadio.updateVolumeDisplay();
            fmRadio.saveSettings();
          } else {
            Serial.print("[TDA7439] NACK Error during IR volume down! Error code: ");
            Serial.println(error);
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
          Serial.println("[IR] Seek down command received");
          fmRadio.seekDown();
        } else {
          // Other sources: NEXT source (reversed)
          fmRadio.sursa++;
          if (fmRadio.sursa > 4) fmRadio.sursa = 1;
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
          Serial.println("[IR] Seek up command received");
          fmRadio.seekUp();
        } else {
          // Other sources: PREVIOUS source (reversed)
          fmRadio.sursa--;
          if (fmRadio.sursa < 1) fmRadio.sursa = 4;
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
        if (fmRadio.sursa > 4) fmRadio.sursa = 1;
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
      if (fmRadio.sursa < 1) fmRadio.sursa = 4;
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
  fmRadio.update();
  checkTouch();
  showTime();

  // If utilMode and rtcUpdateSuccess, redraw util button to update/hide message
  if (fmRadio.utilMode && fmRadio.rtcUpdateSuccess && millis() - fmRadio.rtcUpdateMsgMillis > 2000) {
    fmRadio.rtcUpdateSuccess = false;
    fmRadio.drawUtilButton();
    drawsetScreen();
  }
}

// Add this handler function near the top-level functions (after your includes and before setup/loop):
void handleRoot() {
  if (standbyState) {
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "0");
    server.send(200, "text/html", R"rawliteral(
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
    function refreshInfo() {
      fetch('/systemInfo')
        .then(function(r) { return r.json(); })
        .then(function(d) {
          document.getElementById('infoContent').innerHTML = renderInfo(d);
        })
        .catch(function() {
          document.getElementById('infoContent').innerHTML = "<span class='i2c-fail'>Failed to load system info.</span>";
        });
    }
    refreshInfo();
    setInterval(refreshInfo, 5000);
    document.getElementById('powerBtn').onclick = function() {
      fetch('/power?state=on', {method: 'POST'})
        .then(function() { location.reload(); })
        .catch(function(err) { alert('Error powering on: ' + err); });
    };
  </script>
</body>
</html>
)rawliteral");
    return;
  }
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", R"rawliteral(
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
    <div class='volume-row'>
      <div class='volume-label'>Volume</div>
      <input type='range' min='0' max='48' id='volume' class='volume-slider'>
      <div class='volume-value' id='volumeVal'></div>
    </div>
    <div class='source-row'>
      <button class='source-btn pc' id='src4'>PC</button>
      <button class='source-btn rpi' id='src3'>Raspberry PI</button>
      <button class='source-btn bt' id='src2'>Bluetooth</button>
      <button class='source-btn tun' id='src1'>TUN</button>
    </div>
    <div class='row' id='freqRow' style='display:none;'>
      <label for='freq'>FM Freq (MHz)</label>
      <input type='number' min='87.5' max='108.0' step='0.1' id='freq'>
    </div>
    <div class='row' id='rssiRow' style='display:none;'>
      <label>Signal</label>
      <div style='display:flex; gap:12px; align-items:center;'>
        <div id='rssiVal'>-</div>
        <div id='wifiVal' style='opacity:0.8; font-size:0.95em;'>WiFi: - dBm</div>
      </div>
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
    <div class='row' id='rpiCtrlRow' style='display:none; gap:8px;'>
      <button class='seek-btn' id='rpiPrevBtn'>Prev</button>
      <button class='seek-btn' id='rpiPlayBtn'>Play</button>
      <button class='seek-btn' id='rpiNextBtn'>Next</button>
    </div>
    <div style='margin:16px 0 8px 0; text-align:center;'>
      <b>Equalizer</b>
    </div>
    <div class='row'><span class='eq-label'>Bass</span><input type='range' min='-7' max='7' id='bass'><span id='bassVal'></span></div>
    <div class='row'><span class='eq-label'>Middle</span><input type='range' min='-7' max='7' id='middle'><span id='middleVal'></span></div>
    <div class='row'><span class='eq-label'>Treble</span><input type='range' min='-7' max='7' id='treble'><span id='trebleVal'></span></div>
    <div class='row'><span class='eq-label'>Gain</span><input type='range' min='-45' max='0' id='gain'><span id='gainVal'></span></div>
    <div class='row'><span class='eq-label'>Balance</span><input type='range' min='-15' max='15' id='balance'><span id='balanceVal'></span></div>
  </div>
  <script>
    let state = {};
    let statusInFlight = false;
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
      
      fetch('/preset?' + params, {method: 'POST'})
        .then(response => {
          if (!response.ok) throw new Error('Network response was not ok');
          return response.text();
        })
        .then(() => {
          console.log('Preset stored successfully');
          return Promise.all([fetchPresets(), fetchStatus()]);
        })
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
          fetch('/freq?value=' + preset.freq, {method: 'POST'})
            .then(response => {
              if (!response.ok) throw new Error('Network response was not ok');
              return response.text();
            })
            .then(() => {
              console.log('Preset recalled successfully');
              // Update the UI to reflect the change
              updateUI();
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
                  fetchStatus(); // Refresh status to get actual state
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

    function fetchPresets() {
      fetch('/presets')
        .then(r => r.json())
        .then(arr => {
          presets = arr;
          presetsLoaded = true;
          renderPresets();
        })
        .catch(err => console.error('Error fetching presets:', err));
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

      // Update volume
      const volumeSlider = document.getElementById('volume');
      const volumeVal = document.getElementById('volumeVal');
      volumeSlider.value = state.volume;
      volumeVal.textContent = state.volume;

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
      if (state.source === 1) { // TUN
        freqRow.style.display = 'flex';
        rssiRow.style.display = 'flex';
        rdsRow.style.display = 'flex';
        seekRow.style.display = 'flex';
        presetRow.style.display = 'flex';
        rpiMetaRow.style.display = 'none';
        rpiCtrlRow.style.display = 'none';
        
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
        if (state.source === 3) { // Raspberry PI
          rpiMetaRow.style.display = 'flex';
          rpiCtrlRow.style.display = 'flex';
          document.getElementById('rpiOnlineVal').textContent = state.rpiOnline ? 'online' : 'offline';
          document.getElementById('rpiOnlineVal').style.color = state.rpiOnline ? '#27ae60' : '#e74c3c';
          document.getElementById('rpiStateVal').textContent = state.rpiState || '-';
          document.getElementById('rpiTitleVal').textContent = state.rpiTitle || '-';
          document.getElementById('rpiArtistVal').textContent = state.rpiArtist || '-';
          document.getElementById('rpiFileVal').textContent = state.rpiFile || '-';
          const rpiPlayBtn = document.getElementById('rpiPlayBtn');
          const rpiStateNorm = (state.rpiState || '').toLowerCase();
          // Button shows the next action: Pauza when playing, Play when paused/stopped.
          if (rpiStateNorm === 'play' || rpiStateNorm === 'playing') {
            rpiPlayBtn.textContent = 'Pauza';
          } else {
            rpiPlayBtn.textContent = 'Play';
          }
        } else {
          rpiMetaRow.style.display = 'none';
          rpiCtrlRow.style.display = 'none';
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

      // Update EQ values
      document.getElementById('bass').value = state.bass;
      document.getElementById('bassVal').textContent = state.bass;
      document.getElementById('middle').value = state.middle;
      document.getElementById('middleVal').textContent = state.middle;
      document.getElementById('treble').value = state.treble;
      document.getElementById('trebleVal').textContent = state.treble;
      document.getElementById('gain').value = state.gain;
      document.getElementById('gainVal').textContent = state.gain;
      document.getElementById('balance').value = state.balance;
      document.getElementById('balanceVal').textContent = state.balance;
      
      // Update presets only when frequency actually changed
      if (state.source === 1 && state.freq !== lastRenderedFreq) {
        lastRenderedFreq = state.freq;
        renderPresets();
      }
    }

    function fetchStatus() {
      if (statusInFlight) return;
      statusInFlight = true;
      fetch('/status')
        .then(r => r.json())
        .then(s => {
          state = s;
          if (state.source === 1 && !presetsLoaded) {
            fetchPresets();
          }
          updateUI();
        })
        .catch(err => {
          console.error('Error fetching status:', err);
          // Don't show error to user, just retry later
        })
        .finally(() => { statusInFlight = false; });
    }

    // Event handlers
    document.getElementById('powerBtn').onclick = () => {
      const goingToStandby = (state.power === 'on');
      fetch('/power?state=' + (goingToStandby ? 'off' : 'on'), {method: 'POST'})
        .then(() => {
          if (goingToStandby) {
            // If we just powered off, reload to show standby UI
            setTimeout(() => location.reload(), 300);
          } else {
            fetchStatus();
          }
        })
        .catch(err => console.error('Error toggling power:', err));
    };

    document.getElementById('muteBtn').onclick = () => {
      fetch('/mute?state=' + (state.mute === 'on' ? 'off' : 'on'), {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error toggling mute:', err));
    };

    // Debounce volume updates to avoid flooding the ESP with requests
    let volumeInputTimeout;
    document.getElementById('volume').addEventListener('input', (e) => {
      clearTimeout(volumeInputTimeout);
      const value = e.target.value;
      volumeInputTimeout = setTimeout(() => {
        fetch('/volume?value=' + value, {method: 'POST'})
          .then(() => fetchStatus())
          .catch(err => console.error('Error setting volume:', err));
      }, 200);
    });

    document.querySelectorAll('.source-btn').forEach(btn => {
      btn.onclick = (e) => {
        const source = parseInt(e.target.id.replace('src', ''));
        fetch('/source?value=' + source, {method: 'POST'})
          .then(() => {
            if (source === 1 && !presetsLoaded) fetchPresets();
            return fetchStatus();
          })
          .catch(err => console.error('Error setting source:', err));
      };
    });

    document.getElementById('freq').onchange = (e) => {
      fetch('/freq?value=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting frequency:', err));
    };

    function seek(dir) {
      fetch('/seek?dir=' + dir, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error seeking:', err));
    }

    function rpiCmd(cmd) {
      fetch('/rpi?cmd=' + encodeURIComponent(cmd), {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error sending RPI command:', err));
    }

    // EQ handlers
    document.getElementById('bass').oninput = (e) => {
      fetch('/eq?bass=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting bass:', err));
    };

    document.getElementById('middle').oninput = (e) => {
      fetch('/eq?middle=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting middle:', err));
    };

    document.getElementById('treble').oninput = (e) => {
      fetch('/eq?treble=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting treble:', err));
    };

    document.getElementById('gain').oninput = (e) => {
      fetch('/eq?gain=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting gain:', err));
    };

    document.getElementById('balance').oninput = (e) => {
      fetch('/eq?balance=' + e.target.value, {method: 'POST'})
        .then(() => fetchStatus())
        .catch(err => console.error('Error setting balance:', err));
    };

    document.getElementById('rpiPrevBtn').onclick = () => rpiCmd('PREV');
    document.getElementById('rpiPlayBtn').onclick = () => rpiCmd('PLAYPAUSE');
    document.getElementById('rpiNextBtn').onclick = () => rpiCmd('NEXT');

    // Initial load
    fetchStatus();
    // Update every 5 seconds
    setInterval(fetchStatus, 8000);

    // Add frequency input handling
    let freqInputTimeout;
    document.getElementById('freq').addEventListener('input', function(e) {
      // Clear any pending timeout
      clearTimeout(freqInputTimeout);
      
      // Set a new timeout to update the frequency after typing stops
      freqInputTimeout = setTimeout(() => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 87.5 && value <= 108.0) {
          fetch('/freq?value=' + value, {method: 'POST'})
            .then(() => fetchStatus())
            .catch(err => console.error('Error setting frequency:', err));
        }
      }, 500); // Wait 500ms after typing stops
    });

    // Add blur handler to update frequency when input loses focus
    document.getElementById('freq').addEventListener('blur', function(e) {
      const value = parseFloat(e.target.value);
      if (!isNaN(value) && value >= 87.5 && value <= 108.0) {
        fetch('/freq?value=' + value, {method: 'POST'})
          .then(() => fetchStatus())
          .catch(err => console.error('Error setting frequency:', err));
      }
    });
  </script>
</body>
</html>
)rawliteral");
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
      server.send(200, "text/plain", "Power ON");      
    } else if (state == "off") {
      // Respond immediately, perform work in main loop
      if (!standbyState) {
        standbyState = true;
        powerOffPending = true;
      }      
      server.send(200, "text/plain", "Power OFF");
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

// --- Web Control Endpoints ---
// Mute
void handleMute() {
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  if (server.method() == HTTP_POST && server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      if (!fmRadio.isMuted) fmRadio.toggleMute();
      server.send(200, "text/plain", "Muted");
    } else if (state == "off") {
      if (fmRadio.isMuted) fmRadio.toggleMute();
      server.send(200, "text/plain", "Unmuted");
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
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  if (server.method() == HTTP_POST && server.hasArg("value")) {
    int v = server.arg("value").toInt();
    v = constrain(v, MIN_VOLUME, MAX_VOLUME);
    
    if (fmRadio.currentVolume != v) {
      fmRadio.currentVolume = v;
      pendingVolume = v;
      volumeApplyPending = true;
      server.send(200, "text/plain", String(v));
    } else {
      server.send(200, "text/plain", String(v));
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
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  if (server.method() == HTTP_POST && server.hasArg("value")) {
    int s = server.arg("value").toInt();
    // Mapping: 1=TUN, 2=Bluetooth, 3=Raspberry PI, 4=PC
    Serial.print("[WEB SRC] Received source value: ");
    Serial.println(s);
    if (s >= 1 && s <= 4) {
      fmRadio.sursa = s;
      fmRadio.loadEqualizer();  // Load EQ for new source before any save
      // Defer heavy I2C apply to main loop to keep HTTP responsive
      audioApplyPending = true;
      audioApplyAt = millis();
      Serial.print("[WEB SRC] fmRadio.sursa is now ");
      Serial.println(fmRadio.sursa);
      uiRefreshPending = true;
      fmRadio.saveSettings();
      // Return EQ values as JSON
      String json = "{";
      json += "\"bass\":" + String(Bass) + ",";
      json += "\"middle\":" + String(Middle) + ",";
      json += "\"treble\":" + String(Treble) + ",";
      json += "\"gain\":" + String(Gain) + ",";
      json += "\"balance\":" + String(Balance) + "}";
      server.send(200, "application/json", json);
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
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
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
      server.send(200, "text/plain", String(f, 2));
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
  server.send(200, "text/plain", "OK");
}

// Seek
void handleSeek() {
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  if (server.method() == HTTP_POST && server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "up") {
      pendingSeekDir = 1;
      server.send(200, "text/plain", "Seek up");
    } else if (dir == "down") {
      pendingSeekDir = -1;
      server.send(200, "text/plain", "Seek down");
    } else {
      server.send(400, "text/plain", "Invalid dir");
    }
  } else {
    server.send(400, "text/plain", "Missing dir argument");
  }
}

// Raspberry PI serial commands
void handleRpi() {
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  if (server.method() != HTTP_POST || !server.hasArg("cmd")) {
    server.send(400, "text/plain", "Missing cmd argument");
    return;
  }

  String cmd = server.arg("cmd");
  cmd.toUpperCase();
  if (cmd == "PREV" || cmd == "NEXT" || cmd == "PLAYPAUSE" || cmd == "GET") {
    sendRpiCommand(cmd.c_str());
    server.send(200, "text/plain", "OK");
    return;
  }

  server.send(400, "text/plain", "Invalid cmd");
}

// Status (all info)
void handleStatus() {
  String json;
  json.reserve(512);
  json = "{";
  json += "\"power\":\"" + String(standbyState ? "off" : "on") + "\",";
  json += "\"mute\":\"" + String(fmRadio.isMuted ? "on" : "off") + "\",";
  json += "\"volume\":" + String(fmRadio.currentVolume) + ",";
  json += "\"source\":" + String(fmRadio.sursa) + ",";
  json += "\"freq\":" + String(fmRadio.currentFrequency, 2) + ",";
  // Cached SI4703 RSSI from main loop (avoid I2C here)
  int rssi = (!standbyState && si4703Powered && fmRadio.sursa == 1) ? (int)lastFmRssi : -1;
  json += "\"rssi\":" + String(rssi) + ",";
  // Add WiFi RSSI (dBm). If disconnected, return a sentinel like -127.
  int wifi = -127;
  if (WiFi.status() == WL_CONNECTED) {
    wifi = WiFi.RSSI();
  }
  json += "\"wifi\":" + String(wifi) + ",";
  // Add RDS RadioText when available
  String rds = "";
  if (!standbyState && si4703Powered && fmRadio.sursa == 1) {
    rds = String(fmRadio.radioTextnow);
    rds.trim();
  }
  // JSON-escape minimal characters
  String rdsEsc = rds;
  rdsEsc.replace("\\", "\\\\");
  rdsEsc.replace("\"", "\\\"");
  json += "\"rds\":\"" + rdsEsc + "\",";
  String rpiTitleEsc = rpiTitle;
  String rpiArtistEsc = rpiArtist;
  String rpiFileEsc = rpiFile;
  String rpiStateEsc = rpiState;
  rpiTitleEsc.replace("\\", "\\\\");
  rpiTitleEsc.replace("\"", "\\\"");
  rpiArtistEsc.replace("\\", "\\\\");
  rpiArtistEsc.replace("\"", "\\\"");
  rpiFileEsc.replace("\\", "\\\\");
  rpiFileEsc.replace("\"", "\\\"");
  rpiStateEsc.replace("\\", "\\\\");
  rpiStateEsc.replace("\"", "\\\"");
  json += "\"rpiOnline\":" + String(rpiConnected ? "true" : "false") + ",";
  json += "\"rpiState\":\"" + rpiStateEsc + "\",";
  json += "\"rpiTitle\":\"" + rpiTitleEsc + "\",";
  json += "\"rpiArtist\":\"" + rpiArtistEsc + "\",";
  json += "\"rpiFile\":\"" + rpiFileEsc + "\",";
  json += "\"bass\":" + String(Bass) + ",";
  json += "\"middle\":" + String(Middle) + ",";
  json += "\"treble\":" + String(Treble) + ",";
  json += "\"gain\":" + String(Gain) + ",";
  json += "\"balance\":" + String(Balance) + "}";
  server.send(200, "application/json", json);
}

// System info pentru standby UI - foloseste doar cache (fara I2C in handler ca sa nu blocheze web UI)
void handleSystemInfo() {
  int wifiRssi = -127;
  String wifiSsid = "";
  if (WiFi.status() == WL_CONNECTED) {
    wifiRssi = WiFi.RSSI();
    wifiSsid = WiFi.SSID();
  }
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "-";
  unsigned long uptimeSec = millis() / 1000;
  uint32_t freeHeap = ESP.getFreeHeap();

  String ssidEsc = wifiSsid;
  ssidEsc.replace("\\", "\\\\");
  ssidEsc.replace("\"", "\\\"");

  String json;
  json.reserve(320);
  json = "{";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"wifiRssi\":" + String(wifiRssi) + ",";
  json += "\"wifiSsid\":\"" + ssidEsc + "\",";
  json += "\"i2c\":{";
  json += "\"TDA7439\":" + String(cachedTdaOk ? "true" : "false") + ",";
  json += "\"RTC_DS3231\":" + String(cachedRtcOk ? "true" : "false") + ",";
  json += "\"MCP23017\":" + String(cachedMcpOk ? "true" : "false") + ",";
  json += "\"mcpAddr\":\"" + String(cachedMcpOk && mcpI2cAddr != 0 ? ("0x" + String(mcpI2cAddr, HEX)) : "-") + "\",";
  json += "\"SI4703\":" + String(cachedSi47Ok ? "true" : "false") + "},";
  json += "\"freeHeap\":" + String(freeHeap) + ",";
  json += "\"uptimeSec\":" + String(uptimeSec) + "}";
  server.send(200, "application/json", json);
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
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
  preferences.begin("settings", true);
  String json;
  json.reserve(512);
  json = "[";
  for (int i = 1; i <= 9; ++i) {
    String keyFreq = "preset" + String(i) + "_freq";
    String keyName = "preset" + String(i) + "_name";
    float freq = preferences.getFloat(keyFreq.c_str(), 0.0);
    String name = preferences.getString(keyName.c_str(), "");
    json += "{\"freq\":" + String(freq, 2) + ",\"name\":\"" + name + "\"}";
    if (i < 9) json += ",";
  }
  preferences.end();
  json += "]";
  server.send(200, "application/json", json);
}

void handlePreset() {
  if (standbyState) {
    server.send(403, "text/plain", "Device is in standby");
    return;
  }
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
    String name = preferences.getString(keyName.c_str(), "");
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
  Serial.println("[SI4703] Reinitializing after power cycle...");
  i2cBusRecover();
  // Perform hardware reset to guarantee 2-wire mode after rail power-up
  resetSI4703Hardware();
  // Verify presence
  si4703Powered = detectSI4703();
  if (!si4703Powered) {
    Serial.println("[SI4703] Not detected after reset; skipping radio init.");
    return false;
  }
  // Re-run library init/config
  fmRadio.initRadio();
  fmRadio.radio.setFrequency(fmRadio.currentFrequency * 100);
  fmRadio.radio.setVolume(fmRadio.currentVolume);
  fmRadio.radio.attachReceiveRDS(FMRadioController::RDS_process);
  Serial.println("[SI4703] Reinit complete.");
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
    case 4: tdaInput = 1; break; // PC -> IN1
    case 2: tdaInput = 2; break; // BT -> IN2
    case 1: tdaInput = 4; break; // Radio (TUN) -> IN3
    case 3: tdaInput = 3; break; // RPI -> IN4
    default: tdaInput = 1; break;
  }
  tda7439.setInput(tdaInput);
}

// Apply all persisted audio settings to TDA7439 for current source
void applyTDA7439SettingsForCurrentSource() {
  Wire.beginTransmission(0x44);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("[TDA7439] Apply settings error: ");
    Serial.println(error);
    return;
  }
  setTDA7439InputForSource(fmRadio.sursa);
  delay(2);
  // Apply gain first, then tone, then volume
  int tdaGain = (Gain + 45) / 3; // 0..15
  tda7439.inputGain(tdaGain);
  Serial.print("[TDA7439] Applying input gain: ");
  Serial.println(tdaGain);
  delay(2);
  tda7439.setSnd(Bass, 1);
  delay(2);
  tda7439.setSnd(Middle, 2);
  delay(2);
  tda7439.setSnd(Treble, 3);
  delay(2);
  tda7439.setVolume(fmRadio.currentVolume);
  delay(2);
  tda7439.spkAtt(0);
  delay(2);
  setTDA7439Balance((int8_t)Balance);
  Serial.println("[TDA7439] Applied saved input/volume/EQ/gain/balance");
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
  tda7439Send(0x03, val & 0x0F); // 0x03 = treble
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
