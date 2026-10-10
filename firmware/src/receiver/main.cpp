#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "audio_packet.h"

// Wi-Fi name and password live in include/wifi_credentials.h, which git ignores
// so the password never reaches GitHub.
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#error "Copy include/wifi_credentials.example.h to include/wifi_credentials.h and fill in your Wi-Fi name and password"
#endif

// --- 1. SH1106 1.3" OLED DISPLAY CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define i2c_Address 0x3C // Standard I2C address for 1.3" SH1106 OLED modules

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- 2. HARDWARE PIN ASSIGNMENTS ---
const int DAC_AUDIO_OUT = 25; // Internal DAC1 connected to 100k Potentiometer Pin 1
// No optical sense pin: whether the beam is on the panel now comes from the host (section 3).

// --- 3. HOST LINK COMMANDS (UDP over Wi-Fi) ---
// tracker_bridge.py tracks the ArUco target with the camera and picks the link.
// It sends one short text datagram to HOST_CMD_PORT:
//   "L<offset>"  target in view (centred or not)  -> LiFi, DAC stays silent
//   "R<offset>"  target lost by the camera        -> RF failover, play ESP-NOW audio on the DAC
// <offset> is the target's distance from the crosshair in pixels, -1 when it is lost.
// The receiver only acts on the letter; the offset is for anyone watching the packets.
// The host repeats the command every ~100 ms. If it goes quiet for HOST_TIMEOUT_MS,
// the beam position is unknown, so the receiver fails over to RF on its own.
const uint16_t HOST_CMD_PORT = 4210; // must match RECEIVER_PORT in tracker_bridge.py
const unsigned long HOST_TIMEOUT_MS = 1000;

WiFiUDP hostUdp;
bool udpStarted = false;
bool hostSeen = false;
unsigned long lastHostCmdMs = 0;
int shownDots = 0; // How many animated dots the OLED is showing now (0-3)

// --- 4. ESP-NOW DATA STRUCTURE ---
// struct_audio_packet is defined in include/audio_packet.h, shared with the sender
// so both boards always agree on the packet layout.
struct_audio_packet incomingAudio;
// Read by the ESP-NOW callback (Wi-Fi task), written by loop().
// Starts on RF until the host confirms the beam is aligned.
volatile bool isLaserActive = false;

// --- 5. ESP-NOW RECEIVE CALLBACK FUNCTION (Cross-Core Compatible) ---
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingDataBytes, int len)
{
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataBytes, int len)
{
#endif
  // Ignore anything that isn't exactly one audio packet (e.g. from another ESP-NOW device)
  if (len != sizeof(incomingAudio))
  {
    return;
  }
  memcpy(&incomingAudio, incomingDataBytes, sizeof(incomingAudio));

  // Never trust a count from the radio: clamp it to the buffer size
  int sampleCount = min((int)incomingAudio.bufferSize, AUDIO_SAMPLES_PER_PACKET);

  // Output digital audio to DAC (GPIO 25) ONLY when the host says the beam is off target
  if (!isLaserActive)
  {
    for (int i = 0; i < sampleCount; i++)
    {
      // Samples are already 8-bit (0-255): the sender shifts them down before sending
      uint8_t dacValue = incomingAudio.sampleBuffer[i];
      dacWrite(DAC_AUDIO_OUT, dacValue); // Output backup RF audio on GPIO 25
      delayMicroseconds(125);            // ~8 kHz audio playback rate
    }
  }
}

void setLinkMode(bool laser, const char *reason)
{
  if (laser == isLaserActive)
    return;
  isLaserActive = laser;
  Serial.printf("Link -> %s (%s)\n", laser ? "LiFi (laser)" : "RF (ESP-NOW)", reason);
}

void readHostCommands()
{
  while (hostUdp.parsePacket() > 0)
  {
    char msg[16];
    int len = hostUdp.read(msg, sizeof(msg) - 1);
    if (len <= 0)
      continue;
    msg[len] = '\0';

    // Ignore anything that isn't one of the host's link commands
    if (msg[0] != 'L' && msg[0] != 'R')
      continue;

    lastHostCmdMs = millis();
    hostSeen = true;
    setLinkMode(msg[0] == 'L', "host command");
  }
}

// Prints text centred across the screen. The built-in font is 6 x 8 pixels per
// character at size 1 (5 for the letter + 1 gap), and scales with the text size.
void printCentered(const char *text, int y, uint8_t size)
{
  int width = strlen(text) * 6 * size;
  display.setTextSize(size);
  display.setCursor((SCREEN_WIDTH - width) / 2, y);
  display.print(text);
}

// Branded screen. Link details aren't shown here: they go to the serial monitor
// (115200 baud), which also prints the IP address tracker_bridge.py needs.
void drawSplash(int dots)
{
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextWrap(false);

  printCentered("AEROBEAM", 4, 2);       // 8 chars x 12 px = 96 px wide, 16 px tall
  printCentered("TECH RECEIVER", 24, 1); // 13 chars x 6 px = 78 px
  display.drawFastHLine(8, 35, SCREEN_WIDTH - 16, SH110X_WHITE);
  printCentered("Seamless connection", 41, 1); // 19 chars = 114 px, the most that fits with a margin

  // "...", "..", "." or "" under it; a fixed 3-character slot so the dots don't slide around
  char dotText[4] = "   ";
  for (int i = 0; i < dots; i++)
    dotText[i] = '.';
  printCentered(dotText, 53, 1);

  display.display();
}

void setup()
{
  Serial.begin(115200);
  // Says which firmware this board runs, and when it was compiled
  Serial.printf("\nAeroBeam RECEIVER firmware, built %s %s\n", __DATE__, __TIME__);

  // Initialize I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(21, 22);

  // Initialize SH1106 Display, and put the branded screen up straight away
  delay(100); // Stabilization delay for display controller
  display.begin(i2c_Address, true);
  drawSplash(shownDots);

  // Join the LAN so the host can send link commands. ESP-NOW keeps working alongside,
  // but only on the router's channel, so the sender joins the same network too.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // modem sleep would delay every host command by up to ~100 ms
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  // Start listening for the host once Wi-Fi has given us an IP address
  if (!udpStarted && WiFi.status() == WL_CONNECTED)
  {
    hostUdp.begin(HOST_CMD_PORT);
    udpStarted = true;
    Serial.print("Wi-Fi joined, listening for host on ");
    Serial.print(WiFi.localIP());
    Serial.printf(":%u (channel %ld)\n", HOST_CMD_PORT, (long)WiFi.channel());
  }

  if (udpStarted)
  {
    readHostCommands();
  }

  // Host silent: the beam position is unknown, so fail over to RF
  bool hostOnline = hostSeen && (millis() - lastHostCmdMs <= HOST_TIMEOUT_MS);
  if (!hostOnline)
  {
    setLinkMode(false, "host silent");
  }

  // --- OLED: only the dots move, one step every 0.5 s ---
  // Sending a frame to the screen takes ~25 ms, so redraw only when the picture changes
  int dots = (millis() / 500) % 4;
  if (dots != shownDots)
  {
    shownDots = dots;
    drawSplash(dots);
  }

  delay(5); // poll host commands every ~5 ms
}
