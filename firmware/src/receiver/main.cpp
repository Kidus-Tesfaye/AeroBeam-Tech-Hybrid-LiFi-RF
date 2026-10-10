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
// tracker_bridge.py tracks the ArUco target with the camera and decides whether the
// laser is on the panel. It sends one short text datagram to HOST_CMD_PORT:
//   "L<offset>"  beam aligned     -> LiFi, DAC stays silent
//   "R<offset>"  beam off target  -> RF failover, play ESP-NOW audio on the DAC
// <offset> is the target's distance from the crosshair in pixels, -1 when it is lost.
// The host repeats the command every ~100 ms. If it goes quiet for HOST_TIMEOUT_MS,
// the beam position is unknown, so the receiver fails over to RF on its own.
const uint16_t HOST_CMD_PORT = 4210; // must match RECEIVER_PORT in tracker_bridge.py
const unsigned long HOST_TIMEOUT_MS = 1000;

WiFiUDP hostUdp;
bool udpStarted = false;
bool hostSeen = false;
unsigned long lastHostCmdMs = 0;
int hostOffsetPx = -1;
unsigned long lastDisplayMs = 0;

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

    hostOffsetPx = atoi(msg + 1);
    lastHostCmdMs = millis();
    hostSeen = true;
    setLinkMode(msg[0] == 'L', "host command");
  }
}

void drawHud(bool hostOnline)
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("AEROBEAM RECEIVER");
  display.drawFastHLine(0, 10, 128, SH110X_WHITE);

  display.setCursor(0, 14);
  display.print("Link: ");
  display.println(isLaserActive ? "LiFi (Laser)" : "RF (ESP-NOW)");

  display.setCursor(0, 26);
  display.print("Host: ");
  if (!hostSeen)
  {
    display.println("waiting");
  }
  else if (!hostOnline)
  {
    display.println("TIMEOUT");
  }
  else if (hostOffsetPx < 0)
  {
    display.println("target lost");
  }
  else
  {
    display.print("off ");
    display.print(hostOffsetPx);
    display.println(" px");
  }

  // The IP shown here is what RECEIVER_IP in tracker_bridge.py must be set to
  display.setCursor(0, 38);
  if (WiFi.status() == WL_CONNECTED)
  {
    display.print("IP ");
    display.println(WiFi.localIP());
  }
  else
  {
    display.println("WiFi: connecting...");
  }

  display.setCursor(0, 52);
  display.println(isLaserActive ? "SECURE OPTICAL LINK" : "RF FAILOVER (AUDIO)");

  display.display();
}

void setup()
{
  Serial.begin(115200);
  // Says which firmware this board runs, and when it was compiled
  Serial.printf("\nAeroBeam RECEIVER firmware, built %s %s\n", __DATE__, __TIME__);

  // Initialize I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(21, 22);

  // Initialize SH1106 Display
  delay(100); // Stabilization delay for display controller
  display.begin(i2c_Address, true);

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

  // Startup Screen HUD
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("AeroBeam Receiver");
  display.println("SH1106 Display OK");
  display.display();
  delay(1000);
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

  // --- UPDATE OLED HUD DISPLAY (10 Hz) ---
  if (millis() - lastDisplayMs >= 100)
  {
    lastDisplayMs = millis();
    drawHud(hostOnline);
  }

  delay(5); // poll host commands every ~5 ms
}
