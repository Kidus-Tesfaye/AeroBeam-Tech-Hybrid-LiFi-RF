#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "audio_packet.h"

// --- 1. SH1106 1.3" OLED DISPLAY CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define i2c_Address 0x3C // Standard I2C address for 1.3" SH1106 OLED modules

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- 2. HARDWARE PIN ASSIGNMENTS ---
const int OPTICAL_SENSE_PIN = 34; // Tapped directly from Solar Panel Positive (+) line
const int DAC_AUDIO_OUT = 25;     // Internal DAC1 connected to 100k Potentiometer Pin 1

// --- 3. ESP-NOW DATA STRUCTURE ---
// struct_audio_packet is defined in include/audio_packet.h, shared with the sender
// so both boards always agree on the packet layout.
struct_audio_packet incomingAudio;
bool isLaserActive = true;
float signalVoltage = 0.0;

// --- 4. ESP-NOW RECEIVE CALLBACK FUNCTION (Cross-Core Compatible) ---
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

  // Output digital audio to DAC (GPIO 25) ONLY when the optical beam is blocked
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

void setup()
{
  Serial.begin(115200);

  // Initialize I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL)
  Wire.begin(21, 22);

  // Initialize SH1106 Display
  delay(100); // Stabilization delay for display controller[cite: 1]
  display.begin(i2c_Address, true);

  // Initialize ESP-NOW in Wi-Fi Station Mode[cite: 1, 7]
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW receive callback[cite: 1, 7]
  esp_now_register_recv_cb(OnDataRecv);

  // Startup Screen HUD[cite: 1]
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
  // Read optical voltage directly from Solar Panel (+) line on GPIO 34[cite: 1, 7]
  int rawSignal = analogRead(OPTICAL_SENSE_PIN);
  signalVoltage = (rawSignal / 4095.0) * 3.3;

  // Link Threshold: Voltage > 0.3V indicates laser beam alignment[cite: 1, 7]
  isLaserActive = (signalVoltage > 0.3);

  // --- UPDATE OLED HUD DISPLAY ---[cite: 1]
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("AEROBEAM TECH RECEIVER");
  display.drawFastHLine(0, 10, 128, SH110X_WHITE);

  display.setCursor(0, 18);
  display.print("Active Link: ");
  if (isLaserActive)
  {
    display.println("LiFi (Laser)");
  }
  else
  {
    display.println("RF (ESP-NOW)");
  }

  display.setCursor(0, 32);
  display.print("Opt Voltage: ");
  display.print(signalVoltage, 2);
  display.println(" V");

  display.setCursor(0, 48);
  if (isLaserActive)
  {
    display.println("STATUS: SECURE OPTICAL");
  }
  else
  {
    display.println("STATUS: RF FAILOVER (AUDIO)");
  }

  display.display();
  delay(100); // 10 Hz refresh rate
}
