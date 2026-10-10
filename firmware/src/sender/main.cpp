#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include "audio_packet.h"

Servo panServo;
Servo tiltServo;

// ESP32 GPIO assignments
const int PAN_PIN = 18;     // Pan Servo Pin
const int TILT_PIN = 19;    // Tilt Servo Pin
const int MIC_ADC_PIN = 34; // MAX4466 Audio OUT Split to GPIO 34

// Servo pulse limits
const int PULSE_MIN = 500;
const int PULSE_MAX = 2400;

// Angles in TENTHS of a degree (900 = 90.0 deg)
int panAngle = 900;
int tiltAngle = 900;

// Non-blocking serial line buffer
char buf[32];
uint8_t bufLen = 0;

// --- ESP-NOW CONFIGURATION ---
// Replace with the MAC Address of your RECEIVER ESP32
uint8_t receiverMacAddress[] = {0xB0, 0x3F, 0xD3, 0x70, 0x1E, 0xE4};

struct_audio_packet outgoingAudio;
esp_now_peer_info_t peerInfo;

int angleToUs(int tenths)
{
  return map(tenths, 0, 1800, PULSE_MIN, PULSE_MAX);
}

void parseAndApply(const char *msg)
{
  // Expected format: "P<pan>T<tilt>"  e.g. "P905T900"
  if (msg[0] != 'P')
    return;

  const char *t = strchr(msg, 'T');
  if (t == NULL)
    return;

  int newPan = atoi(msg + 1);
  int newTilt = atoi(t + 1);

  panAngle = constrain(newPan, 0, 1800);
  tiltAngle = constrain(newTilt, 0, 1800);

  panServo.writeMicroseconds(angleToUs(panAngle));
  tiltServo.writeMicroseconds(angleToUs(tiltAngle));
}

void setup()
{
  Serial.begin(115200);
  // Says which firmware this board runs, and when it was compiled
  Serial.printf("\nAeroBeam SENDER firmware, built %s %s\n", __DATE__, __TIME__);

  // Attach servos
  panServo.attach(PAN_PIN, PULSE_MIN, PULSE_MAX);
  tiltServo.attach(TILT_PIN, PULSE_MIN, PULSE_MAX);

  // Center both axes (90.0 deg = 1450 us)
  panServo.writeMicroseconds(angleToUs(panAngle));
  tiltServo.writeMicroseconds(angleToUs(tiltAngle));

  // --- INITIALIZE ESP-NOW (Wi-Fi) ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register Receiver Peer
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add receiver peer");
    return;
  }
}

void loop()
{
  // --- 1. NON-BLOCKING SERVO TRACKING SERIAL PARSER ---
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == '\n')
    {
      buf[bufLen] = '\0';
      if (bufLen > 0)
        parseAndApply(buf);
      bufLen = 0;
    }
    else if (c != '\r')
    {
      if (bufLen < sizeof(buf) - 1)
      {
        buf[bufLen++] = c;
      }
      else
      {
        bufLen = 0; // overflow: discard garbled line
      }
    }
  }

  // --- 2. SAMPLE AUDIO FROM MIC (GPIO 34) & TRANSMIT OVER ESP-NOW ---
  for (int i = 0; i < AUDIO_SAMPLES_PER_PACKET; i++)
  {
    outgoingAudio.sampleBuffer[i] = analogRead(MIC_ADC_PIN) >> 4; // 0-4095 -> 0-255
    delayMicroseconds(125);
  }
  outgoingAudio.bufferSize = AUDIO_SAMPLES_PER_PACKET;

  // Broadcast digitized audio packet over 2.4 GHz ESP-NOW
  esp_now_send(receiverMacAddress, (uint8_t *)&outgoingAudio, sizeof(outgoingAudio));
}
