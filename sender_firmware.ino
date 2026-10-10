#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

Servo panServo;
Servo tiltServo;

// ESP32 GPIO assignments
const int PAN_PIN  = 18;  // Pan Servo Pin[cite: 3, 6, 7]
const int TILT_PIN = 19;  // Tilt Servo Pin[cite: 1, 3, 7]
const int MIC_ADC_PIN = 34; // MAX4466 Audio OUT Split to GPIO 34[cite: 7]

// Servo pulse limits
const int PULSE_MIN = 500;
const int PULSE_MAX = 2400;

// Angles in TENTHS of a degree (900 = 90.0 deg)
int panAngle  = 900;
int tiltAngle = 900;

// Non-blocking serial line buffer
char buf[32];
uint8_t bufLen = 0;

// --- ESP-NOW CONFIGURATION ---
// Replace with the MAC Address of your RECEIVER ESP32
uint8_t receiverMacAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};[cite: 7]

typedef struct struct_audio_packet {
  uint16_t sampleBuffer[128]; // Array holding 128 digitized audio samples[cite: 7]
  uint16_t bufferSize;
} struct_audio_packet;

struct_audio_packet outgoingAudio;
esp_now_peer_info_t peerInfo;

int angleToUs(int tenths) {
  return map(tenths, 0, 1800, PULSE_MIN, PULSE_MAX);
}

void parseAndApply(const char *msg) {
  // Expected format: "P<pan>T<tilt>"  e.g. "P905T900"
  if (msg[0] != 'P') return;

  const char *t = strchr(msg, 'T');
  if (t == NULL) return;

  int newPan  = atoi(msg + 1);
  int newTilt = atoi(t + 1);

  panAngle  = constrain(newPan,  0, 1800);
  tiltAngle = constrain(newTilt, 0, 1800);

  panServo.writeMicroseconds(angleToUs(panAngle));
  tiltServo.writeMicroseconds(angleToUs(tiltAngle));
}

void setup() {
  Serial.begin(115200);

  // Attach servos
  panServo.attach(PAN_PIN, PULSE_MIN, PULSE_MAX);
  tiltServo.attach(TILT_PIN, PULSE_MIN, PULSE_MAX);

  // Center both axes (90.0 deg = 1450 us)
  panServo.writeMicroseconds(angleToUs(panAngle));
  tiltServo.writeMicroseconds(angleToUs(tiltAngle));

  // --- INITIALIZE ESP-NOW (Wi-Fi) ---
  WiFi.mode(WIFI_STA);[cite: 7]
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register Receiver Peer[cite: 7]
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add receiver peer");
    return;
  }
}

void loop() {
  // --- 1. NON-BLOCKING SERVO TRACKING SERIAL PARSER ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      buf[bufLen] = '\0';
      if (bufLen > 0) parseAndApply(buf);
      bufLen = 0;
    } else if (c != '\r') {
      if (bufLen < sizeof(buf) - 1) {
        buf[bufLen++] = c;
      } else {
        bufLen = 0;  // overflow: discard garbled line
      }
    }
  }

  // --- 2. SAMPLE AUDIO FROM MIC (GPIO 34) & TRANSMIT OVER ESP-NOW ---
  for (int i = 0; i < 128; i++) {
    outgoingAudio.sampleBuffer[i] = analogRead(MIC_ADC_PIN); // Digitizes 0-3.3V into 0-4095[cite: 7]
    delayMicroseconds(125); // ~8 kHz audio sampling rate delay
  }
  outgoingAudio.bufferSize = 128;

  // Broadcast digitized audio packet over 2.4 GHz ESP-NOW[cite: 7]
  esp_now_send(receiverMacAddress, (uint8_t *) &outgoingAudio, sizeof(outgoingAudio));
}
