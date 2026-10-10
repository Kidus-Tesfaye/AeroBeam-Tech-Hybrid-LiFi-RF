#pragma once

// Template for include/wifi_credentials.h. Copy this file to that name and fill it in.
// wifi_credentials.h is git-ignored, so your password stays on your PC.
//
// Both boards join this network: the receiver to take link commands from
// tracker_bridge.py, the sender so ESP-NOW runs on the same channel.
// It must be a 2.4 GHz network (the ESP32 has no 5 GHz radio), and the
// laptop running tracker_bridge.py must be on it too.
#define WIFI_SSID "your-network-name"
#define WIFI_PASSWORD "your-password"
