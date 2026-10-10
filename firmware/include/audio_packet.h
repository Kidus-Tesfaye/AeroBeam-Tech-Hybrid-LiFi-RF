#pragma once
#include <stdint.h>

const int AUDIO_SAMPLES_PER_PACKET = 200; // 200 + 2 = 202 bytes, under ESP-NOW's 250

typedef struct struct_audio_packet
{
    uint8_t sampleBuffer[AUDIO_SAMPLES_PER_PACKET]; // 8-bit samples: 200 bytes
    uint16_t bufferSize;                            // how many samples are valid in the buffer (0-200)
} struct_audio_packet;

static_assert(sizeof(struct_audio_packet) <= 250, "audio packet too big for one ESP-NOW message (max 250 bytes)");
