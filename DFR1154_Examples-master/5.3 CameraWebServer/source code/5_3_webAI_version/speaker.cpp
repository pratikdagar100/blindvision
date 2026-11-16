// =====================================================
// BLINDVISION ON-BOARD SPEAKER
// MAX98357 I2S amplifier on the DFR1154 (same pins as DFRobot example 5.2)
// Plays 8-bit unsigned mono PCM @ 22050 Hz that the browser renders
// (sam-js text-to-speech) and POSTs to /speak.
// =====================================================
#include <Arduino.h>
#include "ESP_I2S.h"

#define SPK_BCLK        45
#define SPK_LRC         46
#define SPK_DOUT        42
#define SPK_SAMPLE_RATE 22050

struct SpeakerClip {
  uint8_t *buf;
  size_t len;
  int volume;  // 0..100
};

static I2SClass spkI2S;
static QueueHandle_t spkQueue = NULL;
static volatile bool spkBusy = false;
static bool spkReady = false;

static void speakerTask(void *arg) {
  SpeakerClip clip;
  static int16_t out[256];

  for (;;) {
    if (xQueueReceive(spkQueue, &clip, portMAX_DELAY) != pdTRUE) continue;

    int gain = clip.volume * 256 / 100;  // 8-bit -> 16-bit scaling

    for (size_t i = 0; i < clip.len; i += 256) {
      size_t n = clip.len - i;
      if (n > 256) n = 256;
      for (size_t j = 0; j < n; j++) {
        int32_t s = ((int32_t)clip.buf[i + j] - 128) * gain;
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        out[j] = (int16_t)s;
      }
      spkI2S.write((uint8_t *)out, n * sizeof(int16_t));
    }

    // Flush DMA buffers with silence so the last samples don't repeat
    memset(out, 0, sizeof(out));
    for (int k = 0; k < 16; k++) {
      spkI2S.write((uint8_t *)out, sizeof(out));
    }

    free(clip.buf);
    spkBusy = false;
  }
}

bool initSpeaker() {
  Serial.println("\n--- SPEAKER INITIALIZATION ---");

  spkI2S.setPins(SPK_BCLK, SPK_LRC, SPK_DOUT);
  if (!spkI2S.begin(I2S_MODE_STD, SPK_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("MAX98357 speaker init FAILED!");
    return false;
  }

  spkQueue = xQueueCreate(1, sizeof(SpeakerClip));
  xTaskCreatePinnedToCore(speakerTask, "speaker", 4096, NULL, 2, NULL, 1);

  spkReady = true;
  Serial.println("Speaker ready - playing startup chime.");

  // Startup chime (two rising beeps) so you can hear the speaker works
  const size_t toneLen = SPK_SAMPLE_RATE / 6;  // ~0.17 s each
  uint8_t *chime = (uint8_t *)malloc(toneLen * 2);
  if (chime) {
    for (size_t i = 0; i < toneLen * 2; i++) {
      float f = (i < toneLen) ? 880.0f : 1320.0f;
      float env = 1.0f - (float)(i % toneLen) / toneLen;  // fade out
      chime[i] = (uint8_t)(128 + 100 * env * sinf(2 * PI * f * i / SPK_SAMPLE_RATE));
    }
    extern bool speakerPlayU8(uint8_t *, size_t, int);
    speakerPlayU8(chime, toneLen * 2, 70);
  }
  return true;
}

bool speakerIsBusy() {
  return !spkReady || spkBusy;
}

// Takes ownership of buf (freed after playback, or immediately on failure).
bool speakerPlayU8(uint8_t *buf, size_t len, int volume) {
  if (!spkReady || spkBusy) {
    free(buf);
    return false;
  }
  if (volume < 0) volume = 0;
  if (volume > 100) volume = 100;

  SpeakerClip clip = {buf, len, volume};
  spkBusy = true;
  if (xQueueSend(spkQueue, &clip, 0) != pdTRUE) {
    spkBusy = false;
    free(buf);
    return false;
  }
  return true;
}
