// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * USB Audio Class (UAC) sink — ESP32 as a USB speaker.
 *
 * The board enumerates on the attached computer as a stereo USB audio
 * output device.  PCM arriving from the host is buffered and written to
 * the same I2S path AirPlay uses, so both sources share the DAC, its DSP
 * and its volume control.
 *
 * Only one source may drive I2S at a time: the state callback lets the
 * application suspend AirPlay while the host is streaming and resume it
 * once the stream has been idle for CONFIG_USB_AUDIO_SINK_IDLE_MS.
 *
 * Note this is the opposite direction to audio_output_usb.c, which
 * presents the board as a USB *microphone* fed by AirPlay.  The two are
 * mutually exclusive.
 */

#include "usb_audio_sink.h"

#include "audio_output.h"
#include "dac.h"
#include "display.h"
#include "led.h"
#include "playback_events.h"
#include "settings.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "usb_device_uac.h"

#if CONFIG_USB_DEVICE_UAC_AS_PART
#include "usb_desc_itf.h"
#endif

#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "usb_sink";

#if CONFIG_UAC_SAMPLE_RATE != CONFIG_OUTPUT_SAMPLE_RATE_HZ
#error \
    "CONFIG_UAC_SAMPLE_RATE must match CONFIG_OUTPUT_SAMPLE_RATE_HZ — the UAC descriptor advertises one fixed rate and the sink does not resample"
#endif

#if CONFIG_UAC_SPEAKER_CHANNEL_NUM != 2
#error "CONFIG_USB_AUDIO_SINK requires CONFIG_UAC_SPEAKER_CHANNEL_NUM=2"
#endif

#if !defined(CONFIG_UAC_BIT_RESOLUTION_16)
#error "CONFIG_USB_AUDIO_SINK requires CONFIG_UAC_BIT_RESOLUTION_16"
#endif

#define BYTES_PER_FRAME 4 // 16-bit stereo

// ~100 ms of slack between the USB stack and I2S, primed with ~20 ms so
// the first writes don't immediately underrun.
#define RINGBUF_SIZE \
  ((size_t)(CONFIG_OUTPUT_SAMPLE_RATE_HZ / 10) * BYTES_PER_FRAME)
#define RINGBUF_PREFETCH \
  ((size_t)(CONFIG_OUTPUT_SAMPLE_RATE_HZ / 50) * BYTES_PER_FRAME)

#define IDLE_TIMEOUT_US ((int64_t)CONFIG_USB_AUDIO_SINK_IDLE_MS * 1000)

// usb_device_uac reads each 1 ms packet into a single buffer from the USB
// interrupt and wakes its speaker task, which calls uac_output_cb(); a packet
// that comes before the task has run overwrites the one before it, and only
// the gap between our callbacks shows it.  Each callback runs less than 1 ms
// after its packet, or a later one would have been delivered instead, so a
// gap of g ms means at least floor(g) - 1 ms of audio never reached us.  The
// task may have run late, the interrupt may have been held off, or the host
// may have sent nothing; the gap alone does not say which.  The first
// delivery after the host starts is the component's prefill of several
// packets, and has no gap to measure.
#define USB_BYTES_PER_MS \
  ((size_t)(CONFIG_UAC_SAMPLE_RATE / 1000) * BYTES_PER_FRAME)

#define SINK_TASK_STACK 4096
#define SINK_TASK_PRIO  AUDIO_PLAYBACK_TASK_PRIORITY
#if CONFIG_FREERTOS_UNICORE
#define SINK_TASK_CORE 0
#else
#define SINK_TASK_CORE 1
#endif

#define VOLUME_MIN_DB (-30.0f)
#define VOLUME_MAX_DB 0.0f

// Clamp USB volume to the supported DAC control range.
static float usb_safe_volume_db(float db) {
  if (db < VOLUME_MIN_DB) return VOLUME_MIN_DB;
  if (db > VOLUME_MAX_DB) return VOLUME_MAX_DB;
  return db;
}

static RingbufHandle_t s_ringbuf;
static TaskHandle_t s_task;
static usb_audio_sink_state_cb_t s_state_cb;

// Cross-task USB/I2S handover control.
// The mode controller requests release; the sink task owns the actual
// transition and acknowledges it by clearing s_streaming.
static _Atomic bool s_takeover_allowed = true;
static _Atomic bool s_release_requested = false;
static _Atomic bool s_release_acknowledged = false;

static volatile bool s_streaming;
static volatile int64_t s_last_rx_us;
static uint32_t s_dropped;
static uint32_t s_underruns;
static _Atomic uint32_t s_rx_bytes;
static int64_t s_stats_us;
// Audio that never reached uac_output_cb() this stream, and the longest gap
// between two callbacks since the last stats line.  Written on the
// component's task, read and reset on the sink task.
static _Atomic uint32_t s_missing_ms;
static _Atomic uint32_t s_gap_max_us;
// Sink task only: what has been reported, and the stream's longest gap.
static uint32_t s_missing_logged;
static uint32_t s_stream_gap_max_us;

#define STATS_INTERVAL_US 2000000

// Host feature-unit updates are handled on the TinyUSB task during a
// control transfer; defer the (I2C) DAC writes to the sink task so USB
// never stalls behind the bus.  -1 means "nothing pending".
static _Atomic int32_t s_pending_volume = -1;
static _Atomic int32_t s_pending_mute = -1;
static float s_unmuted_db = -15.0f;
static float s_last_effective_usb_db = -15.0f;
static bool s_usb_volume_guard = true;
static bool s_usb_trim_enabled = false;

// USB-only level alignment: +1.5 dB at the volume API corresponds
// to approximately +3 dB at the TAS5825M in its normal range.
// Never apply this boost while the reconnect safety guard is active.
#define USB_LEVEL_TRIM_DB 0.0f

static float usb_output_volume_db(void) {
  float db = usb_safe_volume_db(s_unmuted_db);

  // Apply alignment only above the conservative startup range.
  // Keep the transition continuous to avoid a sudden volume jump.
  if (s_usb_trim_enabled && !s_usb_volume_guard && db > -20.0f) {
    float trim = db + 20.0f;

    if (trim > USB_LEVEL_TRIM_DB) {
      trim = USB_LEVEL_TRIM_DB;
    }

    db = usb_safe_volume_db(db + trim);
  }

  return db;
}
static _Atomic int32_t s_local_volume_steps = 0;
static int32_t s_last_host_volume = -1;
static bool s_usb_volume_relative = false;
static float s_usb_volume_offset_db = 0.0f;
static _Atomic bool s_usb_disconnected = false;

void usb_audio_sink_notify_local_volume(int direction) {
  if (direction > 0) {
    atomic_fetch_add(&s_local_volume_steps, 1);
  } else if (direction < 0) {
    atomic_fetch_sub(&s_local_volume_steps, 1);
  }
}

void usb_audio_sink_notify_disconnected(void) {
  // Discard controls belonging to the disconnected USB session.
  atomic_store(&s_pending_volume, -1);
  atomic_store(&s_pending_mute, -1);
  atomic_store(&s_local_volume_steps, 0);
  atomic_store(&s_usb_disconnected, true);
}

// Latched mute state, owned by the sink task.
static bool s_muted;

// ============================================================================
// UAC callbacks
// ============================================================================

// Host -> device PCM.  Runs on the component's usb_spk_task (priority
// CONFIG_UAC_SPK_TASK_PRIORITY), not in interrupt context.
static esp_err_t uac_output_cb(uint8_t *buf, size_t len, void *cb_ctx) {
  (void)cb_ctx;

  // Snapshot the handle: usb_audio_sink_init() may still be tearing down.
  RingbufHandle_t ringbuf = s_ringbuf;
  if (ringbuf == NULL || len == 0) {
    return ESP_OK;
  }

  const int64_t now = esp_timer_get_time();
  if (len <= USB_BYTES_PER_MS) {
    const int64_t gap_us = now - s_last_rx_us;
    if (gap_us >= 2000) {
      atomic_fetch_add(&s_missing_ms, (uint32_t)(gap_us / 1000 - 1));
    }
    if (gap_us > (int64_t)atomic_load(&s_gap_max_us)) {
      atomic_store(&s_gap_max_us, (uint32_t)gap_us);
    }
  }
  s_last_rx_us = now;
  atomic_fetch_add(&s_rx_bytes, (uint32_t)len);
  led_audio_feed((const int16_t *)buf, len / BYTES_PER_FRAME);

  if (xRingbufferSend(ringbuf, buf, len, 0) != pdTRUE) {
    // Either the sink task is still handing I2S over from AirPlay, or the
    // host is running ahead of our clock.  Dropping is preferable to
    // blocking the USB stack.
    s_dropped++;
  }

  if (!s_streaming && s_task != NULL) {
    xTaskNotifyGive(s_task);
  }
  return ESP_OK;
}

static void uac_set_mute_cb(uint32_t mute, void *cb_ctx) {
  (void)cb_ctx;
  atomic_store(&s_pending_mute, mute ? 1 : 0);
}

// The component hands us 0..100 (mapped from the host's -50..0 dB).
static void uac_set_volume_cb(uint32_t volume, void *cb_ctx) {
  (void)cb_ctx;
  if (volume > 100) {
    volume = 100;
  }
  ESP_LOGI(TAG, "UAC SET_CUR received: %" PRIu32 " %%", volume);
  atomic_store(&s_pending_volume, (int32_t)volume);
}

static void apply_host_controls(void) {
  // Handle a pending USB disconnect before processing host controls.
  if (atomic_exchange(&s_usb_disconnected, false)) {
    s_usb_volume_guard = true;
    s_usb_trim_enabled = false;
    s_usb_volume_relative = false;
    s_usb_volume_offset_db = 0.0f;
    s_last_host_volume = -1;

    // Limit the retained USB level after disconnect.
    // Do not touch the DAC: another source may own it.
    if (s_last_effective_usb_db > -25.0f) {
      s_last_effective_usb_db = -25.0f;
    }
    s_unmuted_db = s_last_effective_usb_db;

    ESP_LOGI(TAG, "USB volume reconnect guard armed");
  }

  int32_t mute = atomic_exchange(&s_pending_mute, -1);
  int32_t volume = atomic_exchange(&s_pending_volume, -1);
  int32_t steps = atomic_exchange(&s_local_volume_steps, 0);

  if (volume >= 0 || steps != 0) {
    ESP_LOGI(TAG,
             "USB volume diagnostic: host=%ld previous=%ld "
             "local_steps=%ld guard=%d relative=%d",
             (long)volume,
             (long)s_last_host_volume,
             (long)steps,
             (int)s_usb_volume_guard,
             (int)s_usb_volume_relative);
  }

  if (mute >= 0) {
    s_muted = (mute == 1);
    ESP_LOGI(TAG, "Host %s", s_muted ? "muted" : "unmuted");
  }

  // Local EC11/IR input takes precedence over a simultaneous
  // UAC response. The response updates the host reference only.
  if (steps != 0) {
    s_unmuted_db = usb_safe_volume_db(
        s_last_effective_usb_db + (float)steps * 1.0f);
    s_last_effective_usb_db = s_unmuted_db;

    if (s_unmuted_db >= -20.0f) {
      s_usb_trim_enabled = true;
    }

    if (volume >= 0) {
      s_last_host_volume = volume;
    }

    if (s_last_host_volume >= 0) {
      s_usb_volume_guard = false;
      s_usb_volume_relative = true;
    }

    ESP_LOGI(TAG, "Local USB volume: %.1f dB", s_unmuted_db);

  } else if (volume >= 0) {
    float requested_db = usb_safe_volume_db(
        VOLUME_MIN_DB +
        ((float)volume / 100.0f) *
        (VOLUME_MAX_DB - VOLUME_MIN_DB));

    if (s_usb_volume_guard) {
      // Automatic host commands may reduce, but never increase,
      // the protected output level.
      if (requested_db < s_last_effective_usb_db) {
        s_last_effective_usb_db = requested_db;
      }

      s_unmuted_db = s_last_effective_usb_db;

    } else if (s_usb_volume_relative) {
      // Diagnostic phase: do not apply host updates twice.
      // Local EC11/IR steps remain authoritative.
      s_unmuted_db = s_last_effective_usb_db;
    }

    s_last_host_volume = volume;
  }

  // Only USB playback may change the shared DAC here.
  // Radio and AirPlay retain exclusive control while USB is idle.
  if (s_streaming && (mute >= 0 || volume >= 0 || steps != 0)) {
    dac_set_volume(
        s_muted ? VOLUME_MIN_DB : usb_output_volume_db());
  }

  if (volume >= 0 || steps != 0) {
    display_show_volume(usb_safe_volume_db(s_unmuted_db));
  }
}

// ============================================================================
// Writer task
// ============================================================================

static size_t ringbuf_filled(void) {
  return RINGBUF_SIZE - xRingbufferGetCurFreeSize(s_ringbuf);
}

static void ringbuf_drain(void) {
  size_t item_size = 0;
  void *data;
  while ((data = xRingbufferReceiveUpTo(s_ringbuf, &item_size, 0,
                                        RINGBUF_SIZE)) != NULL) {
    vRingbufferReturnItem(s_ringbuf, data);
  }
}

// The longest callback gap since the last call, folded into the stream's.
static uint32_t take_gap_max_us(void) {
  uint32_t gap_us = atomic_exchange(&s_gap_max_us, 0);
  if (gap_us > s_stream_gap_max_us) {
    s_stream_gap_max_us = gap_us;
  }
  return gap_us;
}

// Diagnostic: the host's delivered frame rate should equal the I2S rate.  A
// standing difference means the two clocks are not locked and the ring buffer
// is absorbing the error until it runs dry or overflows.
static void log_stats(void) {
  int64_t now = esp_timer_get_time();
  int64_t elapsed = now - s_stats_us;
  if (elapsed < STATS_INTERVAL_US) {
    return;
  }
  s_stats_us = now;
  uint32_t bytes = atomic_exchange(&s_rx_bytes, 0);
  uint32_t fps = (uint32_t)((uint64_t)(bytes / BYTES_PER_FRAME) * 1000000U /
                            (uint64_t)elapsed);
  ESP_LOGD(TAG,
           "host %" PRIu32 " fps (i2s %d) ring %u/%u dropped %" PRIu32
           " underruns %" PRIu32,
           fps, CONFIG_OUTPUT_SAMPLE_RATE_HZ, (unsigned)ringbuf_filled(),
           (unsigned)RINGBUF_SIZE, s_dropped, s_underruns);

  uint32_t gap_us = take_gap_max_us();
  uint32_t missing_ms = atomic_load(&s_missing_ms);
  if (missing_ms != s_missing_logged) {
    ESP_LOGW(TAG,
             "%" PRIu32 " ms of USB audio missing (longest gap %" PRIu32
             ".%" PRIu32 " ms)",
             missing_ms - s_missing_logged, gap_us / 1000, gap_us % 1000 / 100);
    s_missing_logged = missing_ms;
  }
}

static void usb_sink_task(void *arg) {
  (void)arg;
  static const int16_t silence[256] = {0};

  for (;;) {
    // Process USB reconnects even before the host starts streaming.
    // DAC writes remain disabled while USB does not own the output.
    apply_host_controls();

    // Release USB ownership before radio starts using I2S.
    if (atomic_load(&s_release_requested)) {
      ringbuf_drain();

      if (s_streaming) {
        audio_output_flush();
        playback_events_emit(PLAYBACK_SOURCE_USB,
                             PLAYBACK_EVENT_DISCONNECTED, NULL);
        s_streaming = false;
      }

      // Release playback ownership without restarting AirPlay.
      // The mode controller will select RADIO after this acknowledgement.
      if (s_state_cb != NULL) {
        s_state_cb(false);
      }

      // Keep takeover blocked until the mode controller allows it.
      atomic_store(&s_release_requested, false);
      atomic_store(&s_release_acknowledged, true);
      continue;
    }

    if (!s_streaming) {
      // Idle: wait for the host to start pushing frames, then let a small
      // prefetch accumulate before taking the output over from AirPlay.
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
      if (ringbuf_filled() < RINGBUF_PREFETCH) {
        continue;
      }
      // A mode switch may have reserved I2S for Internet radio.
      // Discard host audio instead of taking ownership.
      if (!atomic_load(&s_takeover_allowed)) {
        ringbuf_drain();
        continue;
      }

      ESP_LOGI(TAG, "Host stream started");

      // Obtain exclusive I2S ownership before changing its clock.
      if (s_state_cb != NULL && !s_state_cb(true)) {
        ESP_LOGE(TAG, "USB stream rejected: audio output not released");
        ringbuf_drain();
        continue;
      }

      // Recheck after the ownership callback: a radio mode switch
      // may have requested release while AirPlay was stopping.
      if (!atomic_load(&s_takeover_allowed) ||
          atomic_load(&s_release_requested)) {
        ESP_LOGW(TAG, "USB takeover cancelled by mode switch");
        ringbuf_drain();
        continue;
      }

      // Apply any pending host volume/mute updates before USB playback.
      apply_host_controls();

      // Restore the current USB volume before enabling audio playback.
      // Otherwise the DAC may still be at the previous source's level.
      dac_set_volume(
        s_muted ? VOLUME_MIN_DB : usb_output_volume_db());

      s_streaming = true;

      // An AirPlay session may have left the clock at its own rate, and this
      // path writes to I2S directly without resampling.
      audio_output_set_sample_rate(CONFIG_UAC_SAMPLE_RATE);
      // The board keeps the amplifier in DAC_POWER_OFF until it sees these,
      // so without them the USB stream is written to a sleeping DAC.
      // CLIENT_CONNECTED also clears whatever the display was last showing.
      playback_events_emit(PLAYBACK_SOURCE_USB, PLAYBACK_EVENT_CONNECTED, NULL);
      playback_events_emit(PLAYBACK_SOURCE_USB, PLAYBACK_EVENT_PLAYING, NULL);
      // UAC carries no track info, so label the source instead.
      playback_event_data_t meta = {0};
      snprintf(meta.metadata.title, METADATA_STRING_MAX, "USB Audio");
      playback_events_emit(PLAYBACK_SOURCE_USB, PLAYBACK_EVENT_METADATA, &meta);
      s_dropped = 0;
      s_underruns = 0;
      atomic_store(&s_rx_bytes, 0);
      atomic_store(&s_missing_ms, 0);
      atomic_store(&s_gap_max_us, 0);
      s_missing_logged = 0;
      s_stream_gap_max_us = 0;
      s_stats_us = esp_timer_get_time();
      continue;
    }

    apply_host_controls();
    log_stats();

    size_t item_size = 0;
    void *data =
        xRingbufferReceiveUpTo(s_ringbuf, &item_size, pdMS_TO_TICKS(20), 512);
    if (data != NULL) {
      if (s_muted) {
        // USB mute must silence the PCM stream, not merely attenuate
        // the TAS5825M to -30 dB.
        static const uint8_t muted_pcm[512] = {0};
        size_t remaining = item_size;

        while (remaining > 0) {
          size_t chunk = remaining < sizeof(muted_pcm)
                             ? remaining : sizeof(muted_pcm);
          audio_output_write(muted_pcm, chunk, pdMS_TO_TICKS(20));
          remaining -= chunk;
        }
      } else {
        audio_output_write(data, item_size, pdMS_TO_TICKS(20));
      }

      vRingbufferReturnItem(s_ringbuf, data);
      continue;
    }

    if (esp_timer_get_time() - s_last_rx_us > IDLE_TIMEOUT_US) {
      (void)take_gap_max_us();
      ESP_LOGI(TAG,
               "Host stream idle, releasing output (%" PRIu32
               " dropped, %" PRIu32 " ms missing, longest gap %" PRIu32
               ".%" PRIu32 " ms)",
               s_dropped, atomic_load(&s_missing_ms),
               s_stream_gap_max_us / 1000, s_stream_gap_max_us % 1000 / 100);
      s_streaming = false;
      ringbuf_drain();
      audio_output_flush();
      playback_events_emit(PLAYBACK_SOURCE_USB, PLAYBACK_EVENT_DISCONNECTED,
                           NULL);
      if (s_state_cb != NULL) {
        s_state_cb(false);
      }
      continue;
    }

    // Short gap between USB packets — keep I2S fed rather than underrun.
    s_underruns++;
    audio_output_write(silence, sizeof(silence), pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t usb_audio_sink_init(usb_audio_sink_state_cb_t state_cb) {
  if (s_task != NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // Start USB audio at a conservative level, independently
  // of the previously selected radio/AirPlay volume.
  (void)settings_get_volume(&s_unmuted_db);
  s_unmuted_db = usb_safe_volume_db(s_unmuted_db);

  if (s_unmuted_db > -25.0f) {
    s_unmuted_db = -25.0f;
  }

  s_last_effective_usb_db = s_unmuted_db;
  s_usb_volume_guard = true;
  s_usb_trim_enabled = false;
  s_usb_volume_relative = false;
  s_last_host_volume = -1;

  s_state_cb = state_cb;
  s_last_rx_us = esp_timer_get_time();

  s_ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
  if (s_ringbuf == NULL) {
    ESP_LOGE(TAG, "Failed to allocate %u byte ring buffer",
             (unsigned)RINGBUF_SIZE);
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreatePinnedToCore(usb_sink_task, "usb_sink", SINK_TASK_STACK, NULL,
                              SINK_TASK_PRIO, &s_task,
                              SINK_TASK_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create writer task");
    vRingbufferDelete(s_ringbuf);
    s_ringbuf = NULL;
    return ESP_ERR_NO_MEM;
  }

  uac_device_config_t cfg = {
      .skip_tinyusb_init = false,
      .output_cb = uac_output_cb,
      .input_cb = NULL,
      .set_mute_cb = uac_set_mute_cb,
      .set_volume_cb = uac_set_volume_cb,
      .cb_ctx = NULL,
#if CONFIG_USB_DEVICE_UAC_AS_PART
      // We own the descriptors, so the driver cannot infer this itself.
      .spk_itf_num = ITF_NUM_AUDIO_STREAMING_SPK,
      .mic_itf_num = 0,
#endif
  };

  esp_err_t err = uac_device_init(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uac_device_init failed: %s", esp_err_to_name(err));
    vTaskDelete(s_task);
    s_task = NULL;
    vRingbufferDelete(s_ringbuf);
    s_ringbuf = NULL;
    return err;
  }

  ESP_LOGI(TAG, "USB audio sink ready (%d Hz stereo, %u ms buffer)",
           CONFIG_UAC_SAMPLE_RATE,
           (unsigned)(RINGBUF_SIZE / BYTES_PER_FRAME * 1000 /
                      CONFIG_OUTPUT_SAMPLE_RATE_HZ));
  return ESP_OK;
}

bool usb_audio_sink_is_streaming(void) {
  return s_streaming;
}

bool usb_audio_sink_release_and_wait(void) {
  // Prevent any new USB takeover before requesting release.
  atomic_store(&s_takeover_allowed, false);
  atomic_store(&s_release_acknowledged, false);
  atomic_store(&s_release_requested, true);

  if (s_task == NULL) {
    atomic_store(&s_release_requested, false);
    atomic_store(&s_release_acknowledged, true);
    return true;
  }

  // Wake the sink even if it is waiting for USB packets.
  xTaskNotifyGive(s_task);

  const TickType_t timeout = pdMS_TO_TICKS(3000);
  const TickType_t start = xTaskGetTickCount();

  while (!atomic_load(&s_release_acknowledged)) {
    if ((xTaskGetTickCount() - start) >= timeout) {
      ESP_LOGE(TAG, "USB audio release timed out");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  ESP_LOGI(TAG, "USB audio released I2S ownership");
  return true;
}

bool usb_audio_sink_is_release_blocked(void) {
  return !atomic_load(&s_takeover_allowed);
}

void usb_audio_sink_allow(void) {
  atomic_store(&s_release_requested, false);
  atomic_store(&s_takeover_allowed, true);
  if (s_task != NULL) {
    xTaskNotifyGive(s_task);
  }
}
