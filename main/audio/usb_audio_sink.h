// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"

// Called when the USB host disconnects.
void usb_audio_sink_notify_disconnected(void);

#include <stdbool.h>

/**
 * USB Audio Class sink — the board enumerates as a USB speaker and plays
 * the host's audio through the same I2S DAC path AirPlay uses.
 */

/**
 * Called when the host's audio stream starts and stops.
 *
 * AirPlay and the USB sink cannot both drive I2S, so the owner of the
 * callback is expected to stop the AirPlay services on `true` and restart
 * them on `false`.  Invoked from the sink's writer task, so it may block.
 */
// Return false to deny USB takeover when streaming becomes active.
// The return value is ignored when streaming becomes inactive.
typedef bool (*usb_audio_sink_state_cb_t)(bool streaming);

/**
 * Start the UAC device and the writer task that drains it into the DAC.
 */
esp_err_t usb_audio_sink_init(usb_audio_sink_state_cb_t state_cb);

/**
 * True while the host is actively sending audio.
 */
bool usb_audio_sink_is_streaming(void);

/**
 * Request that USB audio relinquish I2S ownership.
 *
 * Blocks new USB takeover attempts until USB audio is explicitly
 * allowed again. Waits for the sink task to finish writing to I2S.
 *
 * @return true if USB audio has released I2S, false on timeout.
 */
bool usb_audio_sink_release_and_wait(void);

/**
 * Allow USB audio to take over I2S again when the host streams.
 */
void usb_audio_sink_allow(void);

/**
 * True while USB takeover is blocked for a mode change.
 */
bool usb_audio_sink_is_release_blocked(void);

/**
 * Notify the USB sink about an explicit local volume action
 * (rotary encoder or IR remote).
 *
 * direction: +1 = louder, -1 = quieter.
 */
void usb_audio_sink_notify_local_volume(int direction);
