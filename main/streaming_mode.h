// SPDX-FileCopyrightText: 2026 Streaming-Box contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdbool.h>

typedef enum {
    STREAMING_MODE_RADIO = 0,
    STREAMING_MODE_AIRPLAY
} streaming_mode_t;

// Request a mode change from another FreeRTOS task.
bool streaming_mode_request_toggle(void);

// Request a radio stream restart after changing the selected station.
bool streaming_mode_request_radio_restart(void);

// Return the currently selected operating mode.
streaming_mode_t streaming_mode_get(void);
