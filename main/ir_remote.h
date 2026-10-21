// SPDX-FileCopyrightText: 2026 Streaming-Box contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"

// Initialize the IR receiver on GPIO 7.
// Diagnostic mode: log received signals without triggering actions.
esp_err_t ir_remote_init(void);
