#pragma once

#include <stdbool.h>

#include "esp_err.h"

// Start playback of the selected internet radio station.
esp_err_t radio_player_start(void);

// Stop playback and release the audio output before switching to AirPlay.
bool radio_player_stop_and_wait(void);

// Return whether the radio player is currently active.
bool radio_player_is_running(void);
