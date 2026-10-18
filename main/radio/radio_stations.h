#pragma once

#include <stddef.h>
#include "esp_err.h"

#define RADIO_MAX_FAVORITES 20
#define RADIO_STATION_NAME_MAX 96
#define RADIO_STATION_URL_MAX 512

// Temporary test stream until favorites are connected to the player.
#define RADIO_TEST_STREAM_URL "http://stream.srg-ssr.ch/m/rsj/mp3_128"

typedef struct {
  char name[RADIO_STATION_NAME_MAX];
  char url[RADIO_STATION_URL_MAX];
} radio_station_t;

// Initialize station storage and load saved favorites.
esp_err_t radio_stations_init(void);

// Retrieve the current ordered list of favorites.
size_t radio_stations_count(void);
esp_err_t radio_stations_get(size_t index, radio_station_t *station);

// Manage favorites. Changes must be persisted in NVS.
esp_err_t radio_stations_add(const radio_station_t *station);
esp_err_t radio_stations_remove(size_t index);
esp_err_t radio_stations_move(size_t from, size_t to);

// Remember the selected station across restarts.
esp_err_t radio_stations_select(size_t index);
esp_err_t radio_stations_get_selected(size_t *index);
