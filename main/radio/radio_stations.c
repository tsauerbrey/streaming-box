#include "radio_stations.h"

#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RADIO_NVS_NAMESPACE "radio"
#define RADIO_NVS_KEY_FAVORITES "favorites"
#define RADIO_NVS_KEY_SELECTED "selected"
#define RADIO_STORAGE_VERSION 1

static const char *TAG = "radio_stations";

typedef struct {
  uint32_t version;
  uint32_t count;
  radio_station_t stations[RADIO_MAX_FAVORITES];
} radio_storage_t;

static radio_storage_t s_storage = {
    .version = RADIO_STORAGE_VERSION,
};
static size_t s_selected_index;
static bool s_initialized;
static SemaphoreHandle_t s_stations_mutex;

esp_err_t radio_stations_init(void) {
  if (s_stations_mutex == NULL) {
    s_stations_mutex = xSemaphoreCreateMutex();
    if (s_stations_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create radio stations mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  if (s_initialized) {
    xSemaphoreGive(s_stations_mutex);
    return ESP_OK;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(RADIO_NVS_NAMESPACE, NVS_READONLY, &nvs);

  memset(&s_storage, 0, sizeof(s_storage));
  s_storage.version = RADIO_STORAGE_VERSION;
  s_selected_index = 0;
  s_initialized = false;

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "No saved radio favorites");
    s_initialized = true;
    xSemaphoreGive(s_stations_mutex);
    return ESP_OK;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
    xSemaphoreGive(s_stations_mutex);
    return err;
  }

  radio_storage_t *loaded = calloc(1, sizeof(radio_storage_t));
  if (!loaded) {
    nvs_close(nvs);
    xSemaphoreGive(s_stations_mutex);
    return ESP_ERR_NO_MEM;
  }

  size_t length = sizeof(*loaded);
  err = nvs_get_blob(nvs, RADIO_NVS_KEY_FAVORITES, loaded, &length);

  if (err == ESP_OK) {
    size_t header_size = offsetof(radio_storage_t, stations);

    if (length < header_size) {
      ESP_LOGW(TAG, "Radio favorites header is incomplete");
      err = ESP_ERR_INVALID_SIZE;
    } else if (loaded->version != RADIO_STORAGE_VERSION ||
               loaded->count > RADIO_MAX_FAVORITES ||
               length != header_size +
                         loaded->count * sizeof(radio_station_t)) {
      ESP_LOGW(TAG, "Invalid radio favorites data");
      err = ESP_ERR_INVALID_SIZE;
    } else {
      bool valid = true;
      for (size_t i = 0; i < loaded->count; ++i) {
        if (!memchr(loaded->stations[i].name, '\0',
                    RADIO_STATION_NAME_MAX) ||
            !memchr(loaded->stations[i].url, '\0',
                    RADIO_STATION_URL_MAX)) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        err = ESP_ERR_INVALID_ARG;
      } else {
        memcpy(&s_storage, loaded, length);
      }
    }
  }

  free(loaded);

  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "Radio favorites list is empty");
    err = ESP_OK;
  }

  if (err == ESP_OK && s_storage.count > 0) {
    uint32_t selected = 0;
    esp_err_t selected_err =
        nvs_get_u32(nvs, RADIO_NVS_KEY_SELECTED, &selected);
    if (selected_err == ESP_OK && selected < s_storage.count) {
      s_selected_index = selected;
    }
  }

  nvs_close(nvs);

  if (err == ESP_OK) {
    s_initialized = true;
    ESP_LOGI(TAG, "Loaded %lu radio favorites",
             (unsigned long)s_storage.count);
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

size_t radio_stations_count(void) {
  if (s_stations_mutex == NULL) {
    return 0;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return 0;
  }

  size_t count = s_initialized ? s_storage.count : 0;

  xSemaphoreGive(s_stations_mutex);
  return count;
}

esp_err_t radio_stations_get(size_t index, radio_station_t *station) {
  if (station == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (index >= s_storage.count) {
    err = ESP_ERR_INVALID_ARG;
  } else {
    memcpy(station, &s_storage.stations[index], sizeof(*station));
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

/*
 * Persist a candidate favorites list before publishing it in RAM.
 * Caller must hold s_stations_mutex.
 */
static esp_err_t radio_stations_save_locked(
    const radio_storage_t *candidate, size_t selected_index) {
  if (candidate == NULL || candidate->count > RADIO_MAX_FAVORITES ||
      (candidate->count > 0 && selected_index >= candidate->count)) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(RADIO_NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    return err;
  }

  size_t storage_size =
      offsetof(radio_storage_t, stations) +
      candidate->count * sizeof(radio_station_t);

  err = nvs_set_blob(nvs, RADIO_NVS_KEY_FAVORITES,
                     candidate, storage_size);

  if (err == ESP_OK) {
    err = nvs_set_u32(nvs, RADIO_NVS_KEY_SELECTED,
                      (uint32_t)selected_index);
  }

  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }

  nvs_close(nvs);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save radio favorites: %s",
             esp_err_to_name(err));
  }

  return err;
}

esp_err_t radio_stations_add(const radio_station_t *station) {
  if (station == NULL ||
      station->name[0] == '\0' ||
      station->url[0] == '\0' ||
      memchr(station->name, '\0', RADIO_STATION_NAME_MAX) == NULL ||
      memchr(station->url, '\0', RADIO_STATION_URL_MAX) == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (s_storage.count >= RADIO_MAX_FAVORITES) {
    err = ESP_ERR_NO_MEM;
  } else {
    radio_storage_t *candidate = malloc(sizeof(*candidate));

    if (candidate == NULL) {
      err = ESP_ERR_NO_MEM;
    } else {
      memcpy(candidate, &s_storage, sizeof(*candidate));

      memcpy(&candidate->stations[candidate->count],
             station, sizeof(*station));
      candidate->count++;

      err = radio_stations_save_locked(candidate, s_selected_index);

      if (err == ESP_OK) {
        memcpy(&s_storage, candidate, sizeof(s_storage));
        ESP_LOGI(TAG, "Added radio favorite: %s", station->name);
      }

      free(candidate);
    }
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

esp_err_t radio_stations_get_selected(size_t *index) {
  if (index == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (s_storage.count == 0) {
    err = ESP_ERR_NOT_FOUND;
  } else {
    *index = s_selected_index;
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

esp_err_t radio_stations_select(size_t index) {
  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (index >= s_storage.count) {
    err = ESP_ERR_INVALID_ARG;
  } else if (index != s_selected_index) {
    nvs_handle_t nvs;
    err = nvs_open(RADIO_NVS_NAMESPACE, NVS_READWRITE, &nvs);

    if (err == ESP_OK) {
      err = nvs_set_u32(nvs, RADIO_NVS_KEY_SELECTED,
                        (uint32_t)index);

      if (err == ESP_OK) {
        err = nvs_commit(nvs);
      }

      nvs_close(nvs);
    }

    if (err == ESP_OK) {
      s_selected_index = index;
      ESP_LOGI(TAG, "Selected radio favorite: %lu",
               (unsigned long)index);
    } else {
      ESP_LOGE(TAG, "Failed to select radio favorite: %s",
               esp_err_to_name(err));
    }
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

esp_err_t radio_stations_remove(size_t index) {
  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (index >= s_storage.count) {
    err = ESP_ERR_INVALID_ARG;
  } else {
    radio_storage_t *candidate = malloc(sizeof(*candidate));

    if (candidate == NULL) {
      err = ESP_ERR_NO_MEM;
    } else {
      memcpy(candidate, &s_storage, sizeof(*candidate));

      size_t remaining = candidate->count - index - 1;

      if (remaining > 0) {
        memmove(&candidate->stations[index],
                &candidate->stations[index + 1],
                remaining * sizeof(radio_station_t));
      }

      candidate->count--;
      memset(&candidate->stations[candidate->count], 0,
             sizeof(radio_station_t));

      size_t selected = s_selected_index;

      if (candidate->count == 0) {
        selected = 0;
      } else if (index < selected) {
        selected--;
      } else if (index == selected &&
                 selected >= candidate->count) {
        selected = candidate->count - 1;
      }

      err = radio_stations_save_locked(candidate, selected);

      if (err == ESP_OK) {
        memcpy(&s_storage, candidate, sizeof(s_storage));
        s_selected_index = selected;
        ESP_LOGI(TAG, "Removed radio favorite: %lu",
                 (unsigned long)index);
      }

      free(candidate);
    }
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}

esp_err_t radio_stations_move(size_t from, size_t to) {
  if (s_stations_mutex == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(s_stations_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }

  esp_err_t err = ESP_OK;

  if (!s_initialized) {
    err = ESP_ERR_INVALID_STATE;
  } else if (from >= s_storage.count || to >= s_storage.count) {
    err = ESP_ERR_INVALID_ARG;
  } else if (from != to) {
    radio_storage_t *candidate = malloc(sizeof(*candidate));

    if (candidate == NULL) {
      err = ESP_ERR_NO_MEM;
    } else {
      memcpy(candidate, &s_storage, sizeof(*candidate));

      radio_station_t *moved = malloc(sizeof(*moved));

      if (moved == NULL) {
        err = ESP_ERR_NO_MEM;
      } else {
        memcpy(moved, &candidate->stations[from], sizeof(*moved));

        if (from < to) {
          memmove(&candidate->stations[from],
                  &candidate->stations[from + 1],
                  (to - from) * sizeof(radio_station_t));
        } else {
          memmove(&candidate->stations[to + 1],
                  &candidate->stations[to],
                  (from - to) * sizeof(radio_station_t));
        }

        memcpy(&candidate->stations[to], moved, sizeof(*moved));
        free(moved);

        size_t selected = s_selected_index;

        if (selected == from) {
          selected = to;
        } else if (from < to &&
                   selected > from && selected <= to) {
          selected--;
        } else if (from > to &&
                   selected >= to && selected < from) {
          selected++;
        }

        err = radio_stations_save_locked(candidate, selected);

        if (err == ESP_OK) {
          memcpy(&s_storage, candidate, sizeof(s_storage));
          s_selected_index = selected;

          ESP_LOGI(TAG, "Moved radio favorite: %lu -> %lu",
                   (unsigned long)from, (unsigned long)to);
        }
      }

      free(candidate);
    }
  }

  xSemaphoreGive(s_stations_mutex);
  return err;
}
