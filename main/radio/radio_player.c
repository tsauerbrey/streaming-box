// Streaming-Box internet radio player: HTTP MP3 -> PCM -> I2S.
// Initial version intentionally supports only 44.1 kHz, 16-bit MP3 output.

#include "radio_player.h"
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_audio_types.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "audio/audio_output.h"
#include "decoder/impl/esp_mp3_dec.h"
#include "playback_events.h"
#include "radio_stations.h"

static const char *TAG = "radio_player";
#define INPUT_CAP_INITIAL 8192U
#define INPUT_CAP_MAX 32768U
#define PCM_CAP_INITIAL 8192U
#define HTTP_TIMEOUT_MS 1000
#define RADIO_I2S_TIMEOUT_MS 1000
#define RADIO_RECONNECT_DELAY_MS 3000
#define RADIO_RECONNECT_POLL_MS 100
#define RADIO_HTTP_READ_BUFFER_SIZE 2048
#define RADIO_HTTP_MAX_REDIRECTS 5

static _Atomic(TaskHandle_t) s_radio_task = NULL;
static atomic_bool s_stop_requested = ATOMIC_VAR_INIT(false);
static atomic_bool s_audio_released = ATOMIC_VAR_INIT(true);
static SemaphoreHandle_t s_stopped = NULL;
static SemaphoreHandle_t s_lifecycle_mutex = NULL;

typedef struct {
    esp_audio_dec_handle_t decoder;
    uint8_t *input;
    size_t input_len, input_cap;
    int16_t *pcm;
    size_t pcm_bytes;
    int16_t *stereo;
    size_t stereo_samples;
    bool format_logged;
    uint32_t output_sample_rate;
	bool playback_started;
    esp_err_t fatal_error;
} radio_ctx_t;

static esp_err_t init_sync(void)
{
    if (s_stopped && s_lifecycle_mutex) return ESP_OK;
    if (s_stopped || s_lifecycle_mutex) return ESP_ERR_INVALID_STATE;
    s_stopped = xSemaphoreCreateBinary();
    if (!s_stopped) return ESP_ERR_NO_MEM;
    s_lifecycle_mutex = xSemaphoreCreateMutex();
    if (!s_lifecycle_mutex) {
        vSemaphoreDelete(s_stopped);
        s_stopped = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static bool grow_input(radio_ctx_t *c, size_t need)
{
    if (need <= c->input_cap) return true;
    if (need > INPUT_CAP_MAX) return false;
    size_t cap = c->input_cap;
    while (cap < need && cap < INPUT_CAP_MAX) cap *= 2;
    if (cap > INPUT_CAP_MAX) cap = INPUT_CAP_MAX;
    uint8_t *p = realloc(c->input, cap);
    if (!p) return false;
    c->input = p;
    c->input_cap = cap;
    return true;
}

static bool write_pcm(radio_ctx_t *c, const esp_audio_dec_info_t *info,
                      uint32_t bytes)
{
    if (!bytes || !info->sample_rate || !info->channel || !info->bits_per_sample)
        return true;
    if ((info->sample_rate != 44100 && info->sample_rate != 48000) ||
        info->bits_per_sample != 16 || (info->channel != 1 && info->channel != 2)) {
        ESP_LOGE(TAG, "Unsupported MP3 format: %lu Hz, %u-bit, %u channel(s); supported: 44100/48000 Hz, 16-bit, mono/stereo",
                 (unsigned long)info->sample_rate, info->bits_per_sample,
                 info->channel);
        c->fatal_error = ESP_ERR_NOT_SUPPORTED;
        return false;
    }
    if (bytes % (sizeof(int16_t) * info->channel)) return false;

    if (c->output_sample_rate != info->sample_rate) {
        ESP_LOGI(TAG, "Radio I2S sample rate: %lu Hz",
                 (unsigned long)info->sample_rate);
        audio_output_set_sample_rate(info->sample_rate);
        c->output_sample_rate = info->sample_rate;
    }

    if (!c->format_logged) {
        ESP_LOGI(TAG, "MP3 format: %lu Hz, %u-bit, %s",
                 (unsigned long)info->sample_rate, info->bits_per_sample,
                 info->channel == 1 ? "mono" : "stereo");
        c->format_logged = true;
    }
	if (!c->playback_started) {
        playback_events_emit(PLAYBACK_SOURCE_RADIO,
                             PLAYBACK_EVENT_PLAYING, NULL);
        c->playback_started = true;
    }
    if (info->channel == 2) {
        static unsigned pcm_diag_blocks = 0;
        static unsigned pcm_diag_nonzero_blocks = 0;
        static int32_t pcm_diag_max_peak = 0;
        if (pcm_diag_blocks < 500) {
            const int16_t *samples = (const int16_t *)c->pcm;
            size_t count = bytes / sizeof(int16_t);
            int32_t peak = 0;
            size_t nonzero = 0;

            for (size_t i = 0; i < count; ++i) {
                int32_t v = samples[i];
                if (v < 0) v = -v;
                if (v > peak) peak = v;
                if (v != 0) nonzero++;
            }

            if (nonzero) pcm_diag_nonzero_blocks++;
            if (peak > pcm_diag_max_peak) pcm_diag_max_peak = peak;
            pcm_diag_blocks++;

            if (pcm_diag_blocks % 50 == 0) {
                ESP_LOGW(TAG,
                         "PCM DIAG: blocks=%u nonzero_blocks=%u max_peak=%ld",
                         pcm_diag_blocks, pcm_diag_nonzero_blocks,
                         (long)pcm_diag_max_peak);
            }
        }

        esp_err_t err = audio_output_write_complete(
            c->pcm, bytes, pdMS_TO_TICKS(RADIO_I2S_TIMEOUT_MS));

        if (pcm_diag_blocks == 1) {
            ESP_LOGW(TAG, "I2S DIAG: write=%s", esp_err_to_name(err));
        }

        return err == ESP_OK;
    }

    size_t n = bytes / sizeof(int16_t), needed = n * 2;
    if (needed > c->stereo_samples) {
        int16_t *p = realloc(c->stereo, needed * sizeof(int16_t));
        if (!p) { c->fatal_error = ESP_ERR_NO_MEM; return false; }
        c->stereo = p;
        c->stereo_samples = needed;
    }
    for (size_t i = 0; i < n; ++i) c->stereo[2*i] = c->stereo[2*i+1] = c->pcm[i];
    return audio_output_write_complete(c->stereo, needed * sizeof(int16_t), pdMS_TO_TICKS(RADIO_I2S_TIMEOUT_MS)) == ESP_OK;
}

static esp_err_t decode_buffer(radio_ctx_t *c)
{
    size_t offset = 0;
    while (offset < c->input_len && !atomic_load(&s_stop_requested)) {
        esp_audio_dec_in_raw_t raw = {
            .buffer = c->input + offset,
            .len = (uint32_t)(c->input_len - offset),
            .consumed = 0,
            .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
        };
        esp_audio_dec_out_frame_t frame = {
            .buffer = (uint8_t *)c->pcm,
            .len = (uint32_t)c->pcm_bytes,
            .needed_size = 0,
            .decoded_size = 0,
        };
        esp_audio_dec_info_t info = {0};
        esp_audio_err_t r = esp_mp3_dec_decode(c->decoder, &raw, &frame, &info);
        if (r == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
            if (frame.needed_size <= c->pcm_bytes) return ESP_FAIL;
            int16_t *p = realloc(c->pcm, frame.needed_size);
            if (!p) return ESP_ERR_NO_MEM;
            c->pcm = p;
            c->pcm_bytes = frame.needed_size;
            continue; // Retry same input with larger output buffer.
        }
        if (raw.consumed > raw.len) return ESP_FAIL;
        if (r == ESP_AUDIO_ERR_OK) {
            if (frame.decoded_size > c->pcm_bytes) return ESP_FAIL;
            if (!write_pcm(c, &info, frame.decoded_size))
                return c->fatal_error != ESP_OK ? c->fatal_error : ESP_FAIL;
            if (!raw.consumed) break;
            offset += raw.consumed;
        } else if (r == ESP_AUDIO_ERR_DATA_LACK) {
            break; // Keep the partial frame until the next HTTP chunk.
        } else {
            // Skip damaged data one byte at a time to find the next MP3 frame.
            offset += raw.consumed ? raw.consumed : 1;
        }
    }
    if (offset) {
        c->input_len -= offset;
        if (c->input_len) memmove(c->input, c->input + offset, c->input_len);
    }
    return ESP_OK;
}

static esp_err_t radio_process_data(radio_ctx_t *c,
                                    const uint8_t *data,
                                    size_t n)
{
    if (!c || (!data && n > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (atomic_load(&s_stop_requested)) {
        return ESP_FAIL;
    }

    if (n == 0) {
        return ESP_OK;
    }

    if (c->input_len > SIZE_MAX - n ||
        !grow_input(c, c->input_len + n)) {
        c->fatal_error = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }

    memcpy(c->input + c->input_len, data, n);
    c->input_len += n;

    esp_err_t err = decode_buffer(c);
    if (err != ESP_OK) {
        c->fatal_error = err;
    }

    return err;
}

static bool radio_wait_before_reconnect(void)
{
    int remaining_ms = RADIO_RECONNECT_DELAY_MS;

    while (remaining_ms > 0) {
        if (atomic_load(&s_stop_requested)) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(RADIO_RECONNECT_POLL_MS));
        remaining_ms -= RADIO_RECONNECT_POLL_MS;
    }

    return !atomic_load(&s_stop_requested);
}

static esp_err_t radio_reset_decoder(radio_ctx_t *c)
{
    if (c->decoder) {
        (void)esp_mp3_dec_close(c->decoder);
        c->decoder = NULL;
    }

	if (c->playback_started) {
        playback_events_emit(PLAYBACK_SOURCE_RADIO,
                             PLAYBACK_EVENT_DISCONNECTED, NULL);
        c->playback_started = false;
    }
    c->input_len = 0;
    c->format_logged = false;
    c->output_sample_rate = 0;
    c->fatal_error = ESP_OK;

    if (esp_mp3_dec_open(NULL, 0, &c->decoder) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "MP3 decoder restart failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void cleanup_ctx(radio_ctx_t *c)
{
    if (c->decoder) (void)esp_mp3_dec_close(c->decoder);
    free(c->input);
    free(c->pcm);
    free(c->stereo);
}
static bool radio_is_redirect(int status)
{
    return status == 301 ||
           status == 302 ||
           status == 303 ||
           status == 307 ||
           status == 308;
}
static void radio_task(void *arg)
{
    (void)arg;
    // Prevent the task running before radio_player_start() publishes its handle.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    radio_ctx_t c = {0};
    c.fatal_error = ESP_OK;
    c.input_cap = INPUT_CAP_INITIAL;
    c.input = malloc(c.input_cap);
    c.pcm_bytes = PCM_CAP_INITIAL;
    c.pcm = malloc(c.pcm_bytes);
    esp_err_t result = ESP_OK;
	uint8_t *http_buffer = malloc(RADIO_HTTP_READ_BUFFER_SIZE);
    char *stream_url = malloc(RADIO_STATION_URL_MAX);

    if (!c.input || !c.pcm || !http_buffer || !stream_url) {
    result = ESP_ERR_NO_MEM;
    goto done;
}
	
    // Mode controller must have stopped AirPlay before starting this task.
    audio_output_set_sample_rate(CONFIG_OUTPUT_SAMPLE_RATE_HZ);

    while (!atomic_load(&s_stop_requested)) {
        // Each connection gets a fresh decoder and an empty MP3 input buffer.
        result = radio_reset_decoder(&c);
        if (result != ESP_OK) {
            break;
        }

		const char *selected_url = RADIO_TEST_STREAM_URL;
        size_t selected_index = 0;

        esp_err_t selection_err =
            radio_stations_get_selected(&selected_index);

        if (selection_err == ESP_OK) {
            radio_station_t *station = malloc(sizeof(*station));
            if (station == NULL) {
                result = ESP_ERR_NO_MEM;
                break;
            }

            result = radio_stations_get(selected_index, station);
            if (result == ESP_OK) {
                selected_url = station->url;
                memcpy(stream_url, selected_url,
                       strlen(selected_url) + 1);
            }
            free(station);

            if (result != ESP_OK) {
                break;
            }
        } else if (selection_err == ESP_ERR_NOT_FOUND) {
            memcpy(stream_url, RADIO_TEST_STREAM_URL,
                   sizeof(RADIO_TEST_STREAM_URL));
        } else {
            ESP_LOGE(TAG, "Cannot read selected radio station: %s",
                     esp_err_to_name(selection_err));
            result = selection_err;
            break;
        }

        esp_http_client_config_t config = {
		    .url = stream_url,
		    .timeout_ms = HTTP_TIMEOUT_MS,
		    .buffer_size = RADIO_HTTP_READ_BUFFER_SIZE,
		    .disable_auto_redirect = true,
		    .crt_bundle_attach = esp_crt_bundle_attach,
		};

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            result = ESP_ERR_NO_MEM;
            break;
        }
		int redirect_count = 0;
		int http_status = 0;
        esp_http_client_set_header(client, "Icy-MetaData", "0");
        ESP_LOGI(TAG, "Opening radio stream: %s", stream_url);

        while (!atomic_load(&s_stop_requested)) {
            result = esp_http_client_open(client, 0);
            if (result != ESP_OK) {
                break;
            }

            int64_t content_length = esp_http_client_fetch_headers(client);

            if (content_length == -ESP_ERR_HTTP_EAGAIN) {
                ESP_LOGW(TAG, "Timeout while receiving radio HTTP headers");
                result = ESP_ERR_TIMEOUT;
                break;
            }

            if (content_length < 0) {
                ESP_LOGE(TAG, "Failed to receive radio HTTP headers");
                result = ESP_FAIL;
                break;
            }

            http_status = esp_http_client_get_status_code(client);

            if (radio_is_redirect(http_status)) {
                if (redirect_count >= RADIO_HTTP_MAX_REDIRECTS) {
                    ESP_LOGE(TAG, "Too many radio HTTP redirects");
                    result = ESP_FAIL;
                    break;
                }

                result = esp_http_client_set_redirection(client);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Radio HTTP redirect failed: %s",
                             esp_err_to_name(result));
                    break;
                }

                redirect_count++;
                ESP_LOGI(TAG, "Following radio redirect %d/%d",
                         redirect_count, RADIO_HTTP_MAX_REDIRECTS);

                result = esp_http_client_close(client);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to close radio HTTP connection");
                    break;
                }

                result = esp_http_client_clear_response_buffer(client);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to clear radio HTTP response");
                    break;
                }

                continue;
            }

            if (http_status < 200 || http_status >= 300) {
                ESP_LOGE(TAG, "Radio HTTP status: %d", http_status);
                result = ESP_FAIL;
                break;
            }

            ESP_LOGI(TAG, "Radio HTTP connected: status %d", http_status);
            break;
        }

        if (!atomic_load(&s_stop_requested)) {
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Radio HTTP connection failed: %s",
                         esp_err_to_name(result));
            }
        }

        if (result == ESP_OK && !atomic_load(&s_stop_requested)) {
            ESP_LOGI(TAG, "Starting radio MP3 data reception");

            while (!atomic_load(&s_stop_requested)) {
                int bytes_read = esp_http_client_read(
                    client,
                    (char *)http_buffer,
                    RADIO_HTTP_READ_BUFFER_SIZE
                );

                if (bytes_read > 0) {
                    result = radio_process_data(
                        &c, http_buffer, (size_t)bytes_read
                    );

                    if (result != ESP_OK) {
                        ESP_LOGE(TAG, "Radio MP3 processing failed: %s",
                                 esp_err_to_name(result));
                        break;
                    }

                    continue;
                }

                if (bytes_read == -ESP_ERR_HTTP_EAGAIN) {
                    // Read timeout: check stop request and try again.
                    continue;
                }

                if (bytes_read == 0) {
                    ESP_LOGW(TAG, "Radio HTTP stream ended");
                    break;
                }

                ESP_LOGE(TAG, "Radio HTTP read failed: %d", bytes_read);
                result = ESP_FAIL;
                break;
            }
        }
		
		esp_http_client_cleanup(client);
		
        if (atomic_load(&s_stop_requested)) {
            result = ESP_OK;
            break;
        }

        // Stop playback before waiting for the next connection.
        if (c.playback_started) {
            playback_events_emit(PLAYBACK_SOURCE_RADIO,
                                 PLAYBACK_EVENT_DISCONNECTED, NULL);
            c.playback_started = false;
        }

        ESP_LOGW(TAG, "Radio reconnect in %d ms",
                 RADIO_RECONNECT_DELAY_MS);

        if (!radio_wait_before_reconnect()) {
            result = ESP_OK;
            break;
        }
    }

done:
    if (c.playback_started) {
        playback_events_emit(PLAYBACK_SOURCE_RADIO,
                             PLAYBACK_EVENT_DISCONNECTED, NULL);
        c.playback_started = false;
    }
    free(stream_url);
    free(http_buffer);
    cleanup_ctx(&c);
    // No I2S access after this point; the mode controller may now hand it over.
    atomic_store(&s_audio_released, true);
    atomic_store(&s_radio_task, NULL);
    if (s_stopped) xSemaphoreGive(s_stopped);
    if (result != ESP_OK && !atomic_load(&s_stop_requested))
        ESP_LOGE(TAG, "Radio player exited: %s", esp_err_to_name(result));
    else
        ESP_LOGI(TAG, "Radio player stopped");
    vTaskDelete(NULL);
}

esp_err_t radio_player_start(void)
{
    esp_err_t e = init_sync();
    if (e != ESP_OK) return e;
    if (xSemaphoreTake(s_lifecycle_mutex, pdMS_TO_TICKS(2000)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (atomic_load(&s_radio_task) != NULL || !atomic_load(&s_audio_released)) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    (void)xSemaphoreTake(s_stopped, 0);
    atomic_store(&s_stop_requested, false);
    atomic_store(&s_audio_released, false);
    TaskHandle_t task = NULL;
    BaseType_t ok = xTaskCreate(radio_task, "radio_player", 8192, NULL, 5, &task);
    if (ok != pdPASS) {
        atomic_store(&s_audio_released, true);
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }
    atomic_store(&s_radio_task, task);
    xTaskNotifyGive(task);
    xSemaphoreGive(s_lifecycle_mutex);
    ESP_LOGI(TAG, "Radio task created");
    return ESP_OK;
}

bool radio_player_is_running(void)
{
    return atomic_load(&s_radio_task) != NULL;
}

bool radio_player_stop_and_wait(void)
{
    if (!s_lifecycle_mutex) return false;
    if (xSemaphoreTake(s_lifecycle_mutex, pdMS_TO_TICKS(2000)) != pdTRUE)
        return false;
    bool stopped = false;
    if (atomic_load(&s_radio_task) == NULL || atomic_load(&s_audio_released)) {
        stopped = true;
    } else {
        atomic_store(&s_stop_requested, true);
        if (xSemaphoreTake(s_stopped, pdMS_TO_TICKS(7000)) == pdTRUE &&
            atomic_load(&s_audio_released)) {
            stopped = true;
        } else {
            ESP_LOGE(TAG, "Radio stop not confirmed; I2S handover denied");
        }
    }
    xSemaphoreGive(s_lifecycle_mutex);
    return stopped;
}
