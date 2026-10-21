// SPDX-FileCopyrightText: 2026 Streaming-Box contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ir_remote.h"
#include "buttons.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define IR_RX_GPIO          7
#define IR_RESOLUTION_HZ    1000000
#define IR_SYMBOL_COUNT     128
#define IR_QUEUE_LENGTH     4
#define IR_TASK_STACK       4096

static const char *TAG = "ir_remote";

typedef struct {
    size_t num_symbols;
} ir_rx_event_t;

static QueueHandle_t s_rx_queue;
static rmt_channel_handle_t s_rx_channel;
static rmt_symbol_word_t s_symbols[IR_SYMBOL_COUNT];

static bool IRAM_ATTR ir_rx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *event,
    void *user_data)
{
    (void)channel;

    QueueHandle_t queue = (QueueHandle_t)user_data;
    ir_rx_event_t message = {
        .num_symbols = event->num_symbols,
    };

    BaseType_t higher_priority_woken = pdFALSE;

    xQueueSendFromISR(queue, &message, &higher_priority_woken);

    return higher_priority_woken == pdTRUE;
}


static const char *ir_button_name(uint8_t command)
{
    switch (command) {
        case 0x9C: return "POWER";
        case 0xCA: return "UP";
        case 0xD2: return "DOWN";
        case 0x99: return "LEFT";
        case 0xC1: return "RIGHT";
        case 0xCE: return "OK";
        case 0xCB: return "HOME";
        case 0x08: return "BACK";
        case 0x9D: return "MENU";
        case 0x80: return "VOL+";
        case 0x81: return "VOL-";
        default:   return "UNKNOWN";
    }
}

static void ir_decode_nec(size_t count)
{
    if (count < 33) {
        ESP_LOGI(TAG, "NEC: incomplete frame (%u symbols)",
                 (unsigned)count);
        return;
    }

    uint8_t bytes[4] = {0};

    for (size_t bit = 0; bit < 32; bit++) {
        const rmt_symbol_word_t *symbol = &s_symbols[bit + 1];

        if (symbol->duration1 >= 1100 &&
            symbol->duration1 <= 2200) {
            bytes[bit / 8] |= (uint8_t)(1U << (bit % 8));
        } else if (symbol->duration1 < 300 ||
                   symbol->duration1 > 900) {
            ESP_LOGW(TAG, "NEC: invalid bit %u", (unsigned)bit);
            return;
        }
    }

    if ((uint8_t)(bytes[0] ^ bytes[1]) != 0xFF ||
        (uint8_t)(bytes[2] ^ bytes[3]) != 0xFF) {
        ESP_LOGW(TAG, "NEC: invalid complement");
        return;
    }

    ESP_LOGI(TAG, "NEC: address=0x%02X command=0x%02X button=%s",
             bytes[0], bytes[2],
             bytes[0] == 0x00 ? ir_button_name(bytes[2]) : "UNKNOWN");

    if (bytes[0] == 0x00) {
        buttons_post_ir_command(bytes[2]);
    }
}

static void ir_rx_task(void *arg)
{
    (void)arg;

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 15000000,
    };

    while (true) {
        esp_err_t err = rmt_receive(
            s_rx_channel,
            s_symbols,
            sizeof(s_symbols),
            &receive_config
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RMT receive failed: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ir_rx_event_t event;

        if (xQueueReceive(
                s_rx_queue,
                &event,
                portMAX_DELAY) == pdTRUE) {

            ir_decode_nec(event.num_symbols);

            ESP_LOGI(TAG, "IR signal received: %u symbols",
                     (unsigned)event.num_symbols);

            size_t count = event.num_symbols;

            if (count > IR_SYMBOL_COUNT) {
                count = IR_SYMBOL_COUNT;
            }

            if (count > 34) {
                count = 34;
            }

            for (size_t i = 0; i < count; i++) {
                ESP_LOGI(
                    TAG,
                    "IR[%02u]: %u/%u us, levels %u/%u",
                    (unsigned)i,
                    (unsigned)s_symbols[i].duration0,
                    (unsigned)s_symbols[i].duration1,
                    (unsigned)s_symbols[i].level0,
                    (unsigned)s_symbols[i].level1
                );
            }
        }
    }
}

esp_err_t ir_remote_init(void)
{
    s_rx_queue = xQueueCreate(
        IR_QUEUE_LENGTH,
        sizeof(ir_rx_event_t)
    );

    if (s_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    rmt_rx_channel_config_t channel_config = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = IR_SYMBOL_COUNT,
        .flags.with_dma = false,
    };

    esp_err_t err = rmt_new_rx_channel(
        &channel_config,
        &s_rx_channel
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT channel creation failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = ir_rx_done_callback,
    };

    err = rmt_rx_register_event_callbacks(
        s_rx_channel,
        &callbacks,
        s_rx_queue
    );

    if (err != ESP_OK) {
        return err;
    }

    err = rmt_enable(s_rx_channel);

    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreate(
            ir_rx_task,
            "ir_rx",
            IR_TASK_STACK,
            NULL,
            3,
            NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "IR receiver initialized on GPIO %d",
             IR_RX_GPIO);

    return ESP_OK;
}
