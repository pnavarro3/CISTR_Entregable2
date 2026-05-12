/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/ledc.h"

#define TWAI_LISTENER_TX_GPIO   CONFIG_EXAMPLE_TWAI_TX_GPIO  // Listen only node doesn't need TX pin
#define TWAI_LISTENER_RX_GPIO   CONFIG_EXAMPLE_TWAI_RX_GPIO
#define TWAI_BITRATE            1000000

// Message IDs (must match sender)
#define TWAI_DATA_ID            0x100

// ---------- LEDs ----------
#define LED1_GPIO   18
#define LED2_GPIO   19
#define LED3_GPIO   21
//#define TEST_CONSIGNA  50
#define PWM_MAX_DUTY   4095

// Buffer for burst data handling
#define POLL_DEPTH              200

static const char *TAG = "twai_listen";
static uint8_t s_leds_on = 0;
static QueueHandle_t s_led_queue;

#define CONSIGNA_MAX 4095

// Inicializa los tres canales PWM 
static void leds_init(void)
{
    // Configurar el timer PWM
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,   // 0-4095
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,                
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    // Configurar un canal por cada LED
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };

    ch_cfg.channel  = LEDC_CHANNEL_0;  ch_cfg.gpio_num = LED1_GPIO;  ledc_channel_config(&ch_cfg);
    ch_cfg.channel  = LEDC_CHANNEL_1;  ch_cfg.gpio_num = LED2_GPIO;  ledc_channel_config(&ch_cfg);
    ch_cfg.channel  = LEDC_CHANNEL_2;  ch_cfg.gpio_num = LED3_GPIO;  ledc_channel_config(&ch_cfg);
}
static void set_leds(uint16_t consigna)
{
    uint32_t duty1 = 0, duty2 = 0, duty3 = 0;
    const uint16_t first_stage_max = CONSIGNA_MAX / 3;
    const uint16_t second_stage_max = (CONSIGNA_MAX * 2) / 3;

    if (consigna <= first_stage_max) {
        duty1 = ((uint32_t)consigna * PWM_MAX_DUTY) / first_stage_max;
    } else {
        duty1 = PWM_MAX_DUTY;
    }

    if (consigna > first_stage_max && consigna <= second_stage_max) {
        duty2 = ((uint32_t)(consigna - first_stage_max - 1) * PWM_MAX_DUTY) / (second_stage_max - first_stage_max);
    } else if (consigna > second_stage_max) {
        duty2 = PWM_MAX_DUTY;
    }

    if (consigna > second_stage_max) {
        duty3 = ((uint32_t)(consigna - second_stage_max - 1) * PWM_MAX_DUTY) / (CONSIGNA_MAX - second_stage_max);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty3);  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    s_leds_on = (duty1 > 0 ? 1 : 0) + (duty2 > 0 ? 1 : 0) + (duty3 > 0 ? 1 : 0);

    ESP_LOGI(TAG, "Consigna=%d -> LED1 duty=%lu, LED2 duty=%lu, LED3 duty=%lu", consigna, duty1, duty2, duty3);
}

static void leds_task(void *pvParameters)
{
    uint16_t consigna = 0;

    leds_init();
    set_leds(0);

    while (1) {
        if (xQueueReceive(s_led_queue, &consigna, portMAX_DELAY) == pdTRUE) {
            set_leds(consigna);
        }
    }
}

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_listener_data_t;

typedef struct {
    twai_node_handle_t node_hdl;
    twai_listener_data_t *rx_pool;
    SemaphoreHandle_t free_pool_semaphore;
    SemaphoreHandle_t rx_result_semaphore;
    int write_idx;
    int read_idx;
} twai_listener_ctx_t;

// TWAI receive callback - store data and signal
static bool IRAM_ATTR twai_listener_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken;
    twai_listener_ctx_t *ctx = (twai_listener_ctx_t *)user_ctx;

    if (xSemaphoreTakeFromISR(ctx->free_pool_semaphore, &woken) != pdTRUE) {
        ESP_EARLY_LOGI(TAG, "Pool full, dropping frame");
        return (woken == pdTRUE);
    }
    if (twai_node_receive_from_isr(handle, &ctx->rx_pool[ctx->write_idx].frame) == ESP_OK) {
        ctx->write_idx = (ctx->write_idx + 1) % POLL_DEPTH;
        xSemaphoreGiveFromISR(ctx->rx_result_semaphore, &woken);
    }
    return (woken == pdTRUE);
}

void app_main(void)
{
    printf("===================TWAI Listen Only Example Starting...===================\n");
    s_led_queue = xQueueCreate(1, sizeof(uint16_t));
    assert(s_led_queue != NULL);
    xTaskCreate(leds_task, "leds_task", 2048, NULL, 5, NULL);

    // Create semaphore for receive notification
    twai_listener_ctx_t twai_listener_ctx = {0};
    twai_listener_ctx.free_pool_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, POLL_DEPTH);
    twai_listener_ctx.rx_result_semaphore = xSemaphoreCreateCounting(POLL_DEPTH, 0);
    assert(twai_listener_ctx.free_pool_semaphore != NULL);
    assert(twai_listener_ctx.rx_result_semaphore != NULL);

    twai_listener_ctx.rx_pool = calloc(POLL_DEPTH, sizeof(twai_listener_data_t));
    assert(twai_listener_ctx.rx_pool != NULL);
    for (int i = 0; i < POLL_DEPTH; i++) {
        twai_listener_ctx.rx_pool[i].frame.buffer = twai_listener_ctx.rx_pool[i].data;
        twai_listener_ctx.rx_pool[i].frame.buffer_len = sizeof(twai_listener_ctx.rx_pool[i].data);
    }
    ESP_LOGI(TAG, "Buffer initialized: %d slots for burst data", POLL_DEPTH);

    // Configure TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_LISTENER_TX_GPIO,
            .rx = TWAI_LISTENER_RX_GPIO,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing.bitrate = TWAI_BITRATE,
        .flags.enable_listen_only = false,
        .tx_queue_depth = 5,
    };

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &twai_listener_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI node created");

    // Configure acceptance filter
    twai_mask_filter_config_t data_filter = {
        .id = TWAI_DATA_ID,
        .mask = 0x7F0,      // Match high 7 bits of the ID, ignore low 4 bits
        .is_ext = false,    // Receive only standard ID
    };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(twai_listener_ctx.node_hdl, 0, &data_filter));

    // Register callbacks
    twai_event_callbacks_t callbacks = {
        .on_rx_done = twai_listener_rx_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twai_listener_ctx.node_hdl, &callbacks, &twai_listener_ctx));

    // Enable TWAI node
    ESP_ERROR_CHECK(twai_node_enable(twai_listener_ctx.node_hdl));
    ESP_LOGI(TAG, "TWAI start listening...");

    // Main loop - process all buffered data when signaled
    while (1) {
        if (xSemaphoreTake(twai_listener_ctx.rx_result_semaphore, portMAX_DELAY) == pdTRUE) {
            twai_frame_t *frame = &twai_listener_ctx.rx_pool[twai_listener_ctx.read_idx].frame;
            uint16_t consigna = 0;
            ESP_LOGI(TAG, "RX: %x [%d] %x %x %x %x %x %x %x %x", \
                     frame->header.id, frame->header.dlc, frame->buffer[0], frame->buffer[1], frame->buffer[2], frame->buffer[3], frame->buffer[4], frame->buffer[5], frame->buffer[6], frame->buffer[7]);

            // Caso 1: consulta '?' -> responder cantidad de LEDs encendidos (0..3)
            if (frame->header.dlc == 1 && frame->buffer[0] == '?') {
                uint8_t led_count = s_leds_on;
                twai_frame_t tx_frame = {0};

                tx_frame.header.id = frame->header.id;
                tx_frame.header.dlc = 1;
                tx_frame.buffer = &led_count;
                tx_frame.buffer_len = 1;

                esp_err_t err = twai_node_transmit(twai_listener_ctx.node_hdl, &tx_frame, 10);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Consulta recibida, respuesta enviada: %d LED(s)", s_leds_on);
                } else {
                    ESP_LOGW(TAG, "No se pudo responder consulta, error=%s", esp_err_to_name(err));
                }
            } else if (frame->header.dlc >= 2) {
                // Caso 2: consigna en 2 bytes 
                consigna = (uint16_t)frame->buffer[0] | ((uint16_t)frame->buffer[1] << 8);
                if (consigna <= CONSIGNA_MAX) {
                    if (xQueueOverwrite(s_led_queue, &consigna) != pdPASS) {
                        ESP_LOGW(TAG, "No se pudo enviar la consigna a la tarea de LEDs");
                    }
                } else {
                    ESP_LOGW(TAG, "Consigna fuera de rango: %u", consigna);
                }
            } else {
                ESP_LOGW(TAG, "Mensaje invalido");
            }

            twai_listener_ctx.read_idx = (twai_listener_ctx.read_idx + 1) % POLL_DEPTH;
            xSemaphoreGive(twai_listener_ctx.free_pool_semaphore);
        }
    }

    // Cleanup
    vSemaphoreDelete(twai_listener_ctx.rx_result_semaphore);
    vSemaphoreDelete(twai_listener_ctx.free_pool_semaphore);
    free(twai_listener_ctx.rx_pool);
    ESP_ERROR_CHECK(twai_node_disable(twai_listener_ctx.node_hdl));
    ESP_ERROR_CHECK(twai_node_delete(twai_listener_ctx.node_hdl));
    vQueueDelete(s_led_queue);

}
