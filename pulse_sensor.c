/*
 * MIT License
 *
 * Copyright (c) 2023 Aleksandar Gargenta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file pulse_sensor.c
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "pulse_sensor.h"

#define SET_DEFAULT(v, d) \
    if (!v)               \
    {                     \
        v = d;            \
    }

#define VALIDATE_PULSE_SENSOR(pulse_sensor, op_name, return_value)                \
    if (!pulse_sensor)                                                            \
    {                                                                             \
        ESP_LOGW(TAG, "ignoring attempt to call pulse_sensor_%s(NULL)", op_name); \
        return return_value;                                                      \
    }

struct pulse_sensor_s
{
    pulse_sensor_config_t config;               /// the config used to open this device
    QueueHandle_t queue;                        /// internal queue for managing pulses and ticks
    TaskHandle_t task;                          /// internal task for processing pulses and ticks
    esp_timer_handle_t timer;                   /// internal timer for generating ticks
    int64_t latest_pulse_timestamp;             /// the time (usec since boot) of the latest (most recent) pulse (includes jitter)
    int64_t latest_significant_pulse_timestamp; /// the time (usec since boot) of the latest (most recent) significant pulse (ignores jitter)
    int64_t current_cycle_timestamp;            /// the time (usec since boot) of the first pulse in the current cycle
    uint64_t current_cycle_pulses;              /// the number of pulses in the current cycle
    uint64_t total_pulses;                      /// the total pulses across all cycles (includes jitter)
    uint64_t total_significant_pulses;          /// the total significant pulses across all cycles (ignores jitter)
    uint64_t total_duration;                    /// the total duration (usec) across all cycles
    uint32_t total_cycles;                      /// the total number of cycles
};

typedef enum
{
    PULSE_SENSOR_PULSE = 1, /// a pulse coming from the device
    PULSE_SENSOR_TICK = 2   /// a tick coming from the internal timer
} pulse_sensor_event_type_t;

typedef struct
{
    pulse_sensor_event_type_t type; /// the type of the event
    int64_t timestamp;              /// time (usec since boot) of when this event occurred
} pulse_sensor_event_t;

static const char *TAG = "pulse_sensor";

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    const pulse_sensor_h ps = (pulse_sensor_h)args;
    const pulse_sensor_event_t m = {.type = PULSE_SENSOR_PULSE, .timestamp = esp_timer_get_time()};
    xQueueSendToBackFromISR(ps->queue, (void *)&(m), NULL);
}

static void timer_handler(void *args)
{
    const pulse_sensor_h ps = (pulse_sensor_h)args;
    const pulse_sensor_event_t m = {.type = PULSE_SENSOR_TICK, .timestamp = esp_timer_get_time()};
    const BaseType_t r = xQueueSendToBack(ps->queue, (void *)&(m), pdMS_TO_TICKS(10));
    if (r != pdTRUE)
    {
        ESP_LOGW(TAG, "Timer tick event timeout on GPIO %d queue: %d", ps->config.gpio_num, r);
    }
}

static void pulse_sensor_notify(const pulse_sensor_h ps, const pulse_sensor_notification_type_t type)
{
    ESP_LOGD(TAG, "Notification: gpio=%d, type=%d, pulses=%llu, duration=%lld (usec)",
             ps->config.gpio_num,
             type,
             ps->current_cycle_pulses,
             ps->latest_pulse_timestamp - ps->current_cycle_timestamp);
    if (ps->config.notification_queue)
    {
        const pulse_sensor_notification_t n = {
            .type = type,
            .notification_arg = ps->config.notification_arg,
            .timestamp = ps->latest_pulse_timestamp,
            .pulses = ps->current_cycle_pulses,
            .duration = ps->latest_pulse_timestamp - ps->current_cycle_timestamp};
        const BaseType_t r = xQueueSendToBack(ps->config.notification_queue,
                                              (void *)&(n),
                                              ps->config.notification_timeout);
        if (r != pdTRUE)
        {
            ESP_LOGW(TAG, "Notification timeout on GPIO %d queue: %d", ps->config.gpio_num, r);
        }
    }
}

static void pulse_sensor_task(void *args)
{
    const pulse_sensor_h ps = (pulse_sensor_h)args;
    pulse_sensor_event_t m;
    ESP_LOGD(TAG, "Monitoring on GPIO %d", ps->config.gpio_num);
    while (true)
    {
        if (!xQueueReceive(ps->queue, &m, portMAX_DELAY))
        {
            ESP_LOGW(TAG, "Message timeout on GPIO %d queue. Bailing out.", ps->config.gpio_num);
            break;
        }
        switch (m.type)
        {
        case PULSE_SENSOR_PULSE:
            ESP_LOGV(TAG, "Pulse on %d", ps->config.gpio_num);
            if (ps->current_cycle_pulses == 0)
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(
                    esp_timer_start_periodic(ps->timer, ps->config.cycle_timeout));
                ESP_LOGD(TAG, "Started timer on GPIO %d", ps->config.gpio_num);
            }
            ps->current_cycle_pulses++;
            ps->total_pulses++;

            ps->latest_pulse_timestamp = m.timestamp;
            if (ps->current_cycle_timestamp == 0)
            {
                ps->current_cycle_timestamp = m.timestamp;
            }

            if (ps->current_cycle_pulses == ps->config.min_cycle_pulses)
            {
                ps->total_significant_pulses += ps->current_cycle_pulses;
                ps->total_duration += m.timestamp - ps->current_cycle_timestamp;
                ps->total_cycles++;
                ps->latest_significant_pulse_timestamp = m.timestamp;
                ESP_LOGI(TAG, "Cycle %lu started on GPIO %d", ps->total_cycles, ps->config.gpio_num);
                pulse_sensor_notify(ps, PULSE_SENSOR_CYCLE_STARTED);
            }
            else if (ps->current_cycle_pulses > ps->config.min_cycle_pulses)
            {
                ps->total_significant_pulses++;
                ps->total_duration += m.timestamp - ps->latest_significant_pulse_timestamp;
                ps->latest_significant_pulse_timestamp = m.timestamp;
            }
            break;
        case PULSE_SENSOR_TICK:
            ESP_LOGV(TAG, "Tick on %d", ps->config.gpio_num);
            if (ps->current_cycle_pulses > 0 &&
                m.timestamp - ps->last_pulse_timestamp > ps->config.cycle_timeout)
            {
                if (ps->current_cycle_pulses >= ps->config.min_cycle_pulses)
                {
                    ESP_LOGI(TAG, "Cycle %lu stopped on GPIO %d", ps->total_cycles, ps->config.gpio_num);
                    pulse_sensor_notify(ps, PULSE_SENSOR_CYCLE_STOPPED);
                }
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(ps->timer));
                ESP_LOGD(TAG, "Stopped timer on GPIO %d", ps->config.gpio_num);
                ps->current_cycle_timestamp = 0;
                ps->current_cycle_pulses = 0;
                ESP_LOGD(TAG, "Totals on GPIO %d: pulses=%llu, duration=%lld (usec), cycles=%lu",
                         ps->config.gpio_num,
                         ps->total_pulses,
                         ps->total_duration,
                         ps->total_cycles);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unexpected message type on GPIO %d queue: %d. Ignoring.",
                     ps->config.gpio_num, m.type);
        }
    }
}

esp_err_t pulse_sensor_open(pulse_sensor_config_t config, pulse_sensor_h *pulse_sensor_p)
{
    esp_err_t ret = ESP_FAIL;
    ESP_GOTO_ON_FALSE(pulse_sensor_p != NULL, ESP_ERR_INVALID_ARG, error_handler, TAG, "null");

    const pulse_sensor_h ps = calloc(1, sizeof(struct pulse_sensor_s));
    ESP_GOTO_ON_FALSE(ps != NULL, ESP_ERR_NO_MEM, error_handler, TAG, "malloc pulse sensor");

    ps->config = config;
    const gpio_num_t gpio_num = ps->config.gpio_num;
    SET_DEFAULT(ps->config.min_cycle_pulses, PULSE_SENSOR_MIN_CYCLE_PULSES)
    SET_DEFAULT(ps->config.cycle_timeout, PULSE_SENSOR_CYCLE_TIMEOUT)
    SET_DEFAULT(ps->config.queue_size, PULSE_SENSOR_QUEUE_SIZE)
    SET_DEFAULT(ps->config.notification_timeout, PULSE_SENSOR_NOTIFICATION_TIMEOUT)

    ps->queue = xQueueCreate(ps->config.queue_size, sizeof(pulse_sensor_event_t));
    ESP_GOTO_ON_FALSE(ps->queue != NULL, ESP_ERR_NO_MEM, pulse_sensor_cleanup, TAG, "create queue");

    char name[64];
    snprintf(name, sizeof(name), "pulse sensor task on GPIO %d", gpio_num);
    const BaseType_t r = xTaskCreate(pulse_sensor_task, name, 2048,
                                     (void *)ps, 1, &(ps->task));
    ESP_GOTO_ON_FALSE(r == pdPASS, ESP_ERR_NO_MEM, queue_cleanup, TAG, "create task: %d", r);

    snprintf(name, sizeof(name), "pulse sensor timer on GPIO %d", gpio_num);
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_handler,
        .arg = (void *)ps,
        .name = name};
    ESP_GOTO_ON_ERROR(esp_timer_create(&timer_args, &(ps->timer)), task_cleanup, TAG, "create timer");

    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = true,
        .intr_type = GPIO_INTR_POSEDGE};
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), timer_cleanup, TAG, "config GPIO %d", gpio_num);

    ESP_GOTO_ON_ERROR(gpio_isr_handler_add(gpio_num, gpio_interrupt_handler, (void *)ps),
                      gpio_config_cleanup, TAG, "add ISR handler on GPIO %d", gpio_num);

    *pulse_sensor_p = ps;
    ESP_LOGI(TAG,
             "Opened on GPIO %d: min cycle pulses=%lu, cycle timeout=%llu, queue size=%lu",
             ps->config.gpio_num,
             ps->config.min_cycle_pulses,
             ps->config.cycle_timeout,
             ps->config.queue_size);
    return ESP_OK;

gpio_config_cleanup:
    gpio_config_t io_conf_reset = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf_reset));
timer_cleanup:
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_delete(ps->timer));
task_cleanup:
    vTaskDelete(ps->task);
queue_cleanup:
    vQueueDelete(ps->queue);
pulse_sensor_cleanup:
    free(ps);
error_handler:
    ESP_LOGE(TAG, "Failed to open on GPIO %d: ret=0x%x", config.gpio_num, ret);
    return ret;
}

esp_err_t pulse_sensor_close(pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "close", ESP_ERR_INVALID_ARG)
    const gpio_num_t gpio_num = ps->config.gpio_num;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(gpio_num));
    gpio_config_t io_conf = {.pin_bit_mask = 1ULL << gpio_num};
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));

    if (ps->timer)
    {
        if (ps->current_cycle_pulses > 0)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(ps->timer));
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_delete(ps->timer));
    }

    if (ps->task)
    {
        vTaskDelete(ps->task);
    }

    if (ps->queue)
    {
        vQueueDelete(ps->queue);
    }
    free(ps);

    ESP_LOGI(TAG, "Closed on GPIO %d", gpio_num);
    return ESP_OK;
}

bool pulse_sensor_is_in_cycle(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "is_in_cycle", false)
    return ps->current_cycle_pulses >= ps->config.min_cycle_pulses;
}

int64_t pulse_sensor_get_latest_pulse_timestamp(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_latest_pulse_timestamp", -1)
    return ps->latest_significant_pulse_timestamp;
}

int64_t pulse_sensor_get_current_cycle_timestamp(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_current_cycle_timestamp", -1)
    return ps->current_cycle_timestamp;
}

uint64_t pulse_sensor_get_current_cycle_pulses(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_current_cycle_pulses", 0)
    return ps->current_cycle_pulses;
}

uint64_t pulse_sensor_get_current_cycle_duration(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_current_cycle_duration", 0)
    return ps->current_cycle_timestamp ? esp_timer_get_time() - ps->current_cycle_timestamp : 0;
}

uint64_t pulse_sensor_get_total_pulses(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_total_pulses", 0)
    return ps->total_pulses;
}

uint64_t pulse_sensor_get_total_duration(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_total_duration", 0)
    return ps->total_duration;
}

uint32_t pulse_sensor_get_total_cycles(const pulse_sensor_h ps)
{
    VALIDATE_PULSE_SENSOR(ps, "get_total_cycles", 0)
    return ps->total_cycles;
}
