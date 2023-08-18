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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#include "pulse_sensor.h"

#define VALIDATE_PULSE_SENSOR(pulse_sensor, op_name, return_value)                \
    if (!pulse_sensor)                                                            \
    {                                                                             \
        ESP_LOGW(TAG, "ignoring attempt to call pulse_sensor_%s(NULL)", op_name); \
        return return_value;                                                      \
    }
#define MUTEX_TIMEOUT pdMS_TO_TICKS(3) // max time to wait for a lock (3 ms)
struct pulse_sensor_s
{
    pulse_sensor_config_t config; /// the config used to open this device
    QueueHandle_t queue;          /// internal queue for managing pulses and ticks
    TaskHandle_t task;            /// internal task for processing pulses and ticks
    esp_timer_handle_t timer;     /// internal timer for generating ticks
    SemaphoreHandle_t mutex;      /// synchronization mutex
    pulse_sensor_data_t data;     /// measurement data

    int64_t tentative_pulse_timestamp; /// tentative pulse timestamp (µs since boot)
    int64_t tentative_cycle_timestamp; /// tentative cycle timestamp (µs since boot)
    uint32_t tentative_cycle_pulses;   /// tentative cycle pulses

    int64_t latest_tick_timestamp; /// the time (µs since boot) of the latest tick
    uint32_t latest_tick_pulses;   /// the number of pulses since latest_tick_timestamp
};

typedef enum
{
    PULSE_SENSOR_PULSE = 1, /// a pulse coming from the sensor
    PULSE_SENSOR_TICK = 2   /// a tick coming from the internal timer
} pulse_sensor_event_type_t;

typedef struct
{
    pulse_sensor_event_type_t type; /// the type of the event
    int64_t timestamp;              /// time (µs since boot) of when this event occurred
} pulse_sensor_event_t;

static const char *TAG = "pulse_sensor";

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    const pulse_sensor_t sensor = (pulse_sensor_t)args;
    const pulse_sensor_event_t m = {.type = PULSE_SENSOR_PULSE, .timestamp = esp_timer_get_time()};
    xQueueSendToBackFromISR(sensor->queue, (void *)&(m), NULL);
}

static void timer_handler(void *args)
{
    const pulse_sensor_t sensor = (pulse_sensor_t)args;
    const pulse_sensor_event_t m = {.type = PULSE_SENSOR_TICK, .timestamp = esp_timer_get_time()};
    const BaseType_t r = xQueueSendToBack(sensor->queue, (void *)&(m), pdMS_TO_TICKS(10));
    if (r != pdTRUE)
    {
        ESP_LOGW(TAG, "Timer tick event timeout on GPIO %d queue: %d", sensor->config.gpio_num, r);
    }
}

static esp_err_t pulse_sensor_notify(const pulse_sensor_t sensor,
                                     const pulse_sensor_notification_type_t type,
                                     int64_t timestamp)
{
    if (sensor->config.notification_queue)
    {
        ESP_LOGD(TAG, "Sending notification %d for GPIO %d", type, sensor->config.gpio_num);
        const pulse_sensor_notification_t n = {
            .sensor = sensor,
            .notification_arg = sensor->config.notification_arg,
            .type = type,
            .timestamp = timestamp};
        ESP_RETURN_ON_FALSE(
            xQueueSend(sensor->config.notification_queue, (void *)&(n), sensor->config.notification_timeout),
            ESP_ERR_TIMEOUT, TAG, "sending notification %d on GPIO %d", type, sensor->config.gpio_num);
    }
    return ESP_OK;
}

static void pulse_sensor_task(void *args)
{
    const pulse_sensor_t sensor = (pulse_sensor_t)args;
    pulse_sensor_event_t m;
    ESP_LOGD(TAG, "Monitoring on GPIO %d", sensor->config.gpio_num);
    while (true)
    {
        if (!xQueueReceive(sensor->queue, &m, portMAX_DELAY))
        {
            ESP_LOGW(TAG, "Message timeout on GPIO %d queue. Bailing out.", sensor->config.gpio_num);
            break;
        }
        if (!xSemaphoreTake(sensor->mutex, MUTEX_TIMEOUT))
        {
            ESP_LOGW(TAG, "Failed to acquire mutex within %lu ticks on message %d on GPIO %d. Ignoring.",
                     MUTEX_TIMEOUT, m.type, sensor->config.gpio_num);
            continue;
        }

        switch (m.type)
        {
        case PULSE_SENSOR_PULSE:
            ESP_LOGV(TAG, "Pulse on GPIO %d", sensor->config.gpio_num);
            if (sensor->data.current_cycle_pulses == 0) // in tentative cycle
            {
                sensor->tentative_pulse_timestamp = m.timestamp;
                if (sensor->tentative_cycle_timestamp == 0)
                {
                    sensor->tentative_cycle_timestamp = m.timestamp;
                }
                if (sensor->tentative_cycle_pulses == 0) // first tentative pulse
                {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(
                        esp_timer_start_periodic(sensor->timer, sensor->config.check_period));
                    ESP_LOGD(TAG, "Started timer on GPIO %d", sensor->config.gpio_num);
                }
                sensor->tentative_cycle_pulses++;
                if (sensor->tentative_cycle_pulses == sensor->config.min_cycle_pulses)
                {
                    // graduate from a tentative to a real cycle and update aggregate stats
                    sensor->data.latest_pulse_timestamp = m.timestamp;
                    sensor->data.current_cycle_timestamp = sensor->tentative_cycle_timestamp;
                    sensor->data.current_cycle_pulses = sensor->tentative_cycle_pulses;
                    sensor->data.total_pulses += sensor->tentative_cycle_pulses;
                    sensor->data.total_duration += m.timestamp - sensor->tentative_cycle_timestamp;
                    sensor->data.cycles++;

                    // reset tentative stats
                    sensor->tentative_cycle_pulses = 0;
                    sensor->tentative_cycle_timestamp = 0;
                    sensor->tentative_pulse_timestamp = 0;

                    ESP_LOGI(TAG, "Cycle %lu started on GPIO %d", sensor->data.cycles, sensor->config.gpio_num);
                    pulse_sensor_notify(sensor, PULSE_SENSOR_CYCLE_STARTED, m.timestamp);
                }
            }
            else // in current cycle
            {
                sensor->data.current_cycle_pulses++;
                sensor->data.total_pulses++;
                sensor->data.total_duration += m.timestamp - sensor->data.latest_pulse_timestamp;
                sensor->data.latest_pulse_timestamp = m.timestamp;
            }
            sensor->latest_tick_pulses++;
            sensor->data.current_rate_pulses++;
            break;
        case PULSE_SENSOR_TICK:
            ESP_LOGV(TAG, "Tick on GPIO %d", sensor->config.gpio_num);
            if (sensor->tentative_cycle_pulses > 0) // in tentative cycle
            {
                if (m.timestamp - sensor->tentative_pulse_timestamp > sensor->config.cycle_timeout)
                {
                    ESP_LOGI(TAG, "Ignoring a partial cycle on GPIO %d after %lu pulses",
                             sensor->config.gpio_num, sensor->tentative_cycle_pulses);
                    sensor->data.partial_cycles++;
                    sensor->tentative_cycle_pulses = 0;
                    sensor->tentative_cycle_timestamp = 0;
                    sensor->tentative_pulse_timestamp = 0;
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sensor->timer));
                    pulse_sensor_notify(sensor, PULSE_SENSOR_CYCLE_IGNORED, m.timestamp);
                }
            }
            else if (sensor->data.current_cycle_pulses > 0) // in current cycle
            {
                if (m.timestamp - sensor->data.latest_pulse_timestamp > sensor->config.cycle_timeout)
                {
                    ESP_LOGI(TAG, "Cycle %lu stopped on GPIO %d after %llu pulses and %llu µs",
                             sensor->data.cycles, sensor->config.gpio_num,
                             sensor->data.current_cycle_pulses, m.timestamp - sensor->data.current_cycle_timestamp);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sensor->timer));
                    pulse_sensor_notify(sensor, PULSE_SENSOR_CYCLE_STOPPED, m.timestamp);
                    sensor->data.current_cycle_timestamp = 0;
                    sensor->data.current_cycle_pulses = 0;
                }
            }
            sensor->data.current_rate_timestamp = sensor->latest_tick_timestamp;
            sensor->data.current_rate_pulses = sensor->latest_tick_pulses;
            sensor->latest_tick_timestamp = m.timestamp;
            sensor->latest_tick_pulses = 0;
            break;
        default:
            ESP_LOGW(TAG, "Unexpected message type on GPIO %d queue: %d. Ignoring.",
                     sensor->config.gpio_num, m.type);
        }
        sensor->data.data_timestamp = m.timestamp;
        if (!xSemaphoreGive(sensor->mutex))
        {
            ESP_LOGW(TAG, "Failed to release mutex on message %d on GPIO %d.", m.type, sensor->config.gpio_num);
        }
    }
}

esp_err_t pulse_sensor_open(const pulse_sensor_config_t *config, pulse_sensor_t *sensor_out)
{
    esp_err_t ret = ESP_FAIL;
    ESP_GOTO_ON_FALSE(sensor_out, ESP_ERR_INVALID_ARG, error_handler, TAG, "null");

    const pulse_sensor_t sensor = calloc(1, sizeof(struct pulse_sensor_s));
    ESP_GOTO_ON_FALSE(sensor != NULL, ESP_ERR_NO_MEM, error_handler, TAG, "malloc pulse sensor");

    sensor->config = *config;
    const gpio_num_t gpio_num = sensor->config.gpio_num;

    sensor->mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(sensor->mutex, ESP_ERR_NO_MEM, free_sensor, TAG,
                      "create mutex on GPIO %d", config->gpio_num);

    sensor->queue = xQueueCreate(sensor->config.queue_size, sizeof(pulse_sensor_event_t));
    ESP_GOTO_ON_FALSE(sensor->queue != NULL, ESP_ERR_NO_MEM, free_mutex, TAG, "create queue");

    char name[64];
    snprintf(name, sizeof(name), "pulse sensor task on GPIO %d", gpio_num);
    const BaseType_t r = xTaskCreate(pulse_sensor_task, name, 2048,
                                     (void *)sensor, 1, &(sensor->task));
    ESP_GOTO_ON_FALSE(r == pdPASS, ESP_ERR_NO_MEM, queue_cleanup, TAG, "create task: %d", r);

    snprintf(name, sizeof(name), "pulse sensor timer on GPIO %d", gpio_num);
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_handler,
        .arg = (void *)sensor,
        .name = name};
    ESP_GOTO_ON_ERROR(esp_timer_create(&timer_args, &(sensor->timer)), task_cleanup, TAG, "create timer");

    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = true,
        .intr_type = GPIO_INTR_POSEDGE};
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), timer_cleanup, TAG, "config GPIO %d", gpio_num);

    ESP_GOTO_ON_ERROR(gpio_isr_handler_add(gpio_num, gpio_interrupt_handler, (void *)sensor),
                      gpio_config_cleanup, TAG, "add ISR handler on GPIO %d", gpio_num);

    *sensor_out = sensor;
    ESP_LOGI(TAG,
             "Opened on GPIO %d: min cycle pulses=%lu, cycle timeout=%llu, queue size=%lu",
             sensor->config.gpio_num,
             sensor->config.min_cycle_pulses,
             sensor->config.cycle_timeout,
             sensor->config.queue_size);
    return ESP_OK;

gpio_config_cleanup:
    gpio_config_t io_conf_reset = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf_reset));
timer_cleanup:
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_delete(sensor->timer));
task_cleanup:
    vTaskDelete(sensor->task);
queue_cleanup:
    vQueueDelete(sensor->queue);
free_mutex:
    vSemaphoreDelete(sensor->mutex);
free_sensor:
    free(sensor);
error_handler:
    ESP_LOGE(TAG, "Failed to open on GPIO %d: ret=0x%x", config->gpio_num, ret);
    return ret;
}

esp_err_t pulse_sensor_close(const pulse_sensor_t sensor)
{
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "sensor must not be NULL");
    const gpio_num_t gpio_num = sensor->config.gpio_num;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(gpio_num));
    gpio_config_t io_conf = {.pin_bit_mask = 1ULL << gpio_num};
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));

    if (sensor->timer)
    {
        if (sensor->tentative_cycle_pulses > 0 || sensor->data.current_cycle_pulses > 0)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_stop(sensor->timer));
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_delete(sensor->timer));
    }
    vTaskDelete(sensor->task);
    vQueueDelete(sensor->queue);
    vSemaphoreDelete(sensor->mutex);
    free(sensor);
    ESP_LOGI(TAG, "Closed on GPIO %d", gpio_num);
    return ESP_OK;
}

esp_err_t pulse_sensor_get_data(const pulse_sensor_t sensor, pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "sensor must not be NULL");
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "data must not be NULL");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(sensor->mutex, MUTEX_TIMEOUT), ESP_ERR_TIMEOUT, TAG, "acquire mutex");
    *data = sensor->data;
    ESP_RETURN_ON_FALSE(xSemaphoreGive(sensor->mutex), ESP_ERR_NOT_FINISHED, TAG, "release mutex");
    return ESP_OK;
}

float pulse_sensor_get_current_rate(const pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, 0, TAG, "NULL data");
    return data->current_rate_timestamp == 0
               ? 0
               : data->current_rate_pulses /
                     ((float)(data->data_timestamp - data->current_rate_timestamp) / 1000000);
}

uint64_t pulse_sensor_get_current_cycle_duration(const pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, 0, TAG, "NULL data");
    return data->current_cycle_timestamp == 0 ? 0 : data->data_timestamp - data->current_cycle_timestamp;
}

float pulse_sensor_get_current_cycle_rate(const pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, 0, TAG, "NULL data");
    return data->current_cycle_timestamp == 0
               ? 0
               : data->current_cycle_pulses /
                     ((float)pulse_sensor_get_current_cycle_duration(data) / 1000000);
}

float pulse_sensor_get_total_rate(const pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, 0, TAG, "NULL data");
    return data->total_duration == 0
               ? 0
               : data->total_pulses / ((float)data->total_duration / 1000000);
}

bool pulse_sensor_is_in_cycle(const pulse_sensor_data_t *data)
{
    ESP_RETURN_ON_FALSE(data, false, TAG, "NULL data");
    return data->current_cycle_pulses > 0;
}
