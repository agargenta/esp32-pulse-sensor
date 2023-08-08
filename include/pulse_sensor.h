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
 * @file pulse_sensor.h
 * @brief Interface definitions for interfacing with a GPIO-connected pulse-generating device.
 *
 * This component provides structures and functions that are useful for communicating
 * with a GPIO-connected device that generates periodic pulses, such as a flow meter.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define PULSE_SENSOR_MIN_CYCLE_PULSES 1
#define PULSE_SENSOR_CYCLE_TIMEOUT 1000000 // 1 second (in microseconds)

#define PULSE_SENSOR_QUEUE_SIZE 64
#define PULSE_SENSOR_NOTIFICATION_TIMEOUT pdMS_TO_TICKS(1) // 1 millisecond (in ticks)

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Notification types.
     */
    typedef enum
    {
        PULSE_SENSOR_CYCLE_STARTED = 1,
        PULSE_SENSOR_CYCLE_STOPPED = 2
    } pulse_sensor_notification_type_t;

    /**
     * @brief Notification definition.
     */
    typedef struct
    {
        pulse_sensor_notification_type_t type; /// type of the notification
        void *notification_arg;                /// configured argument
        int64_t timestamp;                     /// time (in microseconds since boot) when this event occurred
        uint64_t pulses;                       /// the number of pulses associated with this event
        uint64_t duration;                     /// the duration (in microseconds) associated with this event
    } pulse_sensor_notification_t;

    /**
     * @brief Configuration arguments structure.
     */
    typedef struct
    {
        gpio_num_t gpio_num;              /// GPIO number for this sensor (*required)
        uint32_t min_cycle_pulses;        /// minimum number of pulses for a cycle to be counted (>= 1); pulses below this threshold are discared (defaults to 1)
        uint64_t cycle_timeout;           /// max time (in microseconds) since the last pulse for a cycle to be considered finished (defaults to 1 second)
        uint32_t queue_size;              /// internal queue size for buffering pulses and time ticks (defaults to 100)
        QueueHandle_t notification_queue; /// notification queue to which to send cycle start/stop events (optional)
        TickType_t notification_timeout;  /// max time (in ticks) to wait to send a notification (defaults to 1 ms)
        void *notification_arg;           /// an argument to pass in each notification message (optional)
    } pulse_sensor_config_t;

    /**
     * @brief A pointer to an opaque structure representing an opened pulse sensor.
     */
    typedef struct pulse_sensor_s *pulse_sensor_h;

    /**
     * @brief Open a device instance with the specified config.
     * @param[in] config The device config instance.
     * @param[out] pulse_sensor_p Pointer to initialised pulse sensor instance.
     * @return ESP_OK if successfully opened; otherwise, an error code.
     */
    esp_err_t pulse_sensor_open(const pulse_sensor_config_t config, pulse_sensor_h *pulse_sensor_p);

    /**
     * @brief Close a device instance.
     * @param[in] pulse_sensor The pulse sensor to close.
     * @return ESP_OK if successfully closed; otherwise, an error code.
     */
    esp_err_t pulse_sensor_close(pulse_sensor_h pulse_sensor);

    /**
     * @brief Check if the pulse sensor is currently in an active cycle.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return true if in active cycle; false otherwise (including on any errors).
     */
    bool pulse_sensor_is_in_cycle(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the timestamp (microseconds since boot) of the latest pulse.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return microseconds since boot of the latest pulse cycle; -1 if pulse_sensor is null.
     */
    int64_t pulse_sensor_get_last_pulse_timestamp(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the timestamp (microseconds since boot) of when the current cycle started.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return microseconds since boot when the current cycle started; 0 if not in cycle;
     *         -1 if pulse_sensor is null.
     */
    int64_t pulse_sensor_get_current_cycle_timestamp(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the count of pulses since the current cycle started.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return the pulse count of the current cycle; 0 if not in cycle; -1 if pulse_sensor is null.
     */
    uint64_t pulse_sensor_get_current_cycle_pulses(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the duration (in microseconds) since the current cycle started.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return the duration (in microseconds) since the current cycle started; 0 if not in cycle;
     *         -1 if pulse_sensor is null.
     */
    uint64_t pulse_sensor_get_current_cycle_duration(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the total count of pulses since this device was opened.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return the pulse count since this device was opened (possibly 0); -1 if pulse_sensor is null.
     */
    uint64_t pulse_sensor_get_total_pulses(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the duration (in microseconds) of all cycles since the device was opened.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return the duration (in microseconds) of all cycles since the device was opened (possibly 0);
     *         -1 if pulse_sensor is null.
     */
    uint64_t pulse_sensor_get_total_duration(const pulse_sensor_h pulse_sensor);

    /**
     * @brief Get the total count of cycles since this device was opened.
     * @param[in] pulse_sensor The pulse sensor to check.
     * @return the total count of cycles since this device was opened (possibly 0);
     *         -1 if pulse_sensor is null.
     */
    uint32_t pulse_sensor_get_total_cycles(const pulse_sensor_h pulse_sensor);

#ifdef __cplusplus
}
#endif
