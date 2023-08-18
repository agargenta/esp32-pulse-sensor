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
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_err.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief A pointer to an opaque structure representing an opened pulse sensor.
     */
    typedef struct pulse_sensor_s *pulse_sensor_t;

    /**
     * @brief Notification types.
     */
    typedef enum
    {
        PULSE_SENSOR_CYCLE_STARTED,
        PULSE_SENSOR_CYCLE_STOPPED,
        PULSE_SENSOR_CYCLE_IGNORED,
    } pulse_sensor_notification_type_t;

    /**
     * @brief Notification definition.
     */
    typedef struct
    {
        pulse_sensor_t sensor;                 /// the sensor that generated this event
        void *notification_arg;                /// configured argument
        pulse_sensor_notification_type_t type; /// type of the notification
        int64_t timestamp;                     /// time (in microseconds since boot) when this event occurred
    } pulse_sensor_notification_t;

    /**
     * @brief Configuration arguments structure.
     */
    typedef struct
    {
        gpio_num_t gpio_num;              /// GPIO number for this sensor (*required)
        uint32_t min_cycle_pulses;        /// minimum number of pulses for a cycle to be counted (>= 1); pulses below this threshold are discared (defaults to 1)
        uint64_t cycle_timeout;           /// max time (in microseconds) since the latest pulse for a cycle to be considered finished (defaults to 1 second)
        uint64_t check_period;            /// time (in microseconds) between checks for the current rate and cycle status (defaults to 1 second)
        uint32_t queue_size;              /// internal queue size for buffering pulses and time ticks (defaults to 100)
        QueueHandle_t notification_queue; /// notification queue to which to send cycle start/stop events (optional)
        TickType_t notification_timeout;  /// max time (in ticks) to wait to send a notification (defaults to 1 ms)
        void *notification_arg;           /// an argument to pass in each notification message (optional)
    } pulse_sensor_config_t;

#define PULSE_SENSOR_CONFIG_DEFAULT()             \
    {                                             \
        .min_cycle_pulses = 1,                    \
        .cycle_timeout = 1000000,                 \
        .check_period = 1000000,                  \
        .queue_size = 64,                         \
        .notification_timeout = pdMS_TO_TICKS(1), \
    }

    typedef struct
    {
        int64_t data_timestamp;          /// the time (µs since boot) when this data was last updated
        int64_t latest_pulse_timestamp;  /// the time (µs since boot) of the latest pulse
        int64_t current_rate_timestamp;  /// the time (µs since boot) of the current rate measurement period (~ 1-2 cycle_timeout ago)
        uint32_t current_rate_pulses;    /// the number of pulses since current_rate_timestamp
        int64_t current_cycle_timestamp; /// the time (usec since boot) of the start of the current cycle
        uint64_t current_cycle_pulses;   /// the number of pulses since current_cycle_timestamp
        uint64_t total_pulses;           /// the number of pulses since this sensor was opened
        uint64_t total_duration;         /// total time (µs) during which there were continuous pulses, since thi sensor was opened
        uint32_t cycles;                 /// the number of cycles since this sensor was opened
        uint32_t partial_cycles;         /// the number of would-be cycles since this sensor was opened (these are the cycles that timed out before being counted)
    } pulse_sensor_data_t;

    /**
     * @brief Open a device instance with the specified config.
     * @param[in] config Pointer to the sensor config instance.
     * @param[out] sensor_out Pointer to initialised pulse sensor instance.
     * @return ESP_OK if successfully opened; otherwise, an error code.
     */
    esp_err_t pulse_sensor_open(const pulse_sensor_config_t *config, pulse_sensor_t *sensor_out);

    /**
     * @brief Close a device instance.
     * @param[in] sensor The pulse sensor to close.
     * @return ESP_OK if successfully closed; otherwise, an error code.
     */
    esp_err_t pulse_sensor_close(const pulse_sensor_t sensor);

    /**
     * @brief Get the current state of the sensor's data.
     * @param[in] data The pulse sensor from which to obtain data.
     * @return ESP_OK if the data was successfully obtained; otherwise, an error code.
     */
    esp_err_t pulse_sensor_get_data(const pulse_sensor_t sensor, pulse_sensor_data_t *data);

    /**
     * @brief A convenience function to get the current pluse rate (pulses per second).
     * @param[in] data The pulse sensor data obtained via pulse_sensor_get_data.
     * @return The current pulse rate (pulses per second).
     */
    float pulse_sensor_get_current_rate(const pulse_sensor_data_t *data);

    /**
     * @brief A convenience function to get the current cycle duration.
     * @param[in] data The pulse sensor data obtained via pulse_sensor_get_data.
     * @return The current cycle pulse duration (µs).
     */
    uint64_t pulse_sensor_get_current_cycle_duration(const pulse_sensor_data_t *data);

    /**
     * @brief A convenience function to get the current cycle pluse rate (pulses per second).
     * @param[in] data The pulse sensor data obtained via pulse_sensor_get_data.
     * @return The current cycle pulse rate (pulses per second).
     */
    float pulse_sensor_get_current_cycle_rate(const pulse_sensor_data_t *data);

    /**
     * @brief A convenience function to get the total pluse rate (pulses per second).
     * @param[in] data The pulse sensor data obtained via pulse_sensor_get_data.
     * @return The total (cumulative) pulse rate (pulses per second).
     */
    float pulse_sensor_get_total_rate(const pulse_sensor_data_t *data);

    /**
     * @brief A convenience function to check if the pulse sensor is currently in an active cycle.
     * @param[in] sensor The pulse sensor to check.
     * @return true if in active cycle; false otherwise (including on any errors).
     */
    bool pulse_sensor_is_in_cycle(const pulse_sensor_data_t *data);

#ifdef __cplusplus
}
#endif
