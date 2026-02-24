#pragma once

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_ULTRASONIC_PING         0x200   ///< No ping (trigger sent, no echo start)
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201   ///< No echo start within timeout
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202   ///< Echo too long (object too far)

/**
 * HC-SR04 ultrasonic sensor descriptor.
 */
typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} ultrasonic_sensor_t;

/**
 * Initialize trigger and echo GPIOs.
 */
esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);

/**
 * Measure distance.
 * @param dev          Sensor descriptor
 * @param max_distance Maximum measurable distance in cm
 * @param distance     Result in meters
 * @return ESP_OK or error code
 */
esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, uint32_t max_distance_cm, float *distance_m);

#ifdef __cplusplus
}
#endif
