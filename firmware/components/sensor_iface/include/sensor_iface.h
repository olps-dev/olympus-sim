#ifndef SENSOR_IFACE_H
#define SENSOR_IFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BME680 sensor data structure
 */
typedef struct {
    float temperature;  // Temperature in Celsius
    float iaq;         // Indoor Air Quality index
} bme680_data_t;

/**
 * @brief Initialize sensor interfaces
 * @return true if successful, false otherwise
 */
bool sensor_iface_init(void);

/**
 * @brief Read BME680 sensor data
 * @param data Pointer to store sensor data
 * @return true if successful, false otherwise
 */
bool bme680_read(bme680_data_t *data);

/**
 * @brief Display text on SSD1306 OLED
 * @param text Text to display
 * @return true if successful, false otherwise
 */
bool ssd1306_draw_banner(const char *text);

/**
 * @brief Read battery voltage from ADC
 * @return Battery voltage in volts, or -1.0 on error
 */
float battery_read_voltage(void);

/**
 * @brief Start sensor monitoring task
 */
void sensor_task_start(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_IFACE_H 