#ifndef APDS9960_H
#define APDS9960_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

// APDS9960 colour sensor, on connector J13 (COLORSENSOR on the schematic).
// Shares the ESP's I2C bus with whatever else hangs off ESP_SCL/ESP_SDA:
//   ESP_SCL   -> GPIO6      APDS_INT -> GPIO5 (wired, not used - we poll)
//   ESP_SDA   -> GPIO7
// R5/R7 pull both lines up with 4.7k on the board, so the internal pull-ups
// stay off.
//
// Call this once at boot: it brings the I2C bus up on those pins and then
// runs apds9960_init() on it. Everything below takes an i2c_port_t so the
// driver itself stays board-agnostic - pass APDS9960_I2C_PORT.
#define APDS9960_I2C_PORT I2C_NUM_0

esp_err_t apds9960_board_init(void);

// Read the ID register (0x92). The cheapest "is the sensor still on the bus"
// probe there is: an error means the wiring or the bus is the problem, a good
// read with no colour data means the sensor is there but not integrating.
esp_err_t apds9960_read_id(i2c_port_t i2c_port, uint8_t *id);

// Color ID definitions returned by the detection algorithm
#define APDS9960_COLOR_WHITE   1
#define APDS9960_COLOR_BLUE    2
#define APDS9960_COLOR_BLACK   3
#define APDS9960_COLOR_MIRROR  4
#define APDS9960_COLOR_RED     5

/**
 * @brief Initialize the APDS9960 sensor
 * * @param i2c_port I2C port number where the sensor is connected
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t apds9960_init(i2c_port_t i2c_port);

/**
 * @brief Check if valid RGBC data is available
 * * @param i2c_port I2C port number
 * @return true if data is ready to be read, false otherwise
 */
bool apds9960_is_data_ready(i2c_port_t i2c_port);

/**
 * @brief Read clear, red, green, and blue color data from the sensor
 * * @param i2c_port I2C port number
 * @param r Pointer to store red value
 * @param g Pointer to store green value
 * @param b Pointer to store blue value
 * @param c Pointer to store clear value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t apds9960_read_color_data(i2c_port_t i2c_port, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);


uint8_t apds9960_detect_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c);

#endif // APDS9960_H