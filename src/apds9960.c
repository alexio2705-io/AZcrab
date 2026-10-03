#include "apds9960.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// APDS9960 I2C address
#define APDS9960_I2C_ADDR           0x39
#define APDS9960_I2C_TIMEOUT_MS     1000

// APDS9960 Sensor registers
#define APDS9960_REG_ENABLE         0x80
#define APDS9960_REG_ATIME          0x81
#define APDS9960_REG_CONTROL        0x8F
#define APDS9960_REG_ID             0x92
#define APDS9960_REG_STATUS         0x93
#define APDS9960_REG_CDATAL         0x94 
#define APDS9960_REG_RDATAL         0x96
#define APDS9960_REG_GDATAL         0x98
#define APDS9960_REG_BDATAL         0x9A

// Register bits
#define APDS9960_ENABLE_PON         (1 << 0) 
#define APDS9960_ENABLE_AEN         (1 << 1) 
#define APDS9960_STATUS_AVALID      (1 << 0)

#define APDS9960_SCL_GPIO 6
#define APDS9960_SDA_GPIO 7
#define APDS9960_I2C_HZ   400000

static const char *TAG = "APDS9960_LIB";

// --- Internal I2C Helper functions ---

static esp_err_t apds9960_write8(i2c_port_t i2c_port, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(i2c_port, APDS9960_I2C_ADDR, write_buf, sizeof(write_buf), APDS9960_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t apds9960_read8(i2c_port_t i2c_port, uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(i2c_port, APDS9960_I2C_ADDR, &reg, 1, value, 1, APDS9960_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t apds9960_read16(i2c_port_t i2c_port, uint8_t reg_l, uint16_t *value) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_write_read_device(i2c_port, APDS9960_I2C_ADDR, &reg_l, 1, buf, 2, APDS9960_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err == ESP_OK) {
        *value = ((uint16_t)buf[1] << 8) | buf[0];
    }
    return err;
}

static esp_err_t apds9960_set_again(i2c_port_t i2c_port, uint8_t gain) {
    uint8_t control = 0;
    esp_err_t err = apds9960_read8(i2c_port, APDS9960_REG_CONTROL, &control);
    if (err != ESP_OK) return err;
    
    control &= ~0x03; 
    control |= (gain & 0x03);
    return apds9960_write8(i2c_port, APDS9960_REG_CONTROL, control);
}

// --- Public API Implementation ---

esp_err_t apds9960_read_id(i2c_port_t i2c_port, uint8_t *id) {
    return apds9960_read8(i2c_port, APDS9960_REG_ID, id);
}

esp_err_t apds9960_init(i2c_port_t i2c_port) {
    uint8_t id = 0;
    esp_err_t err = apds9960_read8(i2c_port, APDS9960_REG_ID, &id);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with APDS9960. I2C Error: %d", err);
        return err;
    }
    
    ESP_LOGI(TAG, "Device ID: 0x%02X found", id);

    // Set integration time to 0xDB (~103ms)
    err |= apds9960_write8(i2c_port, APDS9960_REG_ATIME, 0xDB); 
    
    // Set gain to 16X (Value 2 means 16X in APDS9960 control register)
    err |= apds9960_set_again(i2c_port, 2);

    // Power ON and Enable ALS
    uint8_t enable = APDS9960_ENABLE_PON | APDS9960_ENABLE_AEN;
    err |= apds9960_write8(i2c_port, APDS9960_REG_ENABLE, enable);
    
    // Wait for sensor to power up and start integration
    vTaskDelay(pdMS_TO_TICKS(120));

    return err;
}

bool apds9960_is_data_ready(i2c_port_t i2c_port) {
    uint8_t status = 0;
    if (apds9960_read8(i2c_port, APDS9960_REG_STATUS, &status) == ESP_OK) {
        return (status & APDS9960_STATUS_AVALID) != 0;
    }
    return false;
}

esp_err_t apds9960_read_color_data(i2c_port_t i2c_port, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    esp_err_t err = ESP_OK;
    err |= apds9960_read16(i2c_port, APDS9960_REG_RDATAL, r);
    err |= apds9960_read16(i2c_port, APDS9960_REG_GDATAL, g);
    err |= apds9960_read16(i2c_port, APDS9960_REG_BDATAL, b);
    err |= apds9960_read16(i2c_port, APDS9960_REG_CDATAL, c);
    return err;
}

// Colour byte for the telemetry and the ESP console ONLY - nav.py does not use
// it, it classifies the tile itself from the raw channels (see tile_color()
// there, and the same thresholds mirrored in pi/color.py).
//
// The previous thresholds were "white above C=1400, black below C=200" and
// they no longer matched anything this sensor produces at its current mounting
// height: a white tile measures C=335, so EVERYTHING fell through to the
// MIRROR fallback - white tiles, black tiles and the robot held in mid-air
// alike. Measured on this robot, averaged over 3 s per tile:
//
//     white  R= 80 G=119 B=137 C=335
//     black  R=  8 G= 10 B= 10 C= 27
//     red    R= 43 G= 20 B= 39 C= 97
//
// Red is picked out by GREEN, not by blue: this tile's blue channel almost
// matches its red (39 vs 43), so (r-b)/c is a useless +0.04, while (r-g)/c is
// +0.237 against -0.074 (black) and -0.116 (white).
//
// These are absolute and therefore tied to this hall's lighting - unlike
// nav.py, which rescales its thresholds against the white start tile every
// run. Re-measure with color.py if the numbers here stop matching.
#define COLOR_BLACK_MAX_C   50   // sqrt(27*97): geometric middle of black..red
#define COLOR_WHITE_MIN_C  200   // white 335 vs the next brightest, red 97
#define COLOR_RED_RG_PCT    10   // (r-g)/c above 0.10 -> red
#define COLOR_BLUE_BR_PCT   32   // (b-r)/c above 0.32 -> blue

uint8_t apds9960_detect_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    // Signed on purpose: r-g on uint16 wraps around for every tile that is
    // not reddish, and the comparison would then always be true.
    int32_t ri = r, gi = g, bi = b, ci = c;

    if (ci < COLOR_BLACK_MAX_C) {
        return APDS9960_COLOR_BLACK;
    }
    if ((ri - gi) * 100 > ci * COLOR_RED_RG_PCT) {
        return APDS9960_COLOR_RED;
    }
    if ((bi - ri) * 100 > ci * COLOR_BLUE_BR_PCT) {
        return APDS9960_COLOR_BLUE;
    }
    if (ci > COLOR_WHITE_MIN_C) {
        return APDS9960_COLOR_WHITE;
    }

    // Genuinely ambiguous now: bright enough not to be black, dark enough not
    // to be white, and no colour cast. That is what MIRROR should mean.
    return APDS9960_COLOR_MIRROR;
}

esp_err_t apds9960_board_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = APDS9960_SDA_GPIO,
        .scl_io_num = APDS9960_SCL_GPIO,
        // R5/R7 already pull both lines up with 4.7k on the board.
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = APDS9960_I2C_HZ,
    };

    esp_err_t err = i2c_param_config(APDS9960_I2C_PORT, &cfg);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(APDS9960_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) return err;

    err = apds9960_init(APDS9960_I2C_PORT);
    if (err != ESP_OK) return err;

    // maze_2 overrode ATIME right after init, dropping integration from the
    // 0xDB (~103ms) set above to 0xF6 (~28ms), and the thresholds baked into
    // apds9960_detect_color() were tuned against that shorter integration.
    // Keep it, or every colour call comes out roughly 4x too bright.
    // (The gain it also rewrote there, 16x, is what init already sets.)
    return apds9960_write8(APDS9960_I2C_PORT, APDS9960_REG_ATIME, 0xF6);
}
