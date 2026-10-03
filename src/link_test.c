// ============================================================================
//  ESP<->RPi UART link test - see link_test.h for the wire protocol.
//
//  Одна пара проводов, полный дуплекс - подтверждено по нетлисту платы:
//  ESP GPIO3(TX, net TXD2) -> R23 -> Pi физ. пин 10 (GPIO15), ESP GPIO9(RX,
//  net RXD2) <- R8 <- Pi физ. пин 8 (GPIO14) - штатный /dev/serial0 на Pi.
// ============================================================================
#include "link_test.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "buttons.h"
#include "apds9960.h"
#include "gyro.h"
#include "motor.h"

static const char *TAG = "LINK_TEST";

// ---- UART к Pi: телеметрия наружу, команды внутрь, один и тот же порт -----
// Подтверждено по netlist платы: ESP GPIO3(TX, net TXD2) через R23 идёт на
// Pi физ. пин 10 (GPIO15), ESP GPIO9(RX, net RXD2) через R8 идёт на Pi физ.
// пин 8 (GPIO14) - штатный /dev/serial0 на Pi.
#define LINK_PORT   UART_NUM_1
#define LINK_TX     3
#define LINK_RX     9
#define LINK_BAUD   921600
#define UART_BUF    256

static void uart_init_port(uart_port_t port, int tx, int rx, int baud)
{
    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, UART_BUF * 2, UART_BUF * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


// ============================================================================
//  Телеметрия: ESP -> RPi
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t magic[2];    // 0xAA, 0x55
    float   yaw;
    float   pitch;
    uint8_t color;       // APDS9960_COLOR_* - только номер, как просили
    uint8_t clicker1;    // 0/1, дебаунс уже в buttons.c
    uint8_t clicker2;
    float   enc_left;    // град с последнего zero; NAN -> шлём 0 с online=0
    float   enc_right;
    float   lidar_echo;  // последнее число, принятое от Pi - подтверждает обратный канал
    uint8_t enc_online;  // бит0=left бит1=right, чтобы 0 не путался с "правда офлайн"
    uint8_t crc;         // XOR всех байт между magic и crc
} telem_t;

#define TELEM_SIZE sizeof(telem_t)

static uint8_t telem_crc(const telem_t *p)
{
    const uint8_t *b = (const uint8_t *)p + 2;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(telem_t) - 3; i++) crc ^= b[i];
    return crc;
}

// ============================================================================
//  Команды от RPi: "D:<float>\n"  "Z\n"
// ============================================================================
static volatile float g_lidar_echo = 0.0f;

static void cmd_task(void *arg)
{
    uint8_t line[64];
    int len = 0;

    for (;;) {
        uint8_t byte;
        if (uart_read_bytes(LINK_PORT, &byte, 1, pdMS_TO_TICKS(50)) <= 0)
            continue;

        if (byte == '\n') {
            line[len] = '\0';
            char *cmd = (char *)line;
            float v;
            if (sscanf(cmd, "D:%f", &v) == 1) {
                g_lidar_echo = v;
                ESP_LOGI(TAG, "RPi -> ESP: D=%.1f", v);
            } else if (cmd[0] == 'Z') {
                motor_zero_encoders();
                ESP_LOGI(TAG, "RPi -> ESP: Z (encoders zeroed)");
            }
            len = 0;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = byte;
        } else {
            len = 0; // слишком длинная строка - сбрасываем, ждём следующий \n
        }
    }
}

// ============================================================================
//  Телеметрия: периодическая отправка
// ============================================================================
#define TELEM_HZ 20

static void telem_task(void *arg)
{
    telem_t pkt = {.magic = {0xAA, 0x55}};
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        float yaw = 0.0f, pitch = 0.0f;
        gyro_get_angles(&yaw, &pitch, NULL);

        uint8_t color = 0;
        if (apds9960_is_data_ready(APDS9960_I2C_PORT)) {
            uint16_t r, g, b, c;
            if (apds9960_read_color_data(APDS9960_I2C_PORT, &r, &g, &b, &c) == ESP_OK)
                color = apds9960_detect_color(r, g, b, c);
        }

        float el = motor_left_enc_deg();
        float er = motor_right_enc_deg();

        pkt.yaw       = yaw;
        pkt.pitch     = pitch;
        pkt.color     = color;
        pkt.clicker1  = button_pressed(CLICKER_1) ? 1 : 0;
        pkt.clicker2  = button_pressed(CLICKER_2) ? 1 : 0;
        pkt.enc_left  = isnan(el) ? 0.0f : el;
        pkt.enc_right = isnan(er) ? 0.0f : er;
        pkt.lidar_echo = g_lidar_echo;
        pkt.enc_online = (isnan(el) ? 0 : 1) | (isnan(er) ? 0 : 2);
        pkt.crc       = telem_crc(&pkt);

        uart_write_bytes(LINK_PORT, (const char *)&pkt, TELEM_SIZE);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / TELEM_HZ));
    }
}

// ============================================================================
//  Человекочитаемый дамп в консоль по USB - чтобы стенд был виден без Pi
// ============================================================================
static void debug_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        float yaw = 0.0f, pitch = 0.0f;
        gyro_get_angles(&yaw, &pitch, NULL);
        float el = motor_left_enc_deg(), er = motor_right_enc_deg();

        printf("[LINK] yaw=%6.1f pitch=%5.1f  C1=%d C2=%d  "
               "encL=%s%.1f encR=%s%.1f  lidar_echo=%.1f\n",
               yaw, pitch,
               button_pressed(CLICKER_1), button_pressed(CLICKER_2),
               isnan(el) ? "off/" : "", isnan(el) ? 0.0f : el,
               isnan(er) ? "off/" : "", isnan(er) ? 0.0f : er,
               g_lidar_echo);

        motor_can_log_status();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
    }
}

// ============================================================================
void link_test_run(void)
{
    uart_init_port(LINK_PORT, LINK_TX, LINK_RX, LINK_BAUD);

    xTaskCreate(telem_task, "telem", 3072, NULL, 6, NULL);
    xTaskCreate(cmd_task,   "cmd",   3072, NULL, 6, NULL);
    xTaskCreate(debug_task, "debug", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "Link test running: TX=%d RX=%d @ %d (один порт, оба направления)",
             LINK_TX, LINK_RX, LINK_BAUD);

    // Все три задачи выше всё делают сами; эта просто ждёт.
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
