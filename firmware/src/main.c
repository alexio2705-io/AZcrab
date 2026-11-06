#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "servo.h"
#include "buttons.h"
#include "apds9960.h"
#include "gyro.h"
#include "motor.h"
#include "robot_link.h"
#include "drive_test.h"

static const char *TAG = "MAIN";

// Bench test: the two dispense moves, alternating with a gap long enough to
// reload a cube. Nothing else is initialised - no CAN, no I2C, no gyro - so
// the only thing that can move is the servo. Not run right now (see below) -
// kept so it's one flip of RUN_MAZE away from coming back.
#define DISPENSE_GAP_MS 3000

static void run_dispense_test(void)
{
    for (;;) {
        printf("=== dispense LEFT ===\n");
        servo_dispense_left();
        vTaskDelay(pdMS_TO_TICKS(DISPENSE_GAP_MS));

        printf("=== dispense RIGHT ===\n");
        servo_dispense_right();
        vTaskDelay(pdMS_TO_TICKS(DISPENSE_GAP_MS));
    }
}

// Bench test: only the panel button, nothing else initialised. Prints a line
// the moment the pin's raw level or debounced state changes - raw is the pin
// itself (1 idle, 0 held), edges counts raw changes since boot.
#define BUTTON_TEST_POLL_MS 1

static void run_button_test(void)
{
    buttons_init();
    int last = -1;
    for (;;) {
        int state = button_pressed(BTN_1) | button_raw_level(BTN_1) << 1;
        if (state != last) {
            last = state;
            printf("BTN_1(GPIO47) pressed=%d raw=%d edges=%lu\n",
                   button_pressed(BTN_1), button_raw_level(BTN_1),
                   (unsigned long)button_edge_count(BTN_1));
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_TEST_POLL_MS));
    }
}

// 1 = only print BTN_1, overriding RUN_MAZE below.
#define BUTTON_TEST 0

// 1 = тест шин: BTN_1 - проехать 50 см вперёд без RPi (drive_test.c).
// Перекрывает RUN_MAZE ниже. Перед лабиринтом вернуть 0.
#define DRIVE_TEST 0

// 1 = the maze run: sensors + motors, driven by the RPi over UART
// (robot_link.c). 0 = the servo dispense bench test instead.
#define RUN_MAZE 1

void app_main(void)
{
#if BUTTON_TEST
    run_button_test();
#endif

#if DRIVE_TEST
    ESP_LOGI(TAG, "=== Tyre drive test boot ===");
    buttons_init();
    gyro_init();
    motor_can_init();
    motor_enc_start();
    vTaskDelay(pdMS_TO_TICKS(200));
    motor_probe(1, 5);
    drive_test_run(); // never returns
#endif

#if RUN_MAZE
    ESP_LOGI(TAG, "=== Maze run boot ===");

    buttons_init();

    if (apds9960_board_init() == ESP_OK) ESP_LOGI(TAG, "APDS9960 OK");
    else                                 ESP_LOGE(TAG, "APDS9960 FAILED");

    gyro_init();

    motor_can_init();
    motor_enc_start();
    vTaskDelay(pdMS_TO_TICKS(200)); // let the RX task settle before probing
    motor_probe(1, 5);              // ids 1,2,3,5 as given, +4 as a spare check

    robot_link_run(); // never returns
#else
    servo_init();
    run_dispense_test();
#endif
}
