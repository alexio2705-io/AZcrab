#include "buttons.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BTN";

static const struct {
    gpio_num_t pin;
    bool active_low;
    const char *name;
} INPUTS[BUTTON_COUNT] = {
    // Panel button: 10k pull-up, switch shorts to ground, so the line drops
    // low while held.
    [BTN_1] = {GPIO_NUM_47, true, "BTN_1"},
    // The clickers measure the other way round on the assembled board - they
    // sit low at rest and go high when pressed - so their sense is flipped
    // here rather than at every call site.
    [CLICKER_1] = {GPIO_NUM_1, false, "CLICKER_1"},
    [CLICKER_2] = {GPIO_NUM_4, false, "CLICKER_2"},
};

#define POLL_MS      5
#define DEBOUNCE_CNT 6 // 6 * 5ms = 30ms of agreement before a state change

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_pressed[BUTTON_COUNT];
static bool s_click[BUTTON_COUNT];
static uint32_t s_edges[BUTTON_COUNT];

static bool read_down(int i)
{
    int level = gpio_get_level(INPUTS[i].pin);
    return INPUTS[i].active_low ? (level == 0) : (level != 0);
}

static void buttons_task(void *arg)
{
    uint8_t down[BUTTON_COUNT] = {0};
    uint8_t up[BUTTON_COUNT] = {0};
    int prev_raw[BUTTON_COUNT];

    for (int i = 0; i < BUTTON_COUNT; i++) {
        prev_raw[i] = gpio_get_level(INPUTS[i].pin);
    }

    for (;;) {
        for (int i = 0; i < BUTTON_COUNT; i++) {
            // Raw edge tally, before debounce and before polarity: sampled
            // every 5ms and never cleared, so a press of any length leaves a
            // permanent mark even if its log line scrolls away.
            int raw = gpio_get_level(INPUTS[i].pin);
            if (raw != prev_raw[i]) {
                prev_raw[i] = raw;
                portENTER_CRITICAL(&s_lock);
                s_edges[i]++;
                portEXIT_CRITICAL(&s_lock);
            }

            if (read_down(i)) {
                up[i] = 0;
                // Clamped, so the transition fires exactly once per press no
                // matter how long the button is held.
                if (down[i] < DEBOUNCE_CNT && ++down[i] == DEBOUNCE_CNT) {
                    portENTER_CRITICAL(&s_lock);
                    s_pressed[i] = true;
                    s_click[i] = true;
                    portEXIT_CRITICAL(&s_lock);
                    if (i == BTN_1) {
                        ESP_LOGI(TAG, "%s (GPIO%d) pressed", INPUTS[i].name, INPUTS[i].pin);
                    }
                }
            } else {
                down[i] = 0;
                if (up[i] < DEBOUNCE_CNT && ++up[i] == DEBOUNCE_CNT) {
                    portENTER_CRITICAL(&s_lock);
                    s_pressed[i] = false;
                    portEXIT_CRITICAL(&s_lock);
                    if (i == BTN_1) {
                        ESP_LOGI(TAG, "%s (GPIO%d) released", INPUTS[i].name, INPUTS[i].pin);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void buttons_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < BUTTON_COUNT; i++) {
        mask |= 1ULL << INPUTS[i].pin;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_INPUT,
        // Redundant with the 10k on the board, but it keeps the input from
        // floating if a resistor is ever left unpopulated.
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    // 3k rather than 2k: ESP_LOGI formats on this task's stack.
    xTaskCreate(buttons_task, "buttons", 3072, NULL, 5, NULL);
}

bool button_pressed(button_t b)
{
    if (b >= BUTTON_COUNT) return false;

    portENTER_CRITICAL(&s_lock);
    bool pressed = s_pressed[b];
    portEXIT_CRITICAL(&s_lock);
    return pressed;
}

bool button_clicked(button_t b)
{
    if (b >= BUTTON_COUNT) return false;

    portENTER_CRITICAL(&s_lock);
    bool click = s_click[b];
    s_click[b] = false;
    portEXIT_CRITICAL(&s_lock);
    return click;
}

int button_raw_level(button_t b)
{
    if (b >= BUTTON_COUNT) return -1;
    return gpio_get_level(INPUTS[b].pin);
}

uint32_t button_edge_count(button_t b)
{
    if (b >= BUTTON_COUNT) return 0;

    portENTER_CRITICAL(&s_lock);
    uint32_t n = s_edges[b];
    portEXIT_CRITICAL(&s_lock);
    return n;
}
