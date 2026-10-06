// ============================================================================
//  ESP <-> RPi link for the maze run - see robot_link.h for the wire protocol.
// ============================================================================
#include "robot_link.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "buttons.h"
#include "apds9960.h"
#include "gyro.h"
#include "motor.h"
#include "servo.h"

static const char *TAG = "LINK";

#define LINK_PORT   UART_NUM_1
#define LINK_TX     3
#define LINK_RX     9
#define LINK_BAUD   921600
#define UART_BUF    512

// 200 Hz. The encoders are polled at half that (ENC_POLL_HZ = 100 in motor.c),
// so odometry repeats in every other packet. See robot_link.h on why the
// packet size is irrelevant to this choice (9% of the link at 200 Hz).
#define TELEM_HZ        200
#define COLOR_POLL_HZ   30
#define CMD_TIMEOUT_US  500000   // no "M:" for this long while moving -> brake

// How long a "B" keeps actively holding 0 dps, and how often that hold is
// re-asserted. Long enough to cover the RPi standing still measuring walls or
// turning; bounded so a robot left stopped at the end of a run eventually goes
// silent instead of sitting there heating four motors.
#define BRAKE_HOLD_US        3000000
#define BRAKE_HOLD_PERIOD_US   40000

// nav.py's own forward-speed ramp (RAMP_UP_S, time-based smoothstep) now sets
// the real acceleration profile: for RAMP_UP_S=0.25s and MOVE_MAX_CMS=90cm/s
// on an 86mm wheel, its peak demand is ~7200 dps/s. This cap only needs to
// clear that with headroom so it is never the tighter constraint - it used
// to be the OTHER way around (a distance-based ramp on the RPi side whose
// required dv/dt grew without bound as speed rose, this cap was what capped
// it, and the mismatch between the two was read as a jerk partway through
// the move). Keep this above nav.py's peak; raise both together if needed.
#define MOTOR_ACCEL_DPS_S 11000

// Потолок скорости колеса в режиме момента, если "T:" пришла без третьего
// поля (см. robot_link.h). Порядка скорости переезда препятствия в nav.py.
#define ASSIST_CAP_CMS 35.0f

typedef enum {
    RUN_RUNNING = 0,
    RUN_PAUSED  = 1,
} run_state_t;

// The run is armed from boot - the robot starts as soon as the RPi sends the
// first drive command. BTN_1 pauses (restart from checkpoint), the RPi's "R"
// resumes - see btn_task().
static volatile run_state_t g_run_state = RUN_RUNNING;
static volatile uint8_t     g_color     = APDS9960_COLOR_BLACK;
// Сырые каналы датчика идут в телеметрию рядом с готовым номером цвета:
// пороги классификации подбираются по ним, а на глаз по одному номеру
// понять, почему плитка определилась не так, невозможно.
static volatile uint16_t    g_cr, g_cg, g_cb, g_cc;

// Моторами командуют две задачи - cmd_task и btn_task (пауза). Без общего
// замка cmd_task мог проверить g_run_state == RUNNING, тут же BTN_1 ставил
// паузу и тормозил, а cmd_task следом всё равно отправлял скорость; и
// g_motors_stopped в motor.c правился из обеих задач вперемешку. Под этим
// замком - g_run_state и любой вызов robot_*() после старта задач.
static SemaphoreHandle_t g_motor_lock;

static void motor_lock(void)   { xSemaphoreTake(g_motor_lock, portMAX_DELAY); }
static void motor_unlock(void) { xSemaphoreGive(g_motor_lock); }

typedef struct __attribute__((packed)) {
    uint8_t magic[2];
    uint8_t ver;     // TELEM_VER - см. robot_link.h, зачем он нужен
    float   yaw;
    float   yaw_rate;
    float   pitch;
    float   roll;    // hands, not ramps - see robot_link.h
    float   ax, ay, az;   // linear acceleration, m/s^2, sensor frame
    uint8_t color;
    uint16_t cr, cg, cb, cc;   // сырые каналы APDS9960, 0..10250 при ATIME=0xF6
    float   left_cm;
    float   right_cm;
    uint8_t status;
    uint8_t flags;
    uint8_t crc;
} telem_t;

static uint8_t telem_crc(const telem_t *p)
{
    const uint8_t *b = (const uint8_t *)p + 2;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(telem_t) - 3; i++) crc ^= b[i];
    return crc;
}

static void uart_init_port(void)
{
    uart_config_t cfg = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_PORT, UART_BUF * 2, UART_BUF * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_PORT, LINK_TX, LINK_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// The colour sensor is the only thing here that costs an I2C transaction, so
// it gets its own slow task instead of stalling the telemetry loop.
static void color_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (apds9960_is_data_ready(APDS9960_I2C_PORT)) {
            uint16_t r, g, b, c;
            if (apds9960_read_color_data(APDS9960_I2C_PORT, &r, &g, &b, &c) == ESP_OK) {
                g_cr = r; g_cg = g; g_cb = b; g_cc = c;
                g_color = apds9960_detect_color(r, g, b, c);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / COLOR_POLL_HZ));
    }
}

static void telem_task(void *arg)
{
    telem_t pkt = {.magic = {0xAA, 0x55}, .ver = TELEM_VER};
    TickType_t last = xTaskGetTickCount();
    esp_task_wdt_add(NULL);   // см. robot_link_run() про TWDT

    for (;;) {
        esp_task_wdt_reset();
        float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
        gyro_get_angles(&yaw, &pitch, &roll);

        float lmm = robot_left_dist_mm();
        float rmm = robot_right_dist_mm();

        pkt.yaw      = yaw;
        pkt.yaw_rate = gyro_yaw_rate();
        pkt.pitch    = pitch;
        pkt.roll     = roll;
        // Три компоненты, а не модуль: модуль не отличает подъём от тряски
        // на лесенке, разделить их можно только по вертикали. См. robot_link.h.
        //
        // Через локальные переменные, а не &pkt.ax напрямую: telem_t упакован,
        // её float'ы лежат по нечётным смещениям, и указатель на такое поле -
        // неопределённое поведение. ESP32-S3 невыровненный доступ переваривает
        // аппаратно, так что работало и так, но это капкан на будущее.
        float lax = 0.0f, lay = 0.0f, laz = 0.0f;
        gyro_get_lin_accel(&lax, &lay, &laz);
        pkt.ax = lax;
        pkt.ay = lay;
        pkt.az = laz;
        pkt.color    = g_color;
        pkt.cr       = g_cr;
        pkt.cg       = g_cg;
        pkt.cb       = g_cb;
        pkt.cc       = g_cc;
        pkt.left_cm  = isnan(lmm) ? 0.0f : lmm * 0.1f;
        pkt.right_cm = isnan(rmm) ? 0.0f : rmm * 0.1f;
        pkt.status   = (uint8_t)g_run_state;
        pkt.flags    = (button_pressed(CLICKER_1) ? 1 : 0)
                     | (button_pressed(CLICKER_2) ? 2 : 0)
                     | (isnan(lmm) ? 0 : 4)
                     | (isnan(rmm) ? 0 : 8)
                     | (button_pressed(BTN_1) ? 16 : 0);
        pkt.crc      = telem_crc(&pkt);

        uart_write_bytes(LINK_PORT, (const char *)&pkt, sizeof(pkt));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / TELEM_HZ));
    }
}

static void cmd_task(void *arg)
{
    char    line[64];
    int     len = 0;
    int64_t last_cmd_us  = esp_timer_get_time();
    int64_t hold_until_us = 0;   // keep re-asserting the 0 dps hold until then
    int64_t last_hold_us  = 0;
    bool    moving = false;
    esp_task_wdt_add(NULL);   // см. robot_link_run() про TWDT

    for (;;) {
        esp_task_wdt_reset();
        uint8_t byte;
        if (uart_read_bytes(LINK_PORT, &byte, 1, pdMS_TO_TICKS(20)) > 0) {
            if (byte == '\n' || byte == '\r') {
                line[len] = '\0';
                float l = 0.0f, r = 0.0f, cap = 0.0f;
                // "K:" держит задачу ~1.3с на ходу серво - не под замком,
                // иначе BTN_1 всё это время не смог бы поставить паузу.
                bool motor_cmd = strncmp(line, "K:", 2) != 0;
                if (motor_cmd) motor_lock();

                if (sscanf(line, "M:%f,%f", &l, &r) == 2) {
                    last_cmd_us = esp_timer_get_time();
                    hold_until_us = 0;              // driving again, drop the hold
                    if (g_run_state == RUN_RUNNING && (l != 0.0f || r != 0.0f)) {
                        robot_set_side_speed_cms(l, r);
                        moving = true;
                    } else {
                        // Idle command, or the run is not armed: go silent
                        // rather than hold zero and whine on the spot.
                        robot_stop_all();
                        moving = false;
                    }
                } else if (strncmp(line, "T:", 2) == 0
                           && sscanf(line, "T:%f,%f,%f", &l, &r, &cap) >= 2) {
                    // Третье поле (потолок скорости) необязательно: старый
                    // nav.py его не шлёт, и тогда действует ASSIST_CAP_CMS.
                    if (!(cap > 0.0f)) cap = ASSIST_CAP_CMS;
                    last_cmd_us = esp_timer_get_time();
                    hold_until_us = 0;
                    if (g_run_state == RUN_RUNNING && (l != 0.0f || r != 0.0f)) {
                        robot_set_side_torque_amps(l, r, cap);
                        moving = true;
                    } else {
                        robot_stop_all();
                        moving = false;
                    }
                } else if (line[0] == 'B') {
                    last_cmd_us = esp_timer_get_time();
                    moving = false;
                    if (g_run_state == RUN_RUNNING) {
                        robot_brake();
                        hold_until_us = last_cmd_us + BRAKE_HOLD_US;
                        last_hold_us  = last_cmd_us;
                    } else {
                        // Пауза BTN_1: робот в руках. Тормоз будит моторы и
                        // держит 0 dps - молчим, как на "M:" в паузе.
                        robot_stop_all();
                    }
                } else if (line[0] == 'Z') {
                    last_cmd_us = esp_timer_get_time();
                    motor_zero_encoders();
                } else if (line[0] == 'R') {
                    // Снять паузу BTN_1 - см. btn_task().
                    last_cmd_us = esp_timer_get_time();
                    if (g_run_state == RUN_PAUSED) {
                        g_run_state = RUN_RUNNING;
                        ESP_LOGI(TAG, "RPi: RESUME");
                    }
                } else if (strncmp(line, "K:", 2) == 0) {
                    // Дозатор дёргается только пока робот стоит (RPi шлёт это
                    // после остановки на жертве), поэтому блокировать cmd_task
                    // на время хода серво (~1с) безопасно - "M:" в этот момент
                    // всё равно не идут.
                    last_cmd_us = esp_timer_get_time();
                    if (line[2] == 'L') servo_dispense_left();
                    else if (line[2] == 'R') servo_dispense_right();
                }
                if (motor_cmd) motor_unlock();
                len = 0;
            } else if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)byte;
            } else {
                len = 0;   // overlong line, resync on the next newline
            }
        }

        motor_lock();
        int64_t now = esp_timer_get_time();

        // На паузе моторы заглушил btn_task. Ни удержание тормоза, ни
        // failsafe не должны их снова будить - иначе робот в руках держит
        // 0 dps до BRAKE_HOLD_US.
        if (g_run_state == RUN_PAUSED) {
            hold_until_us = 0;
            moving = false;
        }

        // A closed-loop speed command has to keep arriving or the motor falls
        // out of closed loop (see motor.h) - a single 0 dps frame is a brake
        // that quietly lets go a moment later. The RPi can be silent for a few
        // hundred ms after a stop (it is measuring walls), so re-assert the
        // hold here instead of relying on it to do so.
        if (hold_until_us) {
            if (now >= hold_until_us) {
                robot_stop_all();     // long enough - go silent, don't cook the motors
                hold_until_us = 0;
            } else if (now - last_hold_us >= BRAKE_HOLD_PERIOD_US) {
                robot_brake();
                last_hold_us = now;
            }
        }

        if (moving && (now - last_cmd_us) > CMD_TIMEOUT_US) {
            robot_brake();
            moving = false;
            hold_until_us = now + BRAKE_HOLD_US;
            last_hold_us  = now;
            ESP_LOGW(TAG, "failsafe: no command for %d ms, braking",
                     (int)(CMD_TIMEOUT_US / 1000));
        }
        motor_unlock();
    }
}

// BTN_1 - перезапуск с чекпоинта (lack of progress): только СТАВИТ паузу,
// моторы сразу тормозят и замолкают. Снимает её не кнопка, а RPi командой
// "R" - после того как робота поставили на чекпоинт и нажали GPIO25 на
// малине (nav.py, handle_lift). Раньше BTN_1 переключал пауза/ход, и второе
// нажатие запускало бы робота, пока RPi ещё ждёт свою кнопку.
static void btn_task(void *arg)
{
    for (;;) {
        if (button_clicked(BTN_1)) {
            motor_lock();
            bool pause = g_run_state == RUN_RUNNING;
            if (pause) {
                g_run_state = RUN_PAUSED;
                robot_brake();
            }
            motor_unlock();
            if (pause) {
                vTaskDelay(pdMS_TO_TICKS(150));
                motor_lock();
                if (g_run_state == RUN_PAUSED) robot_stop_all();  // "R" мог успеть снять
                motor_unlock();
                ESP_LOGI(TAG, "button: PAUSE (restart from checkpoint)");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Каждые столько печатей состояния - одна строка состояния шины CAN.
// Отладка "энкодер не отвечает" без неё сводится к гаданию: по одному лишь
// L=.../R=... не видно, теряются кадры при отправке, переполняется ли приём
// или мотор просто молчит. См. motor_can_log_status().
#define DEBUG_CAN_EVERY 4

static void debug_task(void *arg)
{
    static const char *COLOR[] = {"?", "WHITE", "BLUE", "BLACK", "MIRROR", "RED"};
    static const char *STATE[] = {"RUN", "PAUSED"};
    TickType_t last = xTaskGetTickCount();
    int n = 0;

    for (;;) {
        float yaw = 0.0f, pitch = 0.0f;
        gyro_get_angles(&yaw, &pitch, NULL);
        float l = robot_left_dist_mm(), r = robot_right_dist_mm();

        printf("[%s] yaw=%6.1f pitch=%5.1f  %-6s  B1=%d C1=%d C2=%d  L=%s%.1f R=%s%.1f cm\n",
               STATE[g_run_state == RUN_PAUSED ? 1 : 0], yaw, pitch,
               COLOR[g_color < 6 ? g_color : 0],
               button_pressed(BTN_1),
               button_pressed(CLICKER_1), button_pressed(CLICKER_2),
               isnan(l) ? "off/" : "", isnan(l) ? 0.0f : l * 0.1f,
               isnan(r) ? "off/" : "", isnan(r) ? 0.0f : r * 0.1f);

        if (++n >= DEBUG_CAN_EVERY) {
            n = 0;
            motor_can_log_status();
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
    }
}

void robot_link_run(void)
{
    uart_init_port();
    servo_init();
    g_motor_lock = xSemaphoreCreateMutex();
    configASSERT(g_motor_lock);

    robot_wake_all();
    robot_set_accel(MOTOR_ACCEL_DPS_S);
    motor_zero_encoders();
    robot_stop_all();   // silent until the RPi's first drive command

    // telem, cmd (здесь) и enc_poll (motor.c) подписаны на task watchdog:
    // failsafe живёт в самой cmd_task, и её зависания иначе не заметил бы
    // никто. CONFIG_ESP_TASK_WDT_PANIC выключен - через 5с без сброса будет
    // только "Task watchdog got triggered" с именем задачи в консоли, без
    // перезагрузки. Самый долгий законный простой - "K:" (~1.3с).
    xTaskCreate(telem_task, "telem", 3072, NULL, 6,  NULL);
    xTaskCreate(cmd_task,   "cmd",   3072, NULL, 11, NULL);
    xTaskCreate(color_task, "color", 3072, NULL, 5,  NULL);
    xTaskCreate(btn_task,   "btn",   2560, NULL, 7,  NULL);
    xTaskCreate(debug_task, "debug", 3072, NULL, 2,  NULL);

    ESP_LOGI(TAG, "link up: TX=%d RX=%d @ %d, accel=%d dps/s. Armed - BTN_1 pauses, RPi 'R' resumes.",
             LINK_TX, LINK_RX, LINK_BAUD, MOTOR_ACCEL_DPS_S);

    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
