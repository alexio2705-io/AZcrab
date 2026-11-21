// ============================================================================
//  ESP <-> RPi link for the maze run - see robot_link.h for the wire protocol.
// ============================================================================
#include "robot_link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_random.h"

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
// packet size is modest (16% of the link at 200 Hz with v4).
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

// Предел скорости колеса в "M:" и потолка в "T:", см/с на ободе (~2000 dps).
// nav.py шлёт не больше 83 (MOVE_MAX_CMS 55 + MAX_STEER_CMS 28) - это
// страховка от мусора, а не рабочее ограничение. NaN/inf в "M:"/"T:" (sscanf
// их принимает) отбрасывают строку целиком: lroundf(NaN) дал бы мотору
// неопределённую скорость.
#define LINK_MAX_CMS 150.0f

static float clamp_abs(float v, float lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

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
// Время последнего удачного чтения APDS9960 - флаг свежести цвета.
static volatile int64_t     g_color_ok_us;
// Сколько "Z" применено (по модулю 256) - растёт ПОСЛЕ обнуления энкодеров,
// так что пакет с новым значением заведомо несёт одометрию после "Z".
static volatile uint8_t     g_z_cnt;
// Сколько строк от RPi отвергнуто: нет/не та CRC, не разобралась, NaN,
// слишком длинная.
static volatile uint16_t    g_cmd_err;
// Случайное при каждом старте: RPi по его смене видит ребут ESP.
static uint16_t             g_boot_id;

// Флаги свежести (см. robot_link.h): гироскоп - счётчик сэмплов BNO085
// (1 кГц) рос за последние столько; цвет - удачное чтение не старше этого
// (опрос 30 Гц).
#define GYRO_FRESH_US   50000
#define COLOR_FRESH_US 150000

// Моторами командуют две задачи - cmd_task и btn_task (пауза). Без общего
// замка cmd_task мог проверить g_run_state == RUNNING, тут же BTN_1 ставил
// паузу и тормозил, а cmd_task следом всё равно отправлял скорость; и
// g_motors_stopped в motor.c правился из обеих задач вперемешку. Под этим
// замком - g_run_state и любой вызов robot_*() после старта задач.
static SemaphoreHandle_t g_motor_lock;
static QueueHandle_t g_servo_queue;
static portMUX_TYPE g_color_lock = portMUX_INITIALIZER_UNLOCKED;
// Protected by g_motor_lock. Contact stays visible until a deliberate reverse
// or a brake acknowledgement after release; queued forward commands cannot restart.
static bool g_forward;
static uint8_t g_bumper_hits;

static bool forward_translation(float left, float right)
{
    // Counter-rotating wheels are an in-place turn: the user explicitly
    // excludes these from bumper stopping. Reverse also remains available.
    return left >= 0 && right >= 0 && (left > 0 || right > 0);
}

static uint8_t bumper_now(void)
{
    return (button_pressed(CLICKER_1) ? 1 : 0) |
           (button_pressed(CLICKER_2) ? 2 : 0);
}

static void motor_lock(void)   { xSemaphoreTake(g_motor_lock, portMAX_DELAY); }
static void motor_unlock(void) { xSemaphoreGive(g_motor_lock); }

typedef struct __attribute__((packed)) {
    uint8_t magic[2];
    uint8_t ver;     // TELEM_VER - см. robot_link.h, зачем он нужен
    uint8_t seq;     // +1 на каждый пакет - RPi считает пропуски
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
    uint8_t  z_cnt;     // g_z_cnt
    uint16_t boot_id;   // g_boot_id
    uint16_t cmd_err;   // g_cmd_err
    uint16_t motor_dv[4]; // IDs 4,1,2,3 (RL, FL, FR, RR); 0 = unavailable
    int8_t motor_temp[4]; // -128 = unavailable
    uint8_t motor_error[4];
    uint16_t crc;       // CRC-16/CCITT-FALSE по байтам [2 .. crc)
} telem_t;
_Static_assert(sizeof(telem_t) == 74, "telem_t must match RPi v4 layout");

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, без отражений и xorout.
// На RPi это binascii.crc_hqx(data, 0xFFFF).
static uint16_t crc16_ccitt(const uint8_t *b, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*b++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Строка команды "<текст>*XXXX": XXXX - CRC-16 текста до '*' в hex. true -
// сумма сошлась, и line обрезана до самого текста.
static bool cmd_crc_ok(char *line)
{
    char *star = strrchr(line, '*');
    if (!star || strlen(star + 1) != 4) return false;
    char *end;
    unsigned long want = strtoul(star + 1, &end, 16);
    if (*end != '\0') return false;
    if (crc16_ccitt((const uint8_t *)line, (size_t)(star - line)) != want) return false;
    *star = '\0';
    return true;
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
                portENTER_CRITICAL(&g_color_lock);
                g_cr = r; g_cg = g; g_cb = b; g_cc = c;
                g_color = apds9960_detect_color(r, g, b, c);
                g_color_ok_us = esp_timer_get_time();
                portEXIT_CRITICAL(&g_color_lock);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / COLOR_POLL_HZ));
    }
}

static void telem_task(void *arg)
{
    telem_t pkt = {.magic = {0xAA, 0x55}, .ver = TELEM_VER, .boot_id = g_boot_id};
    TickType_t last = xTaskGetTickCount();
    esp_task_wdt_add(NULL);   // см. robot_link_run() про TWDT
    uint32_t gyro_n = gyro_sample_count();
    int64_t  gyro_t = 0;      // когда счётчик сэмплов последний раз рос

    for (;;) {
        esp_task_wdt_reset();
        // z_cnt - ДО одометрии: если он уже новый, обнуление уже сделано, и
        // расстояния ниже - после "Z" (cmd_task растит его после обнуления).
        pkt.z_cnt = g_z_cnt;
        int64_t now = esp_timer_get_time();
        uint32_t n = gyro_sample_count();
        if (n != gyro_n) {
            gyro_n = n;
            gyro_t = now;
        }
        bool gyro_fresh  = gyro_t && now - gyro_t < GYRO_FRESH_US;
        portENTER_CRITICAL(&g_color_lock);
        bool color_fresh = g_color_ok_us && now - g_color_ok_us < COLOR_FRESH_US;
        pkt.color = g_color; pkt.cr = g_cr; pkt.cg = g_cg; pkt.cb = g_cb; pkt.cc = g_cc;
        portEXIT_CRITICAL(&g_color_lock);
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
        pkt.left_cm  = isnan(lmm) ? 0.0f : lmm * 0.1f;
        pkt.right_cm = isnan(rmm) ? 0.0f : rmm * 0.1f;
        pkt.status   = (uint8_t)g_run_state;
        motor_lock();
        uint8_t hits = bumper_now() | g_bumper_hits;
        motor_unlock();
        pkt.flags    = hits
                     | (isnan(lmm) ? 0 : 4)
                     | (isnan(rmm) ? 0 : 8)
                     | (button_pressed(BTN_1) ? 16 : 0)
                     | (gyro_fresh  ? 32 : 0)
                     | (color_fresh ? 64 : 0);
        pkt.cmd_err  = g_cmd_err;
        const uint8_t ids[] = {MOTOR_LEFT_1, MOTOR_LEFT_2, MOTOR_RIGHT_1, MOTOR_RIGHT_2};
        for (int i = 0; i < 4; ++i) {
            uint16_t dv; int8_t temp; uint8_t error;
            motor_get_health(ids[i], &dv, &temp, &error);
            pkt.motor_dv[i] = dv; pkt.motor_temp[i] = temp; pkt.motor_error[i] = error;
        }
        pkt.crc      = crc16_ccitt((const uint8_t *)&pkt + 2, sizeof(pkt) - 4);

        uart_write_bytes(LINK_PORT, (const char *)&pkt, sizeof(pkt));
        pkt.seq++;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / TELEM_HZ));
    }
}

static void cmd_task(void *arg)
{
    char    line[64];
    int     len = 0;
    bool discard = false;
    int64_t last_cmd_us  = esp_timer_get_time();
    int64_t hold_until_us = 0;   // keep re-asserting the 0 dps hold until then
    int64_t last_hold_us  = 0;
    bool    moving = false;
    esp_task_wdt_add(NULL);   // см. robot_link_run() про TWDT

    for (;;) {
        esp_task_wdt_reset();
        // Check before parsing. A continuous oversized/invalid UART stream
        // must not bypass the motion watchdog through parser continue paths.
        motor_lock();
        int64_t watchdog_now = esp_timer_get_time();
        if (g_run_state == RUN_PAUSED) {
            g_forward = moving = false;
            hold_until_us = 0;
        }
        if (hold_until_us) {
            if (watchdog_now >= hold_until_us) {
                robot_stop_all();
                hold_until_us = 0;
            } else if (watchdog_now - last_hold_us >= BRAKE_HOLD_PERIOD_US) {
                robot_brake();
                last_hold_us = watchdog_now;
            }
        }
        if (moving && watchdog_now - last_cmd_us > CMD_TIMEOUT_US) {
            g_forward = false;
            if (g_run_state == RUN_RUNNING) {
                robot_brake();
                hold_until_us = watchdog_now + BRAKE_HOLD_US;
                last_hold_us = watchdog_now;
            }
            moving = false;
            ESP_LOGW(TAG, "failsafe: motion command expired");
        }
        motor_unlock();
        uint8_t byte;
        if (uart_read_bytes(LINK_PORT, &byte, 1, pdMS_TO_TICKS(20)) > 0) {
            if (byte == '\n' || byte == '\r') {
                if (discard) { discard = false; len = 0; continue; }
                line[len] = '\0';
                float l = 0.0f, r = 0.0f, cap = 0.0f;
                int command_end = 0, short_end = 0;
                // Без CRC или с неверной - в счётчик, а дальше по цепочке как
                // пустая строка (её, "\r\n", ошибкой не считаем).
                if (len > 0 && !cmd_crc_ok(line)) {
                    g_cmd_err++;
                    line[0] = '\0';
                }
                // "K:" держит задачу ~1.3с на ходу серво - не под замком,
                // иначе BTN_1 всё это время не смог бы поставить паузу.
                bool motor_cmd = strncmp(line, "K:", 2) != 0;
                if (motor_cmd) motor_lock();

                if (sscanf(line, "M:%f,%f%n", &l, &r, &command_end) == 2 && line[command_end] == '\0'
                    && isfinite(l) && isfinite(r)) {
                    l = clamp_abs(l, LINK_MAX_CMS);
                    r = clamp_abs(r, LINK_MAX_CMS);
                    last_cmd_us = esp_timer_get_time();
                    hold_until_us = 0;              // driving again, drop the hold
                    if (l <= 0 && r <= 0 && (l < 0 || r < 0)) g_bumper_hits = 0;
                    g_forward = forward_translation(l, r);
                    if (g_forward && (bumper_now() || g_bumper_hits)) {
                        g_bumper_hits |= bumper_now();
                        robot_brake();
                        g_forward = moving = false;
                        l = r = 0;
                        hold_until_us = last_cmd_us + BRAKE_HOLD_US;
                        last_hold_us = last_cmd_us;
                        motor_unlock();
                        len = 0;
                        continue;
                    }
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
                           && ((sscanf(line, "T:%f,%f,%f%n", &l, &r, &cap, &command_end) == 3
                                && line[command_end] == '\0')
                               || (sscanf(line, "T:%f,%f%n", &l, &r, &short_end) == 2
                                   && line[short_end] == '\0'))
                           && isfinite(l) && isfinite(r) && isfinite(cap)) {
                    // Третье поле (потолок скорости) необязательно: старый
                    // nav.py его не шлёт, и тогда действует ASSIST_CAP_CMS.
                    // Ток ограничивает motor.c (MOTOR_TORQUE_CMD_MAX_A).
                    if (!(cap > 0.0f)) cap = ASSIST_CAP_CMS;
                    if (cap > LINK_MAX_CMS) cap = LINK_MAX_CMS;
                    last_cmd_us = esp_timer_get_time();
                    hold_until_us = 0;
                    if (l <= 0 && r <= 0 && (l < 0 || r < 0)) g_bumper_hits = 0;
                    g_forward = forward_translation(l, r);
                    if (g_forward && (bumper_now() || g_bumper_hits)) {
                        g_bumper_hits |= bumper_now();
                        robot_brake();
                        g_forward = moving = false;
                        hold_until_us = last_cmd_us + BRAKE_HOLD_US;
                        last_hold_us = last_cmd_us;
                        motor_unlock();
                        len = 0;
                        continue;
                    }
                    if (g_run_state == RUN_RUNNING && (l != 0.0f || r != 0.0f)) {
                        robot_set_side_torque_amps(l, r, cap);
                        moving = true;
                    } else {
                        robot_stop_all();
                        moving = false;
                    }
                } else if (strcmp(line, "B") == 0) {
                    g_forward = false;
                    if (!bumper_now()) g_bumper_hits = 0;
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
                } else if (strcmp(line, "Z") == 0) {
                    motor_zero_encoders();
                    g_z_cnt++;     // строго после обнуления - см. telem_task
                } else if (strcmp(line, "X") == 0) {
                    // Моторы выключить между заездами (nav.py при выходе).
                    g_forward = moving = false;
                    hold_until_us = 0;
                    robot_shutdown_all();
                    ESP_LOGI(TAG, "RPi: motors off");
                } else if (strcmp(line, "W") == 0) {
                    // Включить перед заездом; ускорение - заново, его держит RAM.
                    robot_wake_all();
                    robot_set_accel(MOTOR_ACCEL_DPS_S);
                    ESP_LOGI(TAG, "RPi: motors on");
                } else if (strcmp(line, "R") == 0) {
                    // Снять паузу BTN_1 - см. btn_task().
                    if (g_run_state == RUN_PAUSED) {
                        g_run_state = RUN_RUNNING;
                        ESP_LOGI(TAG, "RPi: RESUME");
                    }
                } else if (strncmp(line, "K:", 2) == 0) {
                    // Queue dispensing without blocking parsing or the watchdog.
                    // Host keeps normal 1.4 s spacing between kit commands.
                    if (g_run_state == RUN_RUNNING && !moving &&
                            (strcmp(line, "K:L") == 0 || strcmp(line, "K:R") == 0)) {
                        char side = line[2];
                        if (xQueueSend(g_servo_queue, &side, 0) != pdTRUE) g_cmd_err++;
                    } else g_cmd_err++;
                } else if (line[0] != '\0') {
                    g_cmd_err++;   // CRC сошлась, но команда не разобралась (или NaN)
                }
                if (motor_cmd) motor_unlock();
                len = 0;
            } else if (discard) {
                continue;
            } else if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)byte;
            } else {
                len = 0;   // overlong line, resync on the next newline
                discard = true;
                g_cmd_err++;
            }
        }

    }
}

// BTN_1 - перезапуск с чекпоинта (lack of progress): только СТАВИТ паузу,
// моторы сразу тормозят и замолкают. Снимает её не кнопка, а RPi командой
// "R" - после того как робота поставили на чекпоинт и нажали GPIO25 на
// малине (nav.py, handle_lift). Раньше BTN_1 переключал пауза/ход, и второе
// нажатие запускало бы робота, пока RPi ещё ждёт свою кнопку.
static void bumper_task(void *arg)
{
    int64_t last_brake = 0;
    bool was_hit = false;
    for (;;) {
        motor_lock();
        uint8_t hits = bumper_now();
        bool hit = g_run_state == RUN_RUNNING && g_forward && (hits || g_bumper_hits);
        int64_t now = esp_timer_get_time();
        if (hit && (!was_hit || now - last_brake >= BRAKE_HOLD_PERIOD_US)) {
            g_bumper_hits |= hits;
            robot_brake();
            last_brake = now;
            // Keep braking while the previous request was forward, including
            // during a blocking servo move. A reverse request releases this.
        }
        was_hit = hit;
        motor_unlock();
        vTaskDelay(1);
    }
}

static void servo_task(void *arg)
{
    char side;
    for (;;) {
        if (xQueueReceive(g_servo_queue, &side, portMAX_DELAY) == pdTRUE) {
            motor_lock();
            bool allowed = g_run_state == RUN_RUNNING;
            motor_unlock();
            if (!allowed) continue;
            if (side == 'L') servo_dispense_left();
            else servo_dispense_right();
        }
    }
}

static void btn_task(void *arg)
{
    for (;;) {
        if (button_clicked(BTN_1)) {
            motor_lock();
            bool pause = g_run_state == RUN_RUNNING;
            if (pause) {
                g_run_state = RUN_PAUSED;
                g_forward = false;
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

        printf("[%s] yaw=%6.1f pitch=%5.1f  %-6s  B1=%d C1=%d C2=%d  L=%s%.1f R=%s%.1f cm  cmd_err=%u\n",
               STATE[g_run_state == RUN_PAUSED ? 1 : 0], yaw, pitch,
               COLOR[g_color < 6 ? g_color : 0],
               button_pressed(BTN_1),
               button_pressed(CLICKER_1), button_pressed(CLICKER_2),
               isnan(l) ? "off/" : "", isnan(l) ? 0.0f : l * 0.1f,
               isnan(r) ? "off/" : "", isnan(r) ? 0.0f : r * 0.1f,
               (unsigned)g_cmd_err);

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
    g_servo_queue = xQueueCreate(1, sizeof(char));
    configASSERT(g_servo_queue);
    g_boot_id = (uint16_t)esp_random();

    robot_wake_all();
    robot_set_accel(MOTOR_ACCEL_DPS_S);
    motor_zero_encoders();
    robot_shutdown_all();   // 0x80, not 0x81: motors whine until "W" from the RPi
    ESP_LOGI(TAG, "motors off until RPi sends W");

    // telem, cmd (здесь) и enc_poll (motor.c) подписаны на task watchdog:
    // failsafe живёт в самой cmd_task, и её зависания иначе не заметил бы
    // никто. CONFIG_ESP_TASK_WDT_PANIC выключен - через 5с без сброса будет
    // только "Task watchdog got triggered" с именем задачи в консоли, без
    // перезагрузки. Самый долгий законный простой - "K:" (~1.3с).
    xTaskCreate(telem_task, "telem", 3072, NULL, 6,  NULL);
    xTaskCreate(cmd_task,   "cmd",   3072, NULL, 11, NULL);
    xTaskCreate(color_task, "color", 3072, NULL, 5,  NULL);
    xTaskCreate(btn_task,   "btn",   2560, NULL, 7,  NULL);
    xTaskCreate(bumper_task,"bumper",2560, NULL, 12, NULL);
    xTaskCreate(servo_task, "dispense",3072, NULL, 4, NULL);
    xTaskCreate(debug_task, "debug", 3072, NULL, 2,  NULL);

    ESP_LOGI(TAG, "link up: TX=%d RX=%d @ %d, accel=%d dps/s, telem v%d %d bytes, boot_id=%04X. "
             "Armed - BTN_1 pauses, RPi 'R' resumes.",
             LINK_TX, LINK_RX, LINK_BAUD, MOTOR_ACCEL_DPS_S,
             TELEM_VER, (int)sizeof(telem_t), g_boot_id);

    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
