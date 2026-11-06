#include "motor.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG_CAN = "MOTOR_CAN";

#define CAN_TX_GPIO GPIO_NUM_8
#define CAN_RX_GPIO GPIO_NUM_18
#define MOTOR_BASE_ID 0x140

static volatile uint32_t g_tx_ok = 0, g_tx_fail = 0;

static void send_frame(uint32_t can_id, const uint8_t data[8])
{
    twai_message_t msg = {
        .identifier = can_id,
        .extd = 0,
        .rtr = 0,
        .data_length_code = 8,
    };
    memcpy(msg.data, data, 8);
    // Fire-and-forget: a robot sends thousands of these, and an occasional
    // one not making it onto the bus in time is normal - nothing here
    // reports or retries failures, the next control tick supersedes it.
    // (Still counted below so motor_can_log_status() can show it.)
    if (twai_transmit(&msg, pdMS_TO_TICKS(3)) == ESP_OK) g_tx_ok++;
    else                                                 g_tx_fail++;
}

// Восстановление после BUS_OFF. Драйвер сам из BUS_OFF не выходит: моторы и
// энкодеры молчали бы до ребута ESP. twai_initiate_recovery() не блокирует -
// она только запускает аппаратный выход (128 x 11 рецессивных бит, ~1.4мс на
// 1 Мбит/с), по его окончании драйвер стоит в STOPPED и шлёт BUS_RECOVERED,
// после чего нужен twai_start(). Отдельная задача, а не cmd_task: пока шина
// лежит, twai_transmit() возвращает ошибку сразу (состояние не RUNNING), так
// что командный цикл не ждёт ничего. Опрос состояния раз в секунду -
// страховка на случай, если алерт пропал (например, BUS_OFF ещё до старта
// задачи).
static uint32_t g_can_recoveries = 0;

static void can_recover_task(void *arg)
{
    for (;;) {
        uint32_t alerts = 0;
        twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));

        twai_status_info_t st;
        if (twai_get_status_info(&st) != ESP_OK) continue;

        if (st.state == TWAI_STATE_BUS_OFF) {
            if (twai_initiate_recovery() == ESP_OK)
                ESP_LOGW(TAG_CAN, "BUS_OFF (TEC=%lu) - recovery started",
                         (unsigned long)st.tx_error_counter);
        } else if (st.state == TWAI_STATE_STOPPED) {
            if (twai_start() == ESP_OK)
                ESP_LOGW(TAG_CAN, "bus recovered, TWAI restarted (#%lu)",
                         (unsigned long)++g_can_recoveries);
        }
    }
}

void motor_can_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    // По умолчанию обе очереди - 5 кадров. Мало: enc_poll_task кладёт 4 кадра
    // одной пачкой, команда скорости - ещё 4, и они приходят вперемешку. При
    // переполнении twai_transmit блокируется на свой таймаут (3мс) и в итоге
    // теряет кадр - то есть пропадает не "лишний" опрос, а ответ энкодера.
    // 32 кадра стоят ~400 байт RAM и снимают вопрос целиком.
    g_config.tx_queue_len = 32;
    g_config.rx_queue_len = 32;
    // Для can_recover_task. Без TWAI_ALERT_AND_LOG: печать из ISR не нужна.
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ei = twai_driver_install(&g_config, &t_config, &f_config);
    if (ei != ESP_OK) {
        ESP_LOGE(TAG_CAN, "twai_driver_install failed: %s (TX=%d RX=%d)",
                 esp_err_to_name(ei), CAN_TX_GPIO, CAN_RX_GPIO);
        return;
    }
    esp_err_t es = twai_start();
    if (es != ESP_OK) {
        ESP_LOGE(TAG_CAN, "twai_start failed: %s", esp_err_to_name(es));
        return;
    }
    ESP_LOGI(TAG_CAN, "TWAI up @ 1Mbps, TX=%d RX=%d", CAN_TX_GPIO, CAN_RX_GPIO);
    xTaskCreate(can_recover_task, "can_recov", 2560, NULL, 9, NULL);
}

// One-line bus health dump: state, error counters, and how many of our own
// sends actually made it onto the bus vs timed out (queue full / bus down).
// TEC/REC climbing and BUS_OFF almost always means "wrong bitrate" or
// "nothing else on the bus" (no other node to ACK the frame); tx_fail high
// with state RUNNING usually means the driver never started, or the wires
// are swapped/not connected.
void motor_can_log_status(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        ESP_LOGW(TAG_CAN, "twai_get_status_info failed - driver not installed?");
        return;
    }
    static const char *STATE[] = {"STOPPED", "RUNNING", "BUS_OFF", "RECOVERING"};
    ESP_LOGI(TAG_CAN,
             "state=%s TEC=%lu REC=%lu tx_ok=%lu tx_fail=%lu "
             "tx_failed=%lu rx_missed=%lu arb_lost=%lu bus_err=%lu",
             (st.state < 4) ? STATE[st.state] : "?",
             (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
             (unsigned long)g_tx_ok, (unsigned long)g_tx_fail,
             (unsigned long)st.tx_failed_count, (unsigned long)st.rx_missed_count,
             (unsigned long)st.arb_lost_count, (unsigned long)st.bus_error_count);
}

void motor_wake(uint8_t id)
{
    uint8_t data[8] = {0x88, 0, 0, 0, 0, 0, 0, 0};
    send_frame(MOTOR_BASE_ID + id, data);
}

void motor_stop(uint8_t id)
{
    uint8_t data[8] = {0x81, 0, 0, 0, 0, 0, 0, 0};
    send_frame(MOTOR_BASE_ID + id, data);
}

void motor_shutdown(uint8_t id)
{
    uint8_t data[8] = {0x80, 0, 0, 0, 0, 0, 0, 0};
    send_frame(MOTOR_BASE_ID + id, data);
}

// LK command 0x34: acceleration limit (deg/s^2) for closed-loop speed, to RAM.
void motor_set_accel(uint8_t id, int32_t dps_per_s)
{
    uint8_t data[8] = {
        0x34,
        0x00, 0x00, 0x00,
        (uint8_t)(dps_per_s & 0xFF),
        (uint8_t)((dps_per_s >> 8) & 0xFF),
        (uint8_t)((dps_per_s >> 16) & 0xFF),
        (uint8_t)((dps_per_s >> 24) & 0xFF),
    };
    send_frame(MOTOR_BASE_ID + id, data);
}

// Предохранитель на случай мусора сверху: LINK_MAX_CMS в robot_link.c
// (150 см/с) - это ~2000 dps, здесь с запасом над ним.
#define MOTOR_SPEED_CMD_MAX_DPS 2500.0f

void set_motor_speed(uint8_t id, float dps)
{
    if (!isfinite(dps)) dps = 0.0f;   // lroundf(NaN) не определён
    if (dps >  MOTOR_SPEED_CMD_MAX_DPS) dps =  MOTOR_SPEED_CMD_MAX_DPS;
    if (dps < -MOTOR_SPEED_CMD_MAX_DPS) dps = -MOTOR_SPEED_CMD_MAX_DPS;
    int32_t raw = (int32_t)lroundf(dps * 100.0f); // 0.01 dps/LSB
    uint8_t data[8] = {
        0xA2,
        0x00, // max torque cap: 0 = use motor's configured stall current
        0x00,
        0x00,
        (uint8_t)(raw & 0xFF),
        (uint8_t)((raw >> 8) & 0xFF),
        (uint8_t)((raw >> 16) & 0xFF),
        (uint8_t)((raw >> 24) & 0xFF),
    };
    send_frame(MOTOR_BASE_ID + id, data);
}

// MF4010 V2 / legacy V2.35: 0xA1 +/-2048 = +/-16.5 A IQ (MG differs).
// Retain roughly the previous physical ceiling, now in correct MF units.
// IQ is not the battery current. Recalibrate on the actual motor firmware.
#define MOTOR_TORQUE_CMD_MAX_A 1.5f

void set_motor_torque_amps(uint8_t id, float amps)
{
    if (!isfinite(amps)) amps = 0.0f;   // NaN проходит мимо сравнений ниже
    if (amps >  MOTOR_TORQUE_CMD_MAX_A) amps =  MOTOR_TORQUE_CMD_MAX_A;
    if (amps < -MOTOR_TORQUE_CMD_MAX_A) amps = -MOTOR_TORQUE_CMD_MAX_A;
    int16_t raw = (int16_t)lroundf(amps * (2048.0f / 16.5f));
    uint8_t data[8] = {
        0xA1,
        0x00, 0x00, 0x00,
        (uint8_t)(raw & 0xFF),
        (uint8_t)(((uint16_t)raw >> 8) & 0xFF),
        0x00, 0x00,
    };
    send_frame(MOTOR_BASE_ID + id, data);
}

#define WHEEL_DIAM_MM 86.0f   // measured
#define WHEEL_DIAM_CM (WHEEL_DIAM_MM * 0.1f)
#define TRACK_CM      12.5f   // contact track; skid-turn effective track needs calibration
#define WHEEL_CIRC_CM (WHEEL_DIAM_CM * (float)M_PI)

// The right pair is mirror-mounted, so its own positive direction is the
// robot's backwards. Applied to the right side's speeds and to its encoder
// read-back alike, which is what keeps "driving forward" counting UP on both
// sides (before this, the right odometry ran negative while moving forward).
#define MOTOR_RIGHT_SIGN (-1)

// The motors ignore a speed command while they are in the 0x81 stopped state,
// so track it and wake them on the first command after a stop.
static bool g_motors_stopped = true;

void robot_set_side_speed_dps(float left_dps, float right_dps)
{
    if (g_motors_stopped) robot_wake_all();

    set_motor_speed(MOTOR_LEFT_1,  left_dps);
    set_motor_speed(MOTOR_LEFT_2,  left_dps);
    set_motor_speed(MOTOR_RIGHT_1,  right_dps * MOTOR_RIGHT_SIGN);
    set_motor_speed(MOTOR_RIGHT_2, -right_dps * MOTOR_RIGHT_SIGN);  // id3 flipped again
}

void robot_set_side_speed_cms(float left_cms, float right_cms)
{
    robot_set_side_speed_dps((left_cms  / WHEEL_CIRC_CM) * 360.0f,
                              (right_cms / WHEEL_CIRC_CM) * 360.0f);
}

// Режим момента (0xA1) - это ток, а не скорость: мотор разгоняет колесо, пока
// что-то ему сопротивляется, и ничем не ограничен, когда сопротивление
// пропадает. 2.5А на четырёх колёсах - это ускорение в несколько g для
// лёгкого робота, так что стоит переднему краю сойти с бордюра или колесу
// повиснуть в воздухе, как за пару десятков мс колёса раскручиваются до
// огромной скорости - это и был "слёт с препятствия". Ограничитель сидит здесь,
// а не на RPi: у RPi запаздывание UART+Python ~30мс, за которые робот уже
// разогнан; здесь - только период опроса энкодера (10мс).
//
// Каждый мотор отдельно: ниже ASSIST_TAPER_FRAC от потолка - полный ток,
// дальше ток линейно спадает до нуля у потолка, а выше потолка колесо уходит
// в обычный режим скорости (0xA2) на потолке - с его ограничением ускорения и
// настроенным в LK Motor Tool током. Повисшее колесо тем самым не
// раскручивается, а робот, перевалив препятствие, не улетает.
#define ASSIST_TAPER_FRAC 0.4f

static void assist_one(uint8_t id, float amps, float cap_dps)
{
    float dir = (amps >= 0.0f) ? 1.0f : -1.0f;
    float v = motor_get_speed_dps(id);
    if (isnan(v)) {
        // Энкодер молчит - скорость не проверить, момент без контроля
        // не даём: едем по скорости на потолке.
        set_motor_speed(id, dir * cap_dps);
        return;
    }
    float vd = v * dir;   // скорость в сторону, куда тянет момент
    if (vd >= cap_dps) {
        set_motor_speed(id, dir * cap_dps);
        return;
    }
    float k = (cap_dps - vd) / (cap_dps * ASSIST_TAPER_FRAC);
    if (k > 1.0f) k = 1.0f;
    set_motor_torque_amps(id, amps * k);
}

void robot_set_side_torque_amps(float left_amps, float right_amps, float cap_cms)
{
    if (g_motors_stopped) robot_wake_all();

    if (cap_cms < 5.0f) cap_cms = 5.0f;
    float cap_dps = (cap_cms / WHEEL_CIRC_CM) * 360.0f;
    assist_one(MOTOR_LEFT_1,  left_amps,                     cap_dps);
    assist_one(MOTOR_LEFT_2,  left_amps,                     cap_dps);
    assist_one(MOTOR_RIGHT_1, right_amps * MOTOR_RIGHT_SIGN, cap_dps);
    assist_one(MOTOR_RIGHT_2, -right_amps * MOTOR_RIGHT_SIGN, cap_dps);
}

void robot_drive(float speed_cms, float omega_dps)
{
    // In-place turn contribution: each side needs to cover an arc of
    // radius TRACK_CM/2 to rotate the body by omega_dps. Clockwise
    // (positive omega) speeds up the left side and slows the right, same
    // convention as heading_hold_omega()'s output.
    float omega_rad_s = omega_dps * ((float)M_PI / 180.0f);
    float turn_cms = omega_rad_s * (TRACK_CM * 0.5f);
    robot_set_side_speed_cms(speed_cms + turn_cms, speed_cms - turn_cms);
}

void robot_stop_all(void)
{
    motor_stop(MOTOR_LEFT_1); motor_stop(MOTOR_LEFT_2);
    motor_stop(MOTOR_RIGHT_1); motor_stop(MOTOR_RIGHT_2);
    g_motors_stopped = true;
}

void robot_shutdown_all(void)
{
    motor_shutdown(MOTOR_LEFT_1); motor_shutdown(MOTOR_LEFT_2);
    motor_shutdown(MOTOR_RIGHT_1); motor_shutdown(MOTOR_RIGHT_2);
    g_motors_stopped = true;
}

void robot_wake_all(void)
{
    motor_wake(MOTOR_LEFT_1); motor_wake(MOTOR_LEFT_2);
    motor_wake(MOTOR_RIGHT_1); motor_wake(MOTOR_RIGHT_2);
    g_motors_stopped = false;
}

void robot_brake(void)
{
    robot_set_side_speed_dps(0.0f, 0.0f);
}

void robot_set_accel(int32_t dps_per_s)
{
    motor_set_accel(MOTOR_LEFT_1,  dps_per_s);
    motor_set_accel(MOTOR_LEFT_2,  dps_per_s);
    motor_set_accel(MOTOR_RIGHT_1, dps_per_s);
    motor_set_accel(MOTOR_RIGHT_2, dps_per_s);
}

#define HEADING_KP        6.0f   // dps of correction per degree of error
#define HEADING_MAX_OMEGA 250.0f // clamp so a big error doesn't spin it wildly

float heading_hold_omega(float target_yaw_deg, float yaw_deg)
{
    // Shortest signed error, wrapped into (-180, 180] so it never fights
    // itself across the 0/360 boundary.
    float error = fmodf(target_yaw_deg - yaw_deg + 540.0f, 360.0f) - 180.0f;
    float omega = HEADING_KP * error;
    if (omega > HEADING_MAX_OMEGA) omega = HEADING_MAX_OMEGA;
    if (omega < -HEADING_MAX_OMEGA) omega = -HEADING_MAX_OMEGA;
    return omega;
}

// ============================================================================
//  Encoder read-back
// ============================================================================
#define MF_CMD_GET_POS    0x92   // multi-loop (absolute, accumulated) angle
#define MF_MAX_ID         32
#define ENC_ONLINE_US     400000 // 400ms - a few missed polls and it's "offline"
// Full 4-id poll cycles/sec (= 4*this frames/s out, as many back).
//
// ОТКАЧЕНО со 200 обратно на 100. Расчёт по занятости шины сходился (200 Гц -
// это ~20% от 1 Мбит/с), но на роботе энкодеры после этого переставали
// отвечать: телеметрия приходила с flags без битов 2/3, и nav.py валился в
// "энкодер не отвечает" ещё до первой команды моторам. Занятость шины была
// не тем, что стоило считать:
//
//   - 0x92 каждому мотору раз в 5мс - это 200 запросов/с НА КОНТРОЛЛЕР, и
//     LK/MG на такой темп отвечать не обязан; пропущенный ответ здесь стоит
//     дороже, чем сэкономленные 5мс свежести;
//   - очередь TX у TWAI по умолчанию всего 5 кадров (см. motor_can_init),
//     а enc_poll_task кладёт 4 кадра пачкой; вместе с командами скорости
//     очередь переполняется, twai_transmit висит до 3мс на каждом кадре,
//     и цикл опроса перестаёт укладываться в свой же период.
//
// Если возвращаться к 200 - сначала посмотреть motor_can_log_status()
// (tx_fail, rx_missed, arb_lost его теперь печатает debug_task), и поднимать
// частоту только после того, как очереди станут глубже.
#define ENC_POLL_HZ       100

typedef struct {
    bool    seen;
    float   angle_deg;   // raw, not zero-adjusted
    int64_t last_rx_us;
    float   speed_dps;   // из разности соседних ответов 0x92, в знаке самого мотора
} enc_state_t;

static enc_state_t   g_enc[MF_MAX_ID + 1];
static portMUX_TYPE  g_enc_mux = portMUX_INITIALIZER_UNLOCKED;
static float         g_enc_zero[MF_MAX_ID + 1];
// Мотор был оффлайн в момент "Z": старый ноль ему оставлять нельзя (после
// возврата - скачок одометрии), поэтому ноль берётся с его первого ответа,
// а до тех пор он в одометрии не участвует. Всё под g_enc_mux.
static bool          g_enc_zero_pending[MF_MAX_ID + 1];
static const uint8_t ENC_IDS[] = {ENC_LEFT_ID_1, ENC_LEFT_ID_2, ENC_RIGHT_ID_1, ENC_RIGHT_ID_2};
typedef struct { uint16_t dv; int8_t temp; uint8_t error; int64_t t; } health_t;
static health_t g_health[MF_MAX_ID + 1];

void motor_get_health(uint8_t id, uint16_t *dv, int8_t *temp, uint8_t *error)
{
    *dv = 0; *temp = -128; *error = 0;
    if (id > MF_MAX_ID) return;
    portENTER_CRITICAL(&g_enc_mux);
    health_t h = g_health[id];
    portEXIT_CRITICAL(&g_enc_mux);
    if (h.t && esp_timer_get_time() - h.t < 2000000) {
        *dv = h.dv; *temp = h.temp; *error = h.error;
    }
}

static bool enc_online(uint8_t id)
{
    if (id > MF_MAX_ID || !g_enc[id].seen) return false;
    return (esp_timer_get_time() - g_enc[id].last_rx_us) < ENC_ONLINE_US;
}

static void enc_rx_task(void *arg)
{
    twai_message_t rx;
    for (;;) {
        if (twai_receive(&rx, portMAX_DELAY) != ESP_OK) continue;
        if (rx.extd || rx.rtr || rx.data_length_code != 8) continue;
        if (rx.identifier <= MOTOR_BASE_ID || rx.identifier > MOTOR_BASE_ID + MF_MAX_ID)
            continue;
        uint8_t id = rx.identifier - MOTOR_BASE_ID;
        const uint8_t *d = rx.data;
        if (d[0] == 0x9A) {
            // LK MF status1: d[1] temperature, d[2..3] voltage 0.01 V/LSB.
            // d[3..4] at 0.1 V (old guess) read 0.6 V on our motors.
            uint16_t centivolts = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
            portENTER_CRITICAL(&g_enc_mux);
            g_health[id] = (health_t){.dv = (uint16_t)((centivolts + 5) / 10),
                .temp = (int8_t)d[1], .error = d[7], .t = esp_timer_get_time()};
            portEXIT_CRITICAL(&g_enc_mux);
            continue;
        }
        if (d[0] != MF_CMD_GET_POS) continue; // only care about position replies here

        int64_t raw = 0;
        for (int i = 0; i < 7; i++) raw |= ((int64_t)d[i + 1] << (8 * i));
        if (raw & (1LL << 55)) raw |= (0xFFLL << 56); // sign-extend 56 -> 64 bit

        float   angle = (float)raw * 0.01f; // 0.01 deg/LSB
        int64_t now   = esp_timer_get_time();

        portENTER_CRITICAL(&g_enc_mux);
        // Скорость - по тем же ответам, что и путь: единицы у них заведомо
        // одни (градусы колеса), в отличие от поля скорости в ответах на
        // команды, где у MG с редуктором их трактовка не проверена.
        // Лёгкое сглаживание гасит дрожание меток времени (~1мс на 10мс).
        if (g_enc[id].seen) {
            float dt = (float)(now - g_enc[id].last_rx_us) * 1e-6f;
            if (dt > 0.003f && dt < 0.1f) {
                float v = (angle - g_enc[id].angle_deg) / dt;
                g_enc[id].speed_dps += 0.5f * (v - g_enc[id].speed_dps);
            } else if (dt >= 0.1f) {
                g_enc[id].speed_dps = 0.0f;   // был перерыв - старой скорости не верим
            }
        }
        g_enc[id].seen       = true;
        g_enc[id].angle_deg  = angle;
        g_enc[id].last_rx_us = now;
        if (g_enc_zero_pending[id]) {
            g_enc_zero[id] = angle;
            g_enc_zero_pending[id] = false;
        }
        portEXIT_CRITICAL(&g_enc_mux);
    }
}

// All four ids every cycle, not one id per cycle: the RPi's decel profile
// closes on this odometry, and a per-motor update rate of a few Hz would let
// the robot travel centimetres between samples.
static void enc_poll_task(void *arg)
{
    const int n = sizeof(ENC_IDS) / sizeof(ENC_IDS[0]);
    const TickType_t period = pdMS_TO_TICKS(1000 / ENC_POLL_HZ);
    TickType_t last = xTaskGetTickCount();
    uint8_t data[8] = {MF_CMD_GET_POS, 0, 0, 0, 0, 0, 0, 0};
    esp_task_wdt_add(NULL);   // см. robot_link_run() про TWDT
    unsigned health_tick = 0;
    for (;;) {
        esp_task_wdt_reset();
        for (int i = 0; i < n; i++) send_frame(MOTOR_BASE_ID + ENC_IDS[i], data);
        // One extra read each 100 ms, round-robin: 2.5 Hz per motor.
        if (++health_tick % 10 == 0) {
            uint8_t status[8] = {0x9A};
            send_frame(MOTOR_BASE_ID + ENC_IDS[(health_tick / 10 - 1) % n], status);
        }
        vTaskDelayUntil(&last, period);
    }
}

void motor_enc_start(void)
{
    xTaskCreate(enc_rx_task,   "enc_rx",   3072, NULL, 10, NULL);
    xTaskCreate(enc_poll_task, "enc_poll", 2048, NULL, 6,  NULL);
}

float motor_get_angle_deg(uint8_t id)
{
    if (id > MF_MAX_ID) return NAN;
    portENTER_CRITICAL(&g_enc_mux);
    bool online = enc_online(id);
    float a = g_enc[id].angle_deg;
    portEXIT_CRITICAL(&g_enc_mux);
    return online ? a : NAN;
}

float motor_get_speed_dps(uint8_t id)
{
    if (id > MF_MAX_ID) return NAN;
    portENTER_CRITICAL(&g_enc_mux);
    bool online = enc_online(id);
    float v = g_enc[id].speed_dps;
    portEXIT_CRITICAL(&g_enc_mux);
    return online ? v : NAN;
}

// Все четыре id - одним куском под g_enc_mux: иначе между ними мог вклиниться
// ответ энкодера, и стороны обнулились бы по разным моментам.
void motor_zero_encoders(void)
{
    const int n = sizeof(ENC_IDS) / sizeof(ENC_IDS[0]);
    portENTER_CRITICAL(&g_enc_mux);
    for (int i = 0; i < n; i++) {
        uint8_t id = ENC_IDS[i];
        if (enc_online(id)) {
            g_enc_zero[id] = g_enc[id].angle_deg;
            g_enc_zero_pending[id] = false;
        } else {
            g_enc_zero_pending[id] = true;
        }
    }
    portEXIT_CRITICAL(&g_enc_mux);
}

// avg() of whichever ids in {a,b} are online, with each optionally sign-flipped
// (flip_a/flip_b = -1 to invert); NAN only if both are offline. Угол и ноль
// читаются вместе под g_enc_mux - не вперемешку с motor_zero_encoders().
static float enc_pair_deg(uint8_t a, int8_t sign_a, uint8_t b, int8_t sign_b)
{
    portENTER_CRITICAL(&g_enc_mux);
    bool oa = enc_online(a) && !g_enc_zero_pending[a];
    bool ob = enc_online(b) && !g_enc_zero_pending[b];
    float va = (g_enc[a].angle_deg - g_enc_zero[a]) * sign_a;
    float vb = (g_enc[b].angle_deg - g_enc_zero[b]) * sign_b;
    portEXIT_CRITICAL(&g_enc_mux);
    if (oa && ob) return (va + vb) * 0.5f;
    if (oa) return va;
    if (ob) return vb;
    return NAN;
}

float motor_left_enc_deg(void)
{
    return enc_pair_deg(ENC_LEFT_ID_1, 1, ENC_LEFT_ID_2, 1);
}

float motor_right_enc_deg(void)
{
    // Same signs as the speed path in robot_set_side_speed_dps(), so forward
    // motion reads positive here just like it does on the left.
    return enc_pair_deg(ENC_RIGHT_ID_1, MOTOR_RIGHT_SIGN,
                        ENC_RIGHT_ID_2, -MOTOR_RIGHT_SIGN);
}

static float enc_deg_to_mm(float deg)
{
    return isnan(deg) ? NAN : (deg / 360.0f) * (float)M_PI * WHEEL_DIAM_MM;
}

float robot_left_dist_mm(void)  { return enc_deg_to_mm(motor_left_enc_deg()); }
float robot_right_dist_mm(void) { return enc_deg_to_mm(motor_right_enc_deg()); }

void motor_probe(uint8_t first_id, uint8_t last_id)
{
    ESP_LOGI(TAG_CAN, "=== Probing ids %u..%u (GET_POS, one at a time) ===",
             first_id, last_id);
    uint8_t data[8] = {MF_CMD_GET_POS, 0, 0, 0, 0, 0, 0, 0};

    for (uint8_t id = first_id; id <= last_id && id <= MF_MAX_ID; id++) {
        // Clear any stale reading so a real reply this round is what we see -
        // otherwise an id that answered once at boot would look "online"
        // forever even if it's actually silent now.
        portENTER_CRITICAL(&g_enc_mux);
        g_enc[id].seen = false;
        portEXIT_CRITICAL(&g_enc_mux);

        motor_wake(id);
        vTaskDelay(pdMS_TO_TICKS(5));
        for (int i = 0; i < 10; i++) {
            send_frame(MOTOR_BASE_ID + id, data);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        float a = motor_get_angle_deg(id);
        if (isnan(a)) ESP_LOGW(TAG_CAN, "  id %2u: NO REPLY", id);
        else          ESP_LOGI(TAG_CAN, "  id %2u: OK   angle=%.2f deg", id, a);
    }
    ESP_LOGI(TAG_CAN, "=== Probe done ===");
}
