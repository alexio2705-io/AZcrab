// ============================================================================
//  Стендовый тест шин - см. drive_test.h.
// ============================================================================
#include "drive_test.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "buttons.h"
#include "gyro.h"
#include "motor.h"

static const char *TAG = "DRIVE_TEST";

// 1 = BTN_1 крутит робота на месте на TURN_TEST_DEG, 0 = едет DRIVE_TEST_CM.
#define TEST_TURN         1

#define DRIVE_TEST_CM     50.0f
#define TURN_TEST_DEG     360.0f   // по часовой

// Профиль - как в nav.py, чтобы шины ехали так же, как в лабиринте.
#define MOVE_MAX_CMS      55.0f    // nav.py MOVE_MAX_CMS
#define RAMP_UP_S         0.25f    // nav.py RAMP_UP_S
#define BRAKE_DECEL_CMS2  300.0f   // nav.py BRAKE_DECEL_CMS2
// Ниже этой скорости не опускаемся, пока цель не достигнута: иначе
// sqrt(2*a*rem) у самой цели даёт почти ноль, и робот подползает вечно.
#define MIN_CMS           5.0f
#define MOTOR_ACCEL_DPS_S 11000    // как в robot_link.c

#define CTRL_PERIOD_MS    10       // энкодеры опрашиваются на 100 Гц
#define DRIVE_TIMEOUT_US  5000000  // 50 см на 55 см/с - ~1.2с; 5с = что-то не так
#define BRAKE_HOLD_MS     600      // сколько держать 0 dps перед тишиной
#define BRAKE_PERIOD_MS   40

// Путь по обеим сторонам; если одна сторона оффлайн - по другой, обе - NAN.
static float dist_cm(float l_mm, float r_mm)
{
    if (!isnan(l_mm) && !isnan(r_mm)) return (l_mm + r_mm) * 0.05f;
    if (!isnan(l_mm)) return l_mm * 0.1f;
    if (!isnan(r_mm)) return r_mm * 0.1f;
    return NAN;
}

static float yaw_diff(float a, float b)
{
    return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

__attribute__((unused)) static void drive_straight(void)
{
    bool use_gyro = gyro_ready();
    float yaw0 = use_gyro ? gyro_yaw() : 0.0f;
    if (!use_gyro) ESP_LOGW(TAG, "гироскоп не готов - еду без удержания курса");

    motor_zero_encoders();
    // Ноль у оффлайн-мотора берётся с его первого ответа (см. motor.c) -
    // дать энкодерам один цикл опроса.
    vTaskDelay(pdMS_TO_TICKS(30));

    (void)button_clicked(BTN_1);   // сбросить само стартовое нажатие
    const char *why = "цель";
    int64_t t0 = esp_timer_get_time();
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        int64_t now = esp_timer_get_time();
        float s = dist_cm(robot_left_dist_mm(), robot_right_dist_mm());
        if (isnan(s))                       { why = "энкодеры не отвечают"; break; }
        if (button_clicked(BTN_1))          { why = "BTN_1 - аварийный стоп"; break; }
        if (now - t0 > DRIVE_TIMEOUT_US)    { why = "таймаут"; break; }

        float rem = DRIVE_TEST_CM - s;
        if (rem <= 0.0f) break;

        // Разгон smoothstep'ом по времени, как в nav.py, торможение - по
        // оставшемуся пути.
        float x = (float)(now - t0) * 1e-6f / RAMP_UP_S;
        if (x > 1.0f) x = 1.0f;
        float v = MOVE_MAX_CMS * x * x * (3.0f - 2.0f * x);
        float v_dec = sqrtf(2.0f * BRAKE_DECEL_CMS2 * rem);
        if (v > v_dec) v = v_dec;
        if (v < MIN_CMS) v = MIN_CMS;

        float omega = use_gyro ? heading_hold_omega(yaw0, gyro_yaw()) : 0.0f;
        robot_drive(v, omega);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }

    float t_drive = (float)(esp_timer_get_time() - t0) * 1e-6f;

    // Активный тормоз: 0 dps надо повторять, иначе мотор выходит из
    // замкнутого контура (см. motor.h), потом - тишина.
    for (int ms = 0; ms < BRAKE_HOLD_MS; ms += BRAKE_PERIOD_MS) {
        robot_brake();
        vTaskDelay(pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
    robot_stop_all();
    vTaskDelay(pdMS_TO_TICKS(50));   // последний ответ энкодеров после остановки

    float l_mm = robot_left_dist_mm(), r_mm = robot_right_dist_mm();
    float s = dist_cm(l_mm, r_mm);
    float l_deg = motor_left_enc_deg(), r_deg = motor_right_enc_deg();
    float deg = (!isnan(l_deg) && !isnan(r_deg)) ? (l_deg + r_deg) * 0.5f
              : (!isnan(l_deg) ? l_deg : r_deg);

    printf("\n=== DRIVE: стоп (%s) за %.2f с ===\n", why, t_drive);
    printf("  энкодеры: L=%s%.1f  R=%s%.1f  ср=%.1f см (цель %.0f, перебег %+.1f см)\n",
           isnan(l_mm) ? "off/" : "", isnan(l_mm) ? 0.0f : l_mm * 0.1f,
           isnan(r_mm) ? "off/" : "", isnan(r_mm) ? 0.0f : r_mm * 0.1f,
           s, DRIVE_TEST_CM, s - DRIVE_TEST_CM);
    if (use_gyro)
        printf("  курс: ушёл на %+.1f deg\n", yaw_diff(gyro_yaw(), yaw0));
    if (!isnan(deg) && deg > 1.0f)
        // D = путь / (обороты * pi) = путь_мм * 360 / (pi * градусы)
        printf("  колесо провернулось на %.0f deg. Диаметр колеса, мм = "
               "путь по рулетке в мм x %.5f\n", deg, 360.0f / ((float)M_PI * deg));
    printf("BTN_1 - ещё раз.\n\n");
}

// Поворот - упрощённая копия turn_to() из nav.py: тот же профиль угловой
// скорости (smoothstep-разгон до полки, торможение sqrt(2*a*rem)), прямая
// подача TURN_FF с подстройкой по проскальзыванию и P по скорости вращения.
// Без манёвров при застревании - на стенде они не нужны.
#define TURN_RATE_MAX_DPS  220.0f
#define TURN_ACCEL_DPS2    700.0f
#define TURN_DECEL_DPS2    550.0f
#define TURN_FF_INIT       0.22f   // см/с колеса на deg/s корпуса
#define TURN_FF_MIN        0.10f
#define TURN_FF_MAX        0.60f
#define TURN_FF_ADAPT      0.08f   // nav.py 0.04 на 200 Гц, здесь 100 Гц
#define TURN_RATE_KP       0.06f
#define TURN_LEAD_S        0.04f
#define TURN_MAX_CMS       60.0f
#define TURN_MIN_CMS       5.0f
#define TURN_TOL_DEG       1.5f
#define TURN_DONE_RATE_DPS 20.0f
#define TURN_SETTLE        3
#define TURN_TIMEOUT_US    6000000
#define TURN_LEAD_TICKS    4       // TURN_LEAD_S / CTRL_PERIOD_MS

__attribute__((unused)) static void turn_in_place(void)
{
    if (!gyro_ready()) {
        printf("\n=== TURN: гироскоп не готов - поворот без него невозможен ===\n\n");
        return;
    }

    motor_zero_encoders();
    vTaskDelay(pdMS_TO_TICKS(30));

    (void)button_clicked(BTN_1);
    const char *why = "цель";
    const float t_up = 1.5f * TURN_RATE_MAX_DPS / TURN_ACCEL_DPS2;
    float ff = TURN_FF_INIT;
    float hist[TURN_LEAD_TICKS + 1] = {0};   // команды колесу за TURN_LEAD_S
    int   hi = 0;
    float turned = 0.0f;                     // намотанный угол, без заворота
    float yaw_prev = gyro_yaw();
    float yaw0 = yaw_prev;
    int settled = 0;
    int64_t t0 = esp_timer_get_time();
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        int64_t now = esp_timer_get_time();
        float yaw = gyro_yaw();
        turned += yaw_diff(yaw, yaw_prev);
        yaw_prev = yaw;
        float rate = gyro_yaw_rate();
        float rem = TURN_TEST_DEG - turned;

        if (button_clicked(BTN_1))       { why = "BTN_1 - аварийный стоп"; break; }
        if (now - t0 > TURN_TIMEOUT_US)  { why = "таймаут, НЕ ДОВЕРНУЛ"; break; }
        if (fabsf(rem) <= TURN_TOL_DEG && fabsf(rate) <= TURN_DONE_RATE_DPS) {
            if (++settled >= TURN_SETTLE) break;
        } else {
            settled = 0;
        }

        // Остаток с поправкой на запаздывание: за TURN_LEAD_S робот
        // довернётся на rate * TURN_LEAD_S.
        float rem_eff = rem - rate * TURN_LEAD_S;
        float x = (float)(now - t0) * 1e-6f / t_up;
        if (x > 1.0f) x = 1.0f;
        float w_ref = TURN_RATE_MAX_DPS * x * x * (3.0f - 2.0f * x);
        float w_dec = sqrtf(2.0f * TURN_DECEL_DPS2 * fabsf(rem_eff));
        if (w_ref > w_dec) w_ref = w_dec;
        w_ref = copysignf(w_ref, rem_eff);

        float spd = w_ref * ff + (w_ref - rate) * TURN_RATE_KP;
        if (spd >  TURN_MAX_CMS) spd =  TURN_MAX_CMS;
        if (spd < -TURN_MAX_CMS) spd = -TURN_MAX_CMS;
        if (fabsf(rem) > TURN_TOL_DEG && fabsf(spd) < TURN_MIN_CMS
            && fabsf(rate) < TURN_DONE_RATE_DPS)
            spd = copysignf(TURN_MIN_CMS, rem);
        // Положительный spd - по часовой: левая вперёд, правая назад.
        robot_set_side_speed_cms(spd, -spd);

        // Подстройка подачи: команда TURN_LEAD_S назад против нынешней
        // скорости, только на уверенном вращении в ту же сторону.
        float spd_then = hist[hi];
        hist[hi] = spd;
        hi = (hi + 1) % (TURN_LEAD_TICKS + 1);
        if (fabsf(rate) > 60.0f && fabsf(spd_then) > TURN_MIN_CMS * 2.0f
            && (rate > 0.0f) == (spd_then > 0.0f)) {
            float k = fabsf(spd_then) / fabsf(rate);
            if (k < TURN_FF_MIN) k = TURN_FF_MIN;
            if (k > TURN_FF_MAX) k = TURN_FF_MAX;
            ff += (k - ff) * TURN_FF_ADAPT;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }

    float t_turn = (float)(esp_timer_get_time() - t0) * 1e-6f;
    for (int ms = 0; ms < BRAKE_HOLD_MS; ms += BRAKE_PERIOD_MS) {
        robot_brake();
        vTaskDelay(pdMS_TO_TICKS(BRAKE_PERIOD_MS));
    }
    robot_stop_all();
    vTaskDelay(pdMS_TO_TICKS(50));

    turned += yaw_diff(gyro_yaw(), yaw_prev);   // докрутку на торможении тоже
    float l_mm = robot_left_dist_mm(), r_mm = robot_right_dist_mm();

    printf("\n=== TURN: стоп (%s) за %.2f с ===\n", why, t_turn);
    printf("  гироскоп: повернул на %.1f deg (цель %.0f, ошибка %+.1f), "
           "курс %.1f -> %.1f\n",
           turned, TURN_TEST_DEG, turned - TURN_TEST_DEG, yaw0, gyro_yaw());
    printf("  TURN_FF=%.3f (в nav.py TURN_FF_INIT=%.2f)\n", ff, TURN_FF_INIT);
    printf("  энкодеры: L=%s%+.1f  R=%s%+.1f см\n",
           isnan(l_mm) ? "off/" : "", isnan(l_mm) ? 0.0f : l_mm * 0.1f,
           isnan(r_mm) ? "off/" : "", isnan(r_mm) ? 0.0f : r_mm * 0.1f);
    if (!isnan(l_mm) && !isnan(r_mm) && fabsf(turned) > 30.0f) {
        // Колесо проходит дугу (колея/2) * угол: на 360 deg это pi*колея.
        // Эффективная колея больше реальной ровно настолько, насколько
        // шины проскальзывают в повороте.
        float arc_cm = (fabsf(l_mm) + fabsf(r_mm)) * 0.05f;
        float track_cm = arc_cm * 360.0f / ((float)M_PI * fabsf(turned));
        printf("  эффективная колея %.1f см (больше реальной - проскальзывание)\n", track_cm);
    }
    printf("BTN_1 - ещё раз.\n\n");
}

void drive_test_run(void)
{
    robot_wake_all();
    robot_set_accel(MOTOR_ACCEL_DPS_S);
    robot_stop_all();

#if TEST_TURN
    ESP_LOGI(TAG, "тест шин: BTN_1 - поворот на %.0f deg по часовой (без RPi)", TURN_TEST_DEG);
#else
    ESP_LOGI(TAG, "тест шин: BTN_1 - проехать %.0f см вперёд (без RPi)", DRIVE_TEST_CM);
#endif
    for (;;) {
        if (button_clicked(BTN_1)) {
            // Убрать руку от кнопки и робота.
            vTaskDelay(pdMS_TO_TICKS(500));
#if TEST_TURN
            turn_in_place();
#else
            drive_straight();
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
