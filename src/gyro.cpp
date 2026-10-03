#include "gyro.h"
#include "BNO08x.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Pin map - from the schematic, reading the ESP32-S3-WROOM-1 pin NAMES (the
// teal IOxx labels), not the red footprint pin numbers next to them. The two
// are easy to mix up: CS_GYRO sits on footprint pin 38, whose name is IO2.
//
// The BNO085's P0/P1 are hard-wired to +3.3V on this board, which is what
// puts the breakout in SPI mode - unlike the breadboard prototype, nothing
// here has to drive them.
// ---------------------------------------------------------------------------
#define GYRO_SCK_GPIO  GPIO_NUM_38 // -> BNO085 SCL
#define GYRO_MISO_GPIO GPIO_NUM_39 // -> BNO085 SDA (DO)
#define GYRO_INT_GPIO  GPIO_NUM_40 // -> BNO085 INT
#define GYRO_RST_GPIO  GPIO_NUM_41 // -> BNO085 RST
#define GYRO_MOSI_GPIO GPIO_NUM_42 // -> BNO085 DI
#define GYRO_CS_GPIO   GPIO_NUM_2  // -> BNO085 CS

#define GYRO_SPI_HOST SPI2_HOST
#define GYRO_SPI_HZ   3000000 // BNO08x maximum

// Fastest orientation output the BNO085 has: the gyro-integrated rotation
// vector runs at 1 kHz, against the 400 Hz ceiling of every other rotation
// vector report. It is the fusion result extrapolated by raw gyro integration
// between fusion updates - same low-drift heading, just sampled 10x finer and
// with far less latency.
//
// Like the game rotation vector it uses no magnetometer, which is what we
// want next to four brushless motors and a CAN harness; the heading is
// relative anyway, measured from the last gyro_zero_yaw().
#define GYRO_REPORT_PERIOD_US 1000 // 1 kHz - the sensor's hard ceiling

// Second report: linear acceleration (gravity removed). It exists only to
// notice the robot being lifted off the field, which is a ~0.5 s event, so
// 100 Hz is already an order of magnitude more than the job needs - and
// keeping it slow leaves the SH-2 stack's budget to the 1 kHz rotation
// vector the control loop actually rides on.
#define GYRO_ACCEL_PERIOD_US 10000 // 100 Hz

static BNO08x *s_imu;

// Given once per orientation sample. Binary, not counting, on purpose: a
// consumer that falls behind (the console does, by a lot) should wake up on
// the newest sample and skip the backlog, never replay it.
static SemaphoreHandle_t s_sample_sem;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_yaw_raw, s_yaw_zero, s_pitch, s_roll;
static float s_yaw_rate;   // deg/s, clockwise-positive, same sense as the yaw
// Linear acceleration, m/s^2, sensor frame. Stays 0,0,0 if the report could
// not be enabled - see gyro.h on why that degrades rather than breaks.
static float s_ax, s_ay, s_az;
static uint32_t s_samples;
static bool s_ready;

#define RAD_TO_DEG 57.2957795f

static float wrap360(float deg)
{
    deg = fmodf(deg, 360.0f);
    return deg < 0.0f ? deg + 360.0f : deg;
}

static void gyro_task(void *arg)
{
    for (;;) {
        // Blocks until SOME report lands (or a ~3s timeout), so this never
        // spins. Two reports are enabled now, and data_available() does not
        // say which one arrived - so both have to be tested every wake-up.
        // The old single `&&` would have dropped into a vTaskDelay() fallback
        // on every accelerometer report, stalling the 1 kHz heading stream.
        if (!s_imu->data_available())
            continue;

        if (s_imu->rpt.rv_gyro_integrated.has_new_data()) {
            bno08x_euler_angle_t e = s_imu->rpt.rv_gyro_integrated.get_euler(true); // degrees
            // Same report carries the angular velocity it integrated, so this
            // costs nothing extra - no second report, no extra SPI traffic.
            bno08x_ang_vel_t v = s_imu->rpt.rv_gyro_integrated.get_vel();           // rad/s

            portENTER_CRITICAL(&s_lock);
            // e.z is counter-clockwise positive (right-handed about +Z); the
            // robot's convention is clockwise, hence the sign flip.
            s_yaw_raw  = wrap360(-e.z);
            s_pitch    = e.y;
            s_roll     = e.x;
            s_yaw_rate = -v.z * RAD_TO_DEG;
            s_ready    = true;
            s_samples++;
            portEXIT_CRITICAL(&s_lock);

            xSemaphoreGive(s_sample_sem);
        }

        if (s_imu->rpt.linear_accelerometer.has_new_data()) {
            bno08x_accel_t a = s_imu->rpt.linear_accelerometer.get();
            portENTER_CRITICAL(&s_lock);
            s_ax = a.x;
            s_ay = a.y;
            s_az = a.z;
            portEXIT_CRITICAL(&s_lock);
        }
    }
}

void gyro_init(void)
{
    s_sample_sem = xSemaphoreCreateBinary();

    // GPIO39..42 are the JTAG pins on the ESP32-S3 and can come up owned by
    // the JTAG controller - hand them back to the GPIO matrix first.
    gpio_reset_pin(GYRO_MISO_GPIO);
    gpio_reset_pin(GYRO_INT_GPIO);
    gpio_reset_pin(GYRO_RST_GPIO);
    gpio_reset_pin(GYRO_MOSI_GPIO);

    bno08x_config_t cfg(GYRO_SPI_HOST, GYRO_MOSI_GPIO, GYRO_MISO_GPIO, GYRO_SCK_GPIO,
                        GYRO_CS_GPIO, GYRO_INT_GPIO, GYRO_RST_GPIO, GYRO_SPI_HZ);

    s_imu = new BNO08x(cfg);
    if (!s_imu->initialize()) {
        printf("gyro: BNO085 init failed - check CS/INT/RST wiring\n");
        return;
    }
    if (!s_imu->rpt.rv_gyro_integrated.enable(GYRO_REPORT_PERIOD_US)) {
        printf("gyro: could not enable gyro-integrated rotation vector\n");
        return;
    }

    // Deliberately NOT fatal: without it the robot simply loses one of the
    // three "somebody picked me up" votes on the RPi side (roll and the
    // colour sensor going dark still carry the decision), and losing lift
    // detection is no reason to refuse to drive.
    if (!s_imu->rpt.linear_accelerometer.enable(GYRO_ACCEL_PERIOD_US))
        printf("gyro: no linear accelerometer - lift detection runs on 2 signals\n");

    // Above the SH-2 data-processing task (6) so a 1 kHz report stream gets
    // drained the moment it is decoded, not a scheduling slot later.
    xTaskCreate(gyro_task, "gyro", 4096, NULL, 7, NULL);
}

bool gyro_wait_sample(uint32_t timeout_ms)
{
    return xSemaphoreTake(s_sample_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

uint32_t gyro_sample_count(void)
{
    portENTER_CRITICAL(&s_lock);
    uint32_t n = s_samples;
    portEXIT_CRITICAL(&s_lock);
    return n;
}

bool gyro_ready(void)
{
    portENTER_CRITICAL(&s_lock);
    bool ready = s_ready;
    portEXIT_CRITICAL(&s_lock);
    return ready;
}

void gyro_get_angles(float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    portENTER_CRITICAL(&s_lock);
    float yaw = wrap360(s_yaw_raw - s_yaw_zero);
    float pitch = s_pitch;
    float roll = s_roll;
    portEXIT_CRITICAL(&s_lock);

    if (yaw_deg) *yaw_deg = yaw;
    if (pitch_deg) *pitch_deg = pitch;
    if (roll_deg) *roll_deg = roll;
}

float gyro_yaw(void)
{
    float yaw;
    gyro_get_angles(&yaw, NULL, NULL);
    return yaw;
}

float gyro_yaw_rate(void)
{
    portENTER_CRITICAL(&s_lock);
    float rate = s_yaw_rate;
    portEXIT_CRITICAL(&s_lock);
    return rate;
}

float gyro_pitch(void)
{
    float pitch;
    gyro_get_angles(NULL, &pitch, NULL);
    return pitch;
}

float gyro_roll(void)
{
    float roll;
    gyro_get_angles(NULL, NULL, &roll);
    return roll;
}

void gyro_get_lin_accel(float *x, float *y, float *z)
{
    portENTER_CRITICAL(&s_lock);
    float ax = s_ax, ay = s_ay, az = s_az;
    portEXIT_CRITICAL(&s_lock);

    if (x) *x = ax;
    if (y) *y = ay;
    if (z) *z = az;
}

float gyro_lin_accel_mag(void)
{
    float x, y, z;
    gyro_get_lin_accel(&x, &y, &z);
    return sqrtf(x * x + y * y + z * z);
}

void gyro_zero_yaw(void)
{
    portENTER_CRITICAL(&s_lock);
    s_yaw_zero = s_yaw_raw;
    portEXIT_CRITICAL(&s_lock);
}
