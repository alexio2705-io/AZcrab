#include "servo.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_GPIO      10
#define SERVO_TIMER     LEDC_TIMER_0
#define SERVO_CHANNEL   LEDC_CHANNEL_0
#define SERVO_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_RES       LEDC_TIMER_14_BIT
#define SERVO_PERIOD_US 20000 // 50Hz, the standard hobby servo frame
#define SERVO_FRAME_MS  20    // one frame - the servo reads its input no more often

// The three positions, and the only numbers worth touching. Start conservative
// and widen: if the servo buzzes or stalls at an end it is grinding against a
// mechanical stop, so pull that value back toward centre.
#define SERVO_LEFT_US   700
#define SERVO_CENTER_US 1500
#define SERVO_RIGHT_US  2300

// Travel speed, in microseconds of pulse width per second. Left alone a servo
// slams to its target as fast as the gears allow; feeding it the target in
// small steps instead makes it crawl there. Lower = slower. At 1800 the full
// 800us swing from centre to an end takes a bit under half a second.
#define SERVO_SPEED_US_PER_S 1800

// How long the bay stays open, so the cube has time to actually fall out.
#define SERVO_HOLD_MS 400

static uint32_t s_current_us = SERVO_CENTER_US;

void servo_set_pulse_us(uint32_t pulse_us)
{
    // 14-bit over a 20ms frame is ~1.2us per step - finer than any servo can
    // resolve, and the multiply stays well inside 32 bits.
    uint32_t duty = (pulse_us * ((1u << 14) - 1)) / SERVO_PERIOD_US;
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
    s_current_us = pulse_us;
}

// Walk the setpoint to the target one frame at a time. The servo chases a
// target that is never far away, so it never gets to build up speed.
static void glide_to(uint32_t target_us)
{
    const uint32_t step = (SERVO_SPEED_US_PER_S * SERVO_FRAME_MS) / 1000;

    while (s_current_us != target_us) {
        uint32_t next;
        if (target_us > s_current_us) {
            next = (target_us - s_current_us < step) ? target_us : s_current_us + step;
        } else {
            next = (s_current_us - target_us < step) ? target_us : s_current_us - step;
        }
        servo_set_pulse_us(next);
        vTaskDelay(pdMS_TO_TICKS(SERVO_FRAME_MS));
    }
}

void servo_center(void)
{
    glide_to(SERVO_CENTER_US);
}

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = 1000000 / SERVO_PERIOD_US,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel);

    // Straight to centre, not glided: at boot we have no idea where the horn
    // is sitting, so there is nothing to ramp away from.
    servo_set_pulse_us(SERVO_CENTER_US);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// Swing out, wait for the cube to clear, come back. The servo keeps being
// driven at centre afterwards rather than going limp, so the closed bay holds
// against the next cube leaning on it.
static void dispense(uint32_t end_pulse_us)
{
    glide_to(end_pulse_us);
    vTaskDelay(pdMS_TO_TICKS(SERVO_HOLD_MS));
    glide_to(SERVO_CENTER_US);
}

void servo_dispense_left(void)
{
    dispense(SERVO_LEFT_US);
}

void servo_dispense_right(void)
{
    dispense(SERVO_RIGHT_US);
}
