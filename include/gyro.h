#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Adafruit BNO085 (BNO08x) 9-DoF IMU, SPI mode.
//
// Wiring per the schematic (D15 "bno085"): P0 and P1 are both hard-wired to
// +3.3V, which selects SPI on the breakout, so SCL=SCK, SDA=MISO(DO), DI=MOSI.
// GPIOs are the ESP32-S3-WROOM-1 pin NAMES (teal IOxx), not the footprint pin
// numbers printed beside them:
//   SCK_GYRO  -> GPIO38     MOSI_GYRO -> GPIO42
//   MISO_GYRO -> GPIO39     RST_GYRO  -> GPIO41
//   INT_GYRO  -> GPIO40     CS_GYRO   -> GPIO2
//
// Runs on the esp32_BNO08x component in components/ (CEVA SH-2 stack).
//
// The sensor is driven at its maximum: gyro-integrated rotation vector at
// 1 kHz over a 3 MHz SPI link. gyro_init() starts a background task that
// keeps the angle cache fresh, so the readers below never touch SPI - they
// just return the last known value and are safe to call from the control
// loop at any rate.

void gyro_init(void);

// Block until a fresh orientation sample lands, or timeout_ms passes; returns
// false on timeout. A consumer slower than the sensor (printing to the
// console, say) wakes on the newest sample and skips whatever it missed -
// this never replays a backlog, so it cannot fall further and further behind.
bool gyro_wait_sample(uint32_t timeout_ms);

// Total orientation samples since boot. Divide the difference between two
// reads by the elapsed time to get the real update rate - handy for checking
// the link is actually keeping up with the 1 kHz the sensor is configured for.
uint32_t gyro_sample_count(void);

// False until the first orientation report has been parsed (typically ~0.5s
// after gyro_init()), and again if the sensor drops off the bus.
bool gyro_ready(void);

// Heading, 0..360 deg, CLOCKWISE-POSITIVE - the convention robot_move() and
// heading_hold_omega() in motor.h expect. Zero is wherever the robot pointed
// at boot, or at the last gyro_zero_yaw().
float gyro_yaw(void);

// Turn rate about the vertical axis, deg/s, clockwise-positive - same sense as
// gyro_yaw(). Comes straight out of the same 1 kHz report the heading does, so
// it is free, and it is a MEASUREMENT rather than a difference of two headings:
// a heading controller damped with this instead of a differentiated angle does
// not amplify sampling noise into steering chatter.
float gyro_yaw_rate(void);

float gyro_pitch(void); // -90..90 deg, nose up positive
float gyro_roll(void);  // -180..180 deg, right side down positive

// Linear acceleration (gravity already removed by the fusion), m/s^2, in the
// sensor frame. Enabled as a second report alongside the rotation vector - it
// is not used for driving, only to notice that somebody picked the robot up
// off the field (a checkpoint restart / lack-of-progress reset).
//
// Both are zero and stay zero if the report could not be enabled, so a caller
// that votes on several signals degrades instead of breaking. Any pointer may
// be NULL.
void gyro_get_lin_accel(float *x, float *y, float *z);

// |linear acceleration|, m/s^2 - the cheapest single number for "the robot is
// being handled". ~0 standing still or rolling at a constant speed on the
// flat; the robot's own profile (MOVE_MAX_CMS over RAMP_UP_S) peaks near
// 2.2 m/s^2, a hand lifting it goes well past that.
float gyro_lin_accel_mag(void);

// All three in one shot, from a single consistent sample. Any pointer may be
// NULL if that axis isn't needed.
void gyro_get_angles(float *yaw_deg, float *pitch_deg, float *roll_deg);

// Make the current heading the new 0 deg. Pitch and roll are absolute
// (gravity-referenced) and are not affected.
void gyro_zero_yaw(void);

#ifdef __cplusplus
}
#endif
