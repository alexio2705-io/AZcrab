#pragma once
#include <stdint.h>

// LK-TECH / MF servo motor CAN driver (MF4015 V2 / MG4005 / compatible).
// Bus: CAN, 1 Mbps, standard frame, 8 data bytes. CAN_TX=GPIO8, CAN_RX=GPIO18.
// Identifier: 0x140 + id (id = 1..32).
//
// This is a robot: an occasional dropped CAN frame among thousands is normal
// and harmless (the next control tick resends it), so nothing here returns
// or propagates esp_err_t - callers just fire commands and move on.

// 4-wheel differential drive, 2 wheels per side (NOT omni - a previous
// version of this file assumed a mecanum/omni platform; that was wrong for
// this robot). Left = ids 1 and 5, right = ids 2 and 3; id 3 spins backwards
// as mounted, so it's sign-flipped everywhere below. Same ids and same
// pairing as the encoder read-back at the bottom of this file.
//
// The whole right pair is mirror-mounted relative to the left one, so its
// positive direction is the robot's backwards - see MOTOR_RIGHT_SIGN in
// motor.c, which is applied to the right side's speed commands AND to its
// encoder read-back, so "forward" counts up on both sides.
#define MOTOR_LEFT_1  1
#define MOTOR_LEFT_2  5
#define MOTOR_RIGHT_1 2
#define MOTOR_RIGHT_2 3   // sign-flipped again on top of the side sign

void motor_can_init(void);

// Logs bus state, error counters, and our own TX success/fail count - see
// the comment above its definition in motor.c for how to read it.
void motor_can_log_status(void);
// LK legacy 0x9A status1. Zero voltage / -128 temperature means no fresh reply.
void motor_get_health(uint8_t id, uint16_t *decivolts, int8_t *temp, uint8_t *error);

void motor_wake(uint8_t id);     // 0x88: wake the motor out of a stopped state
void motor_stop(uint8_t id);     // 0x81: stop, keeps running state (silent coast)
void motor_shutdown(uint8_t id); // 0x80: shutdown, clears running state

// Closed-loop speed control. dps is the target speed in real degrees/second
// (positive or negative); the 0.01 dps/LSB protocol scaling happens inside.
// The motor needs this command to keep arriving to stay in closed-loop mode -
// call it repeatedly (e.g. every few ms), not just once.
void set_motor_speed(uint8_t id, float dps);

// Closed-loop torque-current command (LK 0xA1). This bypasses the driver's
// configured Max Torque Current, so motor.c clamps every request to a small
// software limit. Intended only for a short obstacle boost, never cruising.
void set_motor_torque_amps(uint8_t id, float amps);

// Per-motor acceleration limit (deg/s^2) for closed-loop speed, written to the
// motor's RAM. robot_set_accel() below does all four at once.
void motor_set_accel(uint8_t id, int32_t dps_per_s);

// ---------------------------------------------------------------------------
// Differential-drive kinematics.
//
// Wheel diameter is 86 mm (measured). TRACK_CM in motor.c is only used by
// robot_drive()'s omega mixing and is still a guess - it does not matter for
// the maze run, where every turn is closed on the gyro and the RPi sends each
// side's speed directly.
// ---------------------------------------------------------------------------

// Sets each side's wheel pair to a closed-loop speed directly, in motor
// deg/s (both wheels on a side always get the same command - there's no way
// to run same-side wheels at different speeds on this drivetrain). Needs
// calling repeatedly to stay in closed loop, same as set_motor_speed().
void robot_set_side_speed_dps(float left_dps, float right_dps);

// Same, in cm/s at the wheel rim instead of raw motor deg/s.
void robot_set_side_speed_cms(float left_cms, float right_cms);

// Short, software-limited torque boost for climbing an obstacle. Signs use
// the same robot convention as speed: positive on both sides = forward.
// cap_cms - потолок скорости колеса (см/с на ободе): каждый мотор отдельно
// сбрасывает ток по мере подхода к нему и выше него переходит в режим
// скорости на потолке - см. assist_one() в motor.c, почему без этого робот
// слетал с препятствия.
void robot_set_side_torque_amps(float left_amps, float right_amps, float cap_cms);

// Forward speed + turn rate -> both sides' speeds. Replaces the old omni
// robot_move() - a diff-drive robot can't strafe, so there's no sideways
// component to take.
// speed_cms: forward speed,  positive = forward   (cm/s)
// omega_dps: rotation rate,  positive = clockwise (deg/s)
void robot_drive(float speed_cms, float omega_dps);

// P controller for heading-hold: call every tick and feed the result as
// robot_drive()'s omega_dps to keep the robot's own heading locked on
// target_yaw_deg (e.g. while translating with omega=0 would otherwise let
// the nose drift/spin freely on bumps). Positive output = clockwise, wraps
// correctly across the 0/360 boundary.
float heading_hold_omega(float target_yaw_deg, float yaw_deg);

// Stops/wakes every wheel motor (all 4 real ids) in one call.
// robot_stop_all() is 0x81: the motors go silent and the robot COASTS.
void robot_stop_all(void);
void robot_wake_all(void);

// Active brake: closed-loop 0 dps on all four wheels. Unlike robot_stop_all()
// the motors fight the remaining inertia instead of coasting on, which is what
// ends a drive or a turn on the spot. Follow with robot_stop_all() for a
// silent idle once the robot has settled.
void robot_brake(void);

// Acceleration limit (deg/s^2) for the closed-loop speed mode, written to all
// four wheel motors' RAM (LK command 0x34). Without it the motors ramp at
// their own (usually low) default and the robot never reaches the commanded
// speed within one maze cell.
void robot_set_accel(int32_t dps_per_s);

// Distance travelled by each side since the last motor_zero_encoders(), in mm,
// positive = forward on both sides. NAN if that side's motors are both offline.
float robot_left_dist_mm(void);
float robot_right_dist_mm(void);

// ---------------------------------------------------------------------------
// Encoder read-back (added for the ESP<->RPi UART link test).
//
// motor.c only ever sent commands - nothing decoded a reply, so there was no
// way to read a position back. This adds one RX task (consumes 0x92 GET_POS
// replies off the same TWAI bus motor_can_init() already brought up) plus a
// poll task that asks for them. Same ids and pairing as MOTOR_LEFT_*/
// MOTOR_RIGHT_* above.
// ---------------------------------------------------------------------------
#define ENC_LEFT_ID_1  MOTOR_LEFT_1
#define ENC_LEFT_ID_2  MOTOR_LEFT_2
#define ENC_RIGHT_ID_1 MOTOR_RIGHT_1
#define ENC_RIGHT_ID_2 MOTOR_RIGHT_2   // sign-flipped in motor_right_enc_deg()

// One-shot bench diagnostic: asks each id in [first_id, last_id] for its
// position, one id at a time (10 requests over ~200ms, then move on), and
// logs OK+angle or NO REPLY for every one of them - so you get a clear
// per-motor answer instead of one aggregate bus counter. Needs the RX task
// already running, so call it after motor_enc_start().
void motor_probe(uint8_t first_id, uint8_t last_id);

void motor_enc_start(void);              // starts RX+poll tasks; call once after motor_can_init()
float motor_get_angle_deg(uint8_t id);   // multi-loop angle, deg; NAN if that id never answered
float motor_get_speed_dps(uint8_t id);   // wheel deg/s in the motor's own sign, from 0x92 replies; NAN if offline
void motor_zero_encoders(void);          // marks current position as the new zero for all 4 ids
float motor_left_enc_deg(void);          // avg(id1, id5) since last zero, deg; NAN if both offline
float motor_right_enc_deg(void);         // avg(id2, -id3) since last zero, deg; NAN if both offline
