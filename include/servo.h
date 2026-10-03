#pragma once
#include <stdint.h>

// Cube dispenser servo on GPIO10 (SERVO on the schematic; R30 pulls the line
// down so it stays quiet until the PWM starts).
//
// One servo serves both rear compartments: swinging it to one end releases a
// cube from the left bay, the other end releases from the right, and it rests
// in the middle with both bays closed.
//
// The dispense calls block for the length of the movement (about a second)
// and leave the servo back at centre.

void servo_init(void); // starts the 50Hz PWM and parks at centre

void servo_dispense_left(void);
void servo_dispense_right(void);

// Park position, and the escape hatch for tuning: drive any pulse width
// directly to find where the mechanical ends really are before committing
// them to SERVO_LEFT_US / SERVO_RIGHT_US in servo.c.
void servo_center(void);
void servo_set_pulse_us(uint32_t pulse_us);
