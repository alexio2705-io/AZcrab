#pragma once
#include <stdbool.h>
#include <stdint.h>

// The three momentary inputs on the board: a panel button and two clickers.
// Each has a 10k pull-up to +3V3_ESP and a 0.1uF cap, so they arrive already
// RC-filtered.
//
//   BTN_1     -> GPIO47     CLICKER_1 -> GPIO1   (left)
//                           CLICKER_2 -> GPIO4   (right)
//
// BTN_1 was on GPIO48 per the schematic, but that button itself was dead, so
// a new one was soldered onto GPIO47 (the schematic's BTN_2 pad, which never
// worked with its original button either). It reads the same way: active low,
// and the internal pull-up is on in case the external one is missing.
//
// The two families read with opposite sense on the assembled board - the
// button goes low when held, the clickers go high - and buttons.c hides that,
// so everything below means "pressed" for all three alike.
//
// buttons_init() adds a 30ms software debounce on top and polls everything
// from a background task, so the readers below never touch a GPIO and are
// safe to call from the control loop at any rate.

typedef enum {
    BTN_1,
    CLICKER_1,
    CLICKER_2,
    BUTTON_COUNT,
} button_t;

void buttons_init(void);

// Debounced level: true for as long as the button is held down. This is what
// a clicker used as a bumper or limit switch wants.
bool button_pressed(button_t b);

// Raw pin level, straight off the GPIO with no debounce and no polarity
// applied - for checking wiring on the bench, not for control logic.
int button_raw_level(button_t b);

// How many raw level changes this pin has seen since boot, sampled every 5ms
// with no debounce and no polarity. Never cleared - for proving on the bench
// that a press reaches the pin at all, however briefly.
uint32_t button_edge_count(button_t b);

// True once per press, and reading it clears it - so a start/pause handler
// polling in a loop reacts to each press exactly once, no matter how long the
// button is held or how often the loop asks.
bool button_clicked(button_t b);
