#pragma once

// ESP <-> RPi link for the maze run. Same split as the previous robot: the ESP
// owns the sensors, the motors and the failsafe, the RPi owns navigation and
// sends each side's speed. One UART, full duplex - TX=GPIO3 RX=GPIO9 @ 921600,
// which on the Pi is the stock /dev/serial0 (Pi GPIO15/GPIO14).
//
// ESP -> RPi   binary, 51 bytes, little-endian, 200 Hz:
//    [0-1]    magic    0xAA 0x55
//    [2]      ver      uint8    TELEM_VER - ПОДНИМАТЬ ПРИ ЛЮБОМ ИЗМЕНЕНИИ
//                               РАСКЛАДКИ НИЖЕ. Он здесь не для красоты:
//                               раскладка уже менялась, и старый разборщик
//                               против новой прошивки НЕ замолкает, как
//                               кажется. Магия 0xAA55 встречается и внутри
//                               полезных данных, а XOR-контрольная сумма
//                               пропускает ~1 мусорный кадр из 300 - при
//                               200 Гц это примерно строка в секунду, и
//                               выглядит она как живая телеметрия с дикими
//                               числами. Один такой рассинхрон стоил
//                               отладки "энкодеры не отвечают", другой -
//                               "датчик цвета сошёл с ума". С этим байтом
//                               обе стороны говорят об этом прямо.
//    [3-6]    yaw      float32  deg 0..360, clockwise-positive
//    [7-10]   yaw_rate float32  deg/s, clockwise-positive - measured, not a
//                               difference of headings, so the RPi's heading
//                               controller can damp on it without amplifying
//                               sampling noise into steering chatter
//    [11-14]  pitch    float32  deg, nose up positive
//    [15-18]  roll     float32  deg, right side down positive. The field only
//                               tilts the robot in pitch (a ramp), so a large
//                               roll means hands - see the lift detector in
//                               nav.py, which votes on this, on accel below,
//                               and on the colour sensor going dark.
//    [19-22]  ax       float32  linear acceleration, m/s^2, gravity already
//    [23-26]  ay       float32  removed, in the SENSOR frame. All zero if the
//    [27-30]  az       float32  BNO085 refused the report.
//                               az is the yaw axis, i.e. vertical while the
//                               robot is roughly level. It is sent as three
//                               components rather than a magnitude because
//                               the magnitude cannot tell a lift from shaking:
//                               climbing steps or a ramp rattles the robot
//                               just as hard as a hand does. A lift is a
//                               SUSTAINED NET RISE, which only shows up once
//                               the vertical axis is separated out and
//                               integrated - see the lift detector in nav.py.
//    [31]     color    uint8    1=WHITE 2=BLUE 3=BLACK 4=MIRROR 5=RED. Never
//                               0 - a 0 here means the packet was misparsed.
//    [32-33]  cr       uint16   APDS9960 red,   raw
//    [34-35]  cg       uint16   APDS9960 green, raw
//    [36-37]  cb       uint16   APDS9960 blue,  raw
//    [38-39]  cc       uint16   APDS9960 clear, raw. All four are 0..10250
//                               (ATIME=0xF6, ~28ms, gain 16x) - 10250 means
//                               saturated, and the thresholds are tuned on
//                               these numbers, not on the colour byte above.
//    [40-43]  left_cm  float32  signed, since the last 'Z'
//    [44-47]  right_cm float32  signed, since the last 'Z'
//    [48]     status   uint8    0=running 1=paused
//    [49]     flags    uint8    bit0 clicker1, bit1 clicker2,
//                               bit2 left odometry live, bit3 right odometry live,
//                               bit4 BTN_1 held (GPIO47, debounced)
//    [50]     crc      uint8    XOR of bytes [2..49]
//
// On the packet size and rate: 51 bytes at 200 Hz is 10200 B/s against the
// 92160 B/s this link carries, i.e. 11% - and one packet occupies the wire for
// 0.55 ms. The payload has never been what makes telemetry feel slow, so
// widening the packet to carry roll and acceleration costs nothing, and the
// rate went 100 -> 200 Hz because the RPi's control loop closes on this
// odometry (ENC_POLL_HZ in motor.c moved with it). What actually costs
// latency is the sampling cadence on both ends and the GIL on the RPi.
//
// RPi -> ESP   ASCII, newline-terminated:
//    "M:<L>,<R>\n"  set left/right speed, cm/s at the wheel rim (float, signed)
//    "T:<L>,<R>[,<cap>]\n"  short torque assist, motor IQ amperes
//                   (software-clamped). cap - потолок скорости колеса, см/с
//                   (по умолчанию ASSIST_CAP_CMS в robot_link.c): у потолка
//                   ток сбрасывается, выше него мотор держит потолок в режиме
//                   скорости - см. assist_one() в motor.c.
//    "B\n"          active brake - closed-loop 0 dps, kills inertia, no coast.
//                   The hold is re-asserted on the ESP for a few seconds, so
//                   the motors stay locked while the RPi is busy (measuring
//                   walls, deciding) - a single 0 dps frame would quietly let
//                   go, since closed loop needs the command to keep arriving.
//    "Z\n"          zero the odometry counters
//    "R\n"          lift the BTN_1 pause (status -> running). BTN_1 itself only
//                   pauses: it is the "restart from checkpoint" button, and the
//                   RPi resumes once the robot is back on the checkpoint and
//                   GPIO25 on the Pi has been pressed.
//    "K:L\n"        dispense one rescue kit from the left rear bay (servo.h)
//    "K:R\n"        dispense one rescue kit from the right rear bay
//                   Blocks cmd_task for about a second (the servo's travel
//                   time) - harmless, since the RPi only sends this while
//                   the robot is already stopped for a victim report.
//
// The run is armed from boot: the robot moves as soon as the RPi sends its
// first "M:" command; BTN_1 pauses and only the RPi's "R" resumes. The motors are
// silent until that first command, and a gap of more than 500 ms between
// "M:" commands brakes the robot on its own.
//
// robot_link_run() does not return - call it from app_main() once buttons_init,
// apds9960_board_init, gyro_init, motor_can_init and motor_enc_start are done.
// Bump on every change to the telemetry layout above. Both readers on the RPi
// (nav.py, color.py) refuse a packet whose version is not this one and say so
// instead of quietly printing nonsense.
#define TELEM_VER 2

void robot_link_run(void);
