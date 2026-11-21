#pragma once

// ESP <-> RPi link for the maze run. Same split as the previous robot: the ESP
// owns the sensors, the motors and the failsafe, the RPi owns navigation and
// sends each side's speed. One UART, full duplex - TX=GPIO3 RX=GPIO9 @ 921600,
// which on the Pi is the stock /dev/serial0 (Pi GPIO15/GPIO14).
//
// ESP -> RPi   binary, 74 bytes, little-endian, 200 Hz (v4):
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
//    [3]      seq      uint8    +1 на каждый пакет - RPi считает пропуски
//    [4-7]    yaw      float32  deg 0..360, clockwise-positive
//    [8-11]   yaw_rate float32  deg/s, clockwise-positive - measured, not a
//                               difference of headings, so the RPi's heading
//                               controller can damp on it without amplifying
//                               sampling noise into steering chatter
//    [12-15]  pitch    float32  deg, nose up positive
//    [16-19]  roll     float32  deg, right side down positive. The field only
//                               tilts the robot in pitch (a ramp), so a large
//                               roll means hands - see the lift detector in
//                               nav.py, which votes on this, on accel below,
//                               and on the colour sensor going dark.
//    [20-23]  ax       float32  linear acceleration, m/s^2, gravity already
//    [24-27]  ay       float32  removed, in the SENSOR frame. All zero if the
//    [28-31]  az       float32  BNO085 refused the report.
//                               az is the yaw axis, i.e. vertical while the
//                               robot is roughly level. It is sent as three
//                               components rather than a magnitude because
//                               the magnitude cannot tell a lift from shaking:
//                               climbing steps or a ramp rattles the robot
//                               just as hard as a hand does. A lift is a
//                               SUSTAINED NET RISE, which only shows up once
//                               the vertical axis is separated out and
//                               integrated - see the lift detector in nav.py.
//    [32]     color    uint8    1=WHITE 2=BLUE 3=BLACK 4=MIRROR 5=RED. Never
//                               0 - a 0 here means the packet was misparsed.
//    [33-34]  cr       uint16   APDS9960 red,   raw
//    [35-36]  cg       uint16   APDS9960 green, raw
//    [37-38]  cb       uint16   APDS9960 blue,  raw
//    [39-40]  cc       uint16   APDS9960 clear, raw. All four are 0..10250
//                               (ATIME=0xF6, ~28ms, gain 16x) - 10250 means
//                               saturated, and the thresholds are tuned on
//                               these numbers, not on the colour byte above.
//    [41-44]  left_cm  float32  signed, since the last 'Z'
//    [45-48]  right_cm float32  signed, since the last 'Z'
//    [49]     status   uint8    0=running 1=paused
//    [50]     flags    uint8    bit0 clicker1, bit1 clicker2,
//                               bit2 left odometry live, bit3 right odometry live,
//                               bit4 BTN_1 held (GPIO47, debounced),
//                               bit5 гироскоп свежий (счётчик сэмплов BNO085
//                               рос за последние 50мс; 0 - yaw замер, а не 0°),
//                               bit6 цвет свежий (удачное чтение APDS9960 не
//                               старше 150мс; 0 - каналы выше старые)
//    [51]     z_cnt    uint8    сколько "Z" применено (по модулю 256). Растёт
//                               ПОСЛЕ обнуления энкодеров: пакет с новым
//                               значением несёт одометрию уже после "Z".
//    [52-53]  boot_id  uint16   случайное при старте ESP - смена = ребут
//    [54-55]  cmd_err  uint16   сколько строк от RPi ESP отверг (нет/не та
//                               CRC, не разобралась, NaN/inf, длиннее 63)
//    [56-63]  motor_dv uint16[4] bus voltage, 0.1 V/LSB, 0 = stale
//    [64-67]  motor_temp int8[4] Celsius, -128 = stale
//    [68-71]  motor_error uint8[4] raw status faults; order IDs 4,1,2,3
//    [72-73]  crc      uint16   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
//                               по байтам [2..71]; на RPi -
//                               binascii.crc_hqx(pkt[2:72], 0xFFFF). Была XOR
//                               в 1 байт - пропускала ~1 мусорный кадр из 300.
//
// On the packet size and rate: 74 bytes at 200 Hz is 14800 B/s against the
// 92160 B/s this link carries, i.e. 16% - and one packet occupies the wire for
// 0.80 ms. The payload has never been what makes telemetry feel slow, so
// widening the packet to carry roll and acceleration costs nothing, and the
// rate went 100 -> 200 Hz because the RPi's control loop closes on this
// odometry (ENC_POLL_HZ in motor.c moved with it). What actually costs
// latency is the sampling cadence on both ends and the GIL on the RPi.
//
// RPi -> ESP   ASCII, newline-terminated, каждая строка с CRC:
//    "<команда>*XXXX\n", XXXX - CRC-16/CCITT-FALSE (как у телеметрии) текста
//    до '*', 4 hex-цифры. Строку без '*' или с неверной суммой ESP
//    отбрасывает и растит cmd_err. Ниже команды - без этого хвоста.
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
// v4: append 4*u16 decivolts, 4*i8 temperatures, 4*u8 faults before CRC.
// Motor order 4,1,2,3; packet length 74. Update Raspberry Pi at the same time.
#define TELEM_VER 4

void robot_link_run(void);
