#pragma once

// ESP<->RPi UART link test.
//
// Один UART, полный дуплекс - TX=GPIO3 RX=GPIO9, подтверждено по нетлисту
// платы: ESP GPIO3(TX, net TXD2) через R23 -> Pi физ. пин 10 (GPIO15),
// ESP GPIO9(RX, net RXD2) через R8 <- Pi физ. пин 8 (GPIO14) - штатный
// /dev/serial0 на Pi, dtoverlay не нужен.
//
// ESP -> RPi: бинарный пакет @ 20 Гц, little-endian, TELEM_SIZE байт - см.
//     link_test.c за точной раскладкой (yaw, pitch, цвет, оба концевика,
//     оба энкодера, и последнее число, отражённое от RPi).
//
// RPi -> ESP: ASCII, с переводом строки:
//     "D:<float>\n"   число с лидара, мм - только чтобы доказать, что этот
//                     канал работает; ESP просто отражает его в следующем
//                     пакете телеметрии.
//     "Z\n"           обнулить базу энкодеров (motor_zero_encoders()).
//
// link_test_run() не возвращается - вызывать из app_main() после того, как
// подняты все зависимости (buttons_init, apds9960_board_init, gyro_init,
// motor_can_init + motor_enc_start).
void link_test_run(void);
