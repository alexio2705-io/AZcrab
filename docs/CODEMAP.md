# Карта кода

Генерируется `.claude/hooks/codemap.py` после каждой правки кода. Руками не править.
Где что менять по задачам — ручной указатель `docs/robot_code_map.md`.
Тесты Pi — `pi/tests/` (unittest).

## Корень (4)

- `detect_app.py` — Веб-страница live-детекции жертв. · `FrameStore`, `detection_loop`, `mjpeg_generator`, `index` … ещё 3
- `map_web.py` — Веб-страница карты лабиринта и найденных меток. · `index`, `state`, `detection_image`
- `victim_detect.py` — Детектор жертв для RoboCup Rescue Maze. · `VictimDetector`
- `victim_static_test.py` — Статический тест детектора жертв — робот стоит на месте, без езды. · `main`

## pi/ — Raspberry Pi (79)

- `pi/actions.py` — Глаголы для заданий технического испытания. Задание - файл task_*.py, · `blink`, `blink_victim`, `blink_finish`, `stand` … ещё 15
- `pi/battery_probe.py` — Напряжение батареи для меню экрана, пока nav.py не запущен. · `port_in_use`, `volts_from_bytes`, `read_volts`, `BatteryProbe`
- `pi/boot.py` — Запуск железа и уборка - общие для обычного заезда (nav.py) и заданий · `Run`, `start`, `stop`, `run`
- `pi/bringup.py` — Проверка Pi после включения: всё ли на месте перед заездом. Только читает. · `report`, `run`, `busy_processes`, `check_python` … ещё 14
- `pi/bumper.py` — Бампер: концевики, что делать при ударе на ходу и после остановки. · `get_bumper`, `both_pressed`, `bumper_side`, `check_bumper_while_driving` … ещё 2
- `pi/challenge.py` — Режим технического испытания - заготовка, чтобы на площадке не писать · `load_config`, `signal_timing`, `describe`, `print_config` … ещё 13
- `pi/challenge_blocks.py` — Small, hardware-free challenge description. Hardware imports happen only in run(). · `Block`, `wait`, `blink`, `kits` … ещё 4
- `pi/color.py` — Сырые каналы APDS9960 - живой вывод и калибровка порогов распознавания плиток. · `link_report`, `nav_verdict`, `calibrate`, `main`
- `pi/colors_page.py` — Страница /colors: разметка цветов колец вручную и подбор порогов по ней. · `register`
- `pi/config.py` — Константы, которые нужны сразу нескольким модулям. · `add_victim_class`, `cog_marks_not_victims`
- `pi/continuous.py` — Hardware-free continuous-motion policies shared with the simulator (cm, s). · `stopping_speed`, `observation_speed`, `timed_rear`, `stop_suspended` … ещё 3
- `pi/corridor.py` — Стены клеток впереди по точкам лидара на ходу - для проезда прямых без · `corridor_points`, `point_time`, `to_path_frame`, `visible_steps` … ещё 2
- `pi/dash_live.py` — Дашборд, живая карта, светодиоды и детектор подъёма. Перенесено из nav.py как есть. · `leds_lifted`, `leds_sleep`, `blink_start_return`, `start_map_web` … ещё 2
- `pi/dash_web.py` — Пульт робота: живой дашборд заезда в браузере + запись и повтор заездов. · `hist_row`, `SysStats`, `Hub`, `feed_loop` … ещё 8
- `pi/dataset_web.py` — Сбор датасета для дообучения YOLO с двух камер робота. · `load_settings`, `log`, `Camera`, `cam_settings_controls` … ещё 7
- `pi/drive_checks.py` — Шаги поездки: аварийные проверки (снят, пауза, связь, энкодеры, застревание) и одометрия. · `integrate_motion`, `check_abort`, `update_odometry`, `check_progress` … ещё 4
- `pi/drive_front.py` — Шаги поездки: передняя стена, стены клеток впереди и решение у границы клетки. · `track_front_wall`, `wall_not_floor`, `collect_cell_walls`, `remaining_distance` … ещё 2
- `pi/drive_motion.py` — Шаги поездки: скорость, центровка, руль, команда моторам, статус. · `speed_profile`, `wall_offset`, `steering`, `send_motor_command` … ещё 2
- `pi/drive_ramp.py` — Шаги поездки: наклон пола (ступенька, горка) и ассист моментом. · `track_slope`, `update_assist`
- `pi/drive_stop.py` — Шаги поездки: где тормозить и что делать, когда доехали. · `aim_at_mark`, `check_arrived`
- `pi/drive_trip.py` — Поездка drive_cells: объект trip со всем её состоянием и подготовка к старту. · `Trip`, `start_trip`
- `pi/driving.py` — Езда вперёд по клеткам: такт поездки по шагам (run_tick) и короткие подъезды (creep_mm). · `drive_cells`, `run_tick`, `finish_trip`, `creep_mm` … ещё 1
- `pi/esp_link.py` — Связь с ESP: телеметрия, команды моторам, гироскоп, энкодеры, кнопки. · `crc16`, `link_report`, `link_errors`, `get_yaw` … ещё 31
- `pi/explore.py` — Обход лабиринта и возврат на старт. · `Walk`, `step`, `go_to`, `unvisited` … ещё 4
- `pi/floor_pose.py` — Navigation pose for floor projection. x forward, y left, yaw left-positive. · `from_snapshot`
- `pi/floor_tiles.py` — Цвет плитки под роботом: белая, синяя, чёрная, чекпоинт; калибровка белого. · `black_threshold`, `tile_color`, `blue_stop`, `load_tile_calib` … ещё 2
- `pi/front_cam.py` — Передняя USB-камера: метки вживую, реакция кота на дисплее и на странице. · `UsbCamera`, `Confirm`, `read_frame`, `draw_tile` … ещё 8
- `pi/front_floor.py` — Project the next grid tile into the prepared USB-camera frame. · `model_point`, `model_h`, `pitched_h`, `prep` … ещё 12
- `pi/front_victim.py` — Метки на передней стене - передней USB-камерой, стоя (для nav.py). · `square`, `FrontCamera`, `FrontVictimDetector`
- `pi/ld19.py` — Чтение лидара LDROBOT LD19 на Raspberry Pi. · `crc8`, `Packet`, `to_robot`, `parse_packet` … ещё 2
- `pi/ld19_web.py` — Живая визуализация LD19: сервер на Пи, радар в браузере. · `Lidar`, `Handler`, `main`
- `pi/leds.py` — Лента WS2812 на Raspberry Pi: драйвер поверх SPI + мигание розовым. · `NeoPixels`, `blink_pink`, `blink_n`, `blink_return` … ещё 1
- `pi/letters_cv.py` — Буквы H / S / U на стене без нейросети - запасной вариант для технического · `classify_glyph`, `classify_mask`, `find_letter`, `main`
- `pi/lidar_proc.py` — LD19 в отдельном процессе: читает порт, считает стены (walls.measure) и · `main`
- `pi/lidar_walls.py` — Лидар: поток сканов и решение, есть ли стена с каждой стороны. · `get_walls`, `get_wall_snapshot`, `get_timed_back`, `get_walls_still` … ещё 6
- `pi/lift.py` — Признаки «робота сняли с поля» - замер и подбор порогов для LIFT_* в nav.py. · `link_report`, `Detector`, `selftest`, `main`
- `pi/live.py` — Живой поток состояния nav.py для пульта (dash_web.py). · `LogTee`, `LivePublisher`, `read_lines`
- `pi/manhattan.py` — Курс по стенам ("манхэттен"): стены поля идут только по двум направлениям, · `segments`, `wall_angle`
- `pi/marks.py` — Метки жертв, замеченные не стоя в центре клетки. · `watch_marks_while_driving`
- `pi/maze_map.py` — Карта лабиринта: стены, рёбра, голосование, поиск пути. · `vote_wall`, `close_edge`, `open_edge`, `set_cell_walls` … ещё 6
- `pi/motion_estimate.py` — Checked wheel/heading integration. Distances are cm; yaw is clockwise degrees. · `MotionDiscontinuity`, `MotionSample`, `ProjectedOdometry`, `rebase`
- `pi/motion_guard.py` — Conservative longitudinal slip evidence; missing ranges mean unobservable. · `SlipGuard`, `CenterFilter`
- `pi/nav.py` — Проезд лабиринта: разведка BFS/flood-fill по неизвестному полю, разгон/ · `main`
- `pi/neopixel_test.py` — Тест ленты с робота: экран и кнопки у малины (его же запускает меню на · `Test`, `main`
- `pi/oled.py` — Отладочный OLED 128x64 (SSD1315, I2C) сзади робота. · `Canvas`, `Ssd1306`, `say`, `daemon_running` … ещё 55
- `pi/oled_cat.py` — Кот для дисплея робота: движение - из ролика, черты - нарисованные. · `fill_ellipse`, `fill_poly`, `line`, `background` … ещё 7
- `pi/oled_font.py` — Растровые шрифты для oled.py - misc-fixed 5x8 и 6x10 из X11
- `pi/oled_moods.py` — Кот «говорит», какую букву прочитал робот: слева кот, справа буква во · `lit`, `put_cat`, `stroke`, `arc` … ещё 11
- `pi/oled_video.py` — Видео для дисплея робота (oled.py): ролик -> кадры 128x64 по 1 биту. · `check_output`, `load`, `frame_bits`, `make` … ещё 1
- `pi/phone_web.py` — Запуск заезда с телефона: страница с кнопками «Запустить» / «Стоп». · `proc_cmdlines`, `find_script`, `proc_start`, `stdout_file` … ещё 10
- `pi/pi_keys.py` — Кнопки у малины для программ, которые управляются с робота (меню на · `Debounce`, `Keys`
- `pi/protocol.py` — Shared ESP wire layout. Keep synchronized with esp32/include/robot_link.h.
- `pi/ramps.py` — Горки: наклон, замер длины, запись в карту. · `tilt_deg`, `is_tilted`, `record_ramp`, `level_of`
- `pi/raw_scan.py` — Сырой скан LD19 без фильтров: CSV + сводка по ближним точкам. · `main`
- `pi/recovery.py` — Старт и восстановление: кнопки, снятие с поля, чекпоинт, ожидание моторов. · `wait_start_button`, `handle_lift`, `detect_heading`, `restart_from_checkpoint` … ещё 1
- `pi/remote_start.py` — Старт с телефона (phone_web.py) вместо кнопки на роботе. · `request`, `clear`, `take`
- `pi/ring_colors.py` — Чтение цветов колец когнитивной цели. · `load_config`, `save_config`, `classify`, `classify_map` … ещё 10
- `pi/runplan.py` — Маршрут по карте клеток - BFS из nav.find_path_bfs, вынесенный сюда, чтобы · `weighted_path`, `bfs_path`, `go_straight`, `black_retreat`
- `pi/servo_test.py` — Проверка сервы дозатора с малины - те же команды, что шлёт nav.py. · `nav_running`, `esp_alive`, `dispense`, `interactive` … ещё 1
- `pi/state.py` — Общее состояние робота: карта, телеметрия, флаги заезда. Меняется во время работы.
- `pi/task_blocks.py` — Training example. Preview: python task_blocks.py --show; run without --show.
- `pi/task_cog.py` — Когнитивное задание (5×5). · `tiles_queue`, `targets`, `finish_cell`, `late` … ещё 2
- `pi/task_colors.py` — Задание «Цвета + H/S» (docs/tech_challenges.md, задание 2). · `ready_to_finish`, `scenario`
- `pi/task_logic.py` — Чистая логика заданий, без железа - проверяется тестами · `ColorQueue`, `tile_of_ring`, `ring_order`
- `pi/task_template.py` — Training challenge example: change TABLE and FINISH_COLOR before the run. · `react`, `scenario`
- `pi/task_tomsk.py` — Задание «Томск» (16.05.2026, docs/tech_challenges.md, задание 1). · `late`, `zone_here`, `scenario`
- `pi/telem.py` — Телеметрия ESP в консоль - для отладки линка, кнопок и датчиков. · `flags_str`, `main`
- `pi/teleop.py` — Ручной пульт моторов с телефона: джойстик на странице -> "M:л,п" на ESP. · `mix`, `Target`, `nav_running`, `Esp` … ещё 3
- `pi/tile_calib.py` — Калибровка датчика цвета по плиткам на поле: с робота (экран, кнопки, · `Lamp`, `measure`, `on_key`, `calib_screen` … ещё 6
- `pi/tile_classify.py` — Распознавание плитки по APDS9960 по ОБРАЗЦАМ, снятым на поле. · `features`, `distance`, `load`, `save` … ещё 2
- `pi/tile_rules.py` — Правила плиток: что робот делает на плитке каждого цвета. · `Rule`, `use_checkpoint`, `role`, `must_stop` … ещё 3
- `pi/timing.py` — Учёт времени по категориям (декоратор @timing._timed) и отчёты по нему. · `time_report`, `tick_report`
- `pi/turning.py` — Повороты: поворот на угол, доворот на курс, выравнивание у стены. · `angle_err`, `snap_target`, `turn`, `gyro_heading` … ещё 2
- `pi/victim_watch.py` — Наблюдатель меток жертв на ходу - сторона nav.py. · `attach_shm`, `WatchBox`, `VictimWatcher`
- `pi/victim_web.py` — Live-детекция меток на обеих камерах с веб-страницей. · `get_cfg`, `set_cfg`, `cog_verdict`, `crop_box` … ещё 13
- `pi/victims.py` — Жертвы: осмотр сторон камерами, сообщение о жертве, выдача аптечки. · `dispense`, `kits_left`, `dispense_kit`, `scan_sides` … ещё 3
- `pi/walls.py` — Расстояния до передней, правой, левой и задней стен. · `self_outline_mm`, `is_masked`, `is_left_rope`, `without_unsupported_rope_edges` … ещё 4
- `pi/watch_logic.py` — Чистая логика наблюдателя меток жертв на ходу (victim_watch.py, nav.py). · `prefer_ncnn`, `victim_boxes`, `px_to_mm`, `mark_pos_mm` … ещё 19
- `pi/watch_proc.py` — YOLO наблюдателя меток на ходу - отдельным процессом (см. victim_watch.py). · `save`, `main`

## camera_web/ (1)

- `camera_web/app.py` — — · `StreamingOutput`, `start_camera`, `mjpeg_generator`, `index` … ещё 1

## src/ + include/ — ESP32-S3 (17)

- `include/apds9960.h` — APDS9960 colour sensor, on connector J13 (COLORSENSOR on the schematic). · `apds9960_board_init`, `apds9960_read_id`, `apds9960_init`, `apds9960_is_data_ready` … ещё 2
- `include/buttons.h` — The three momentary inputs on the board: a panel button and two clickers. · `buttons_init`, `button_pressed`, `button_raw_level`, `button_edge_count` … ещё 1
- `include/drive_test.h` — Стендовый тест шин: по нажатию BTN_1 робот сам, без RPi, либо крутится на · `drive_test_run`
- `include/gyro.h` — Adafruit BNO085 (BNO08x) 9-DoF IMU, SPI mode. · `gyro_init`, `gyro_wait_sample`, `gyro_sample_count`, `gyro_ready` … ещё 8
- `include/link_test.h` — ESP<->RPi UART link test. · `link_test_run`
- `include/motor.h` — LK-TECH / MF servo motor CAN driver (MF4015 V2 / MG4005 / compatible). · `motor_can_init`, `motor_can_log_status`, `motor_get_health`, `motor_wake` … ещё 24
- `include/robot_link.h` — ESP <-> RPi link for the maze run. Same split as the previous robot: the ESP · `robot_link_run`
- `include/servo.h` — Cube dispenser servo on GPIO10 (SERVO on the schematic; R30 pulls the line · `servo_init`, `servo_dispense_left`, `servo_dispense_right`, `servo_center` … ещё 1
- `src/apds9960.c` — реализация `include/apds9960.h`
- `src/buttons.c` — реализация `include/buttons.h`
- `src/drive_test.c` — реализация `include/drive_test.h`
- `src/gyro.cpp` — реализация `include/gyro.h`
- `src/link_test.c` — реализация `include/link_test.h`
- `src/main.c` — точка входа прошивки (`app_main`)
- `src/motor.c` — реализация `include/motor.h`
- `src/robot_link.c` — реализация `include/robot_link.h`
- `src/servo.c` — реализация `include/servo.h`
