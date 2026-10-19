#!/usr/bin/env python3
"""Задание «Томск» (16.05.2026, docs/tech_challenges.md, задание 1).

Условие: 3 минуты, лабиринт заранее неизвестен. В тупиках лежат синяя,
красная и чёрная плитки - посетить строго по порядку синяя → красная →
чёрная. На плитке мигать (1 с ВКЛ / 1 с ВЫКЛ): синяя 1 раз, красная 3,
чёрная 3; сбросить комплекты: 1 / 2 / 3. В конце вернуться на старт и
стоять не меньше 10 с.

Что делает робот:
- синяя, красная, чёрная - «стоп в центре» (на чёрную заезжает до центра),
  стоянки на синей нет;
- разведка, пока следующая по очереди зона не известна; известна - едет к ней;
- зона не по порядку - только запоминается (карта цветов);
- на своей зоне - мигание и комплекты на месте, без разворота;
- осталось время только на дорогу - домой; на старте моторы выключены 11 с;
- камер нет (лента есть); сделанные зоны помнятся и после перезапуска.

Запуск: ~/maze/venv/bin/python ~/maze/task_tomsk.py
"""
import config
import state
import actions
import boot
import task_logic
import tile_rules

LIMIT_S = 180
FINISH_S = 11
START = (0, 0)
BLUE, RED, BLACK = config.TILE_BLUE, config.TILE_RED, config.TILE_BLACK
BLINKS = {BLUE: 1, RED: 3, BLACK: 3}
KITS = {BLUE: 1, RED: 2, BLACK: 3}

zones = task_logic.ColorQueue([BLUE, RED, BLACK])
w = timer = None        # робот на поле и часы - после кнопки старта


def late():
    return timer.time_to_go(w, START, extra_s=FINISH_S)


def zone_here(cell):
    """Стоим на клетке: если это зона, чья очередь, - отработать её."""
    color = state.tile_map.get(cell)
    if color is None or not zones.is_turn(cell, color) or late():
        return
    print(f"  ★ зона {color} ({len(zones.done) + 1}/3)")
    actions.blink(BLINKS[color], on_s=1.0, off_s=1.0)
    actions.give_kits(w, KITS[color])
    zones.mark(cell)


for _color in (BLUE, RED, BLACK):
    tile_rules.RULES[_color] = tile_rules.Rule(tile_rules.STOP, react=zone_here)


def scenario(run):
    global w, timer
    w = actions.new_walk()
    timer = actions.Timer(LIMIT_S)
    actions.look_around(w)          # стартовая клетка: стены, плитка, метки
    actions.visit_targets(w, lambda: zones.targets(state.tile_map), zone_here,
                          zones.finished, late)
    actions.go_to_cell(w, START)
    actions.stand_motors_off(FINISH_S)


if __name__ == "__main__":
    boot.run(scenario, cameras=False)
