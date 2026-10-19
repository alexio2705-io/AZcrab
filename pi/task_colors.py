#!/usr/bin/env python3
"""Задание «Цвета + H/S» (docs/tech_challenges.md, задание 2).

Условие: на поле 2 красные, 2 синие и 1 чёрная плитка. На чёрной стоять
3 с и мигнуть 3 раза. Синюю проезжать без всякой реакции. На красной
стоять 5 с и мигнуть 5 раз - на каждом заезде. Метки H и S - определить.
Финиш на чёрной (стоять 10 с), но только после посещения синей и красной.

Что делает робот:
- чёрная и красная - «стоп в центре»; мигание 0,5/0,5 с: 3 раза (3 с
  стоянки) и 5 раз (5 с), на каждом визите;
- синяя - обычная плитка, стоянки на ней нет;
- H и S - жертвы: мигание как у жертвы, без комплектов. Как показать
  «определение» - уточнить у судей и поменять тут (kits=...);
- разведка, пока не посещены синяя и красная, не найдены чёрная и обе
  буквы (или пока есть что разведывать);
- финиш: едет на чёрную (по приезде мигнёт 3 раза), моторы выключены 10,5 с.
  Чёрной не нашлось - финиш на старте.

Запуск: ~/maze/venv/bin/python ~/maze/task_colors.py
"""
import config
import state
import actions
import boot
import tile_rules

FINISH_S = 10.5
START = (0, 0)
BLUE, RED, BLACK = config.TILE_BLUE, config.TILE_RED, config.TILE_BLACK
LETTERS = {"H", "S"}

config.add_victim_class("H", kits=0)
config.add_victim_class("S", kits=0)
tile_rules.RULES[BLACK] = tile_rules.Rule(tile_rules.STOP, react=lambda cell: actions.blink(3))
tile_rules.RULES[RED] = tile_rules.Rule(tile_rules.STOP, react=lambda cell: actions.blink(5))
tile_rules.RULES[BLUE] = tile_rules.Rule(tile_rules.NORMAL)


def ready_to_finish():
    """Синяя проехана, на красной отреагировали, чёрная и обе буквы найдены."""
    seen = set(state.tile_map.values())
    letters = {m["class"] for m in actions.marks_found(LETTERS)}
    return (BLUE in seen and RED in state.react_cells.values()
            and BLACK in seen and letters >= LETTERS)


def scenario(run):
    w = actions.new_walk()
    actions.look_around(w)          # стартовая клетка: стены, плитка, метки
    actions.explore_until(w, ready_to_finish)
    if actions.go_to_color(w, BLACK) is None:
        print("  чёрной на карте нет - финиш на старте")
        actions.go_to_cell(w, START)
    actions.stand_motors_off(FINISH_S)


if __name__ == "__main__":
    boot.run(scenario)
