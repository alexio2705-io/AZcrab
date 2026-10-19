#!/usr/bin/env python3
"""Когнитивное задание (5×5).

Условие: поле 5×5, старт в углу, горки могут быть. В стартовой и в
следующей клетке - когнитивные метки из красных, чёрных и синих колец,
комплектов на них нет. Цвета колец от центра наружу - порядок плиток.
Плиток каждого цвета столько, сколько колец этого цвета. Заезжать можно на
любые, по чёрной тоже, но мигать и выдавать комплект - только на плитке,
чья очередь. Φ/Ψ/Ω и H/S/U - по одному комплекту сразу у метки. Финиш на
чекпоинте (серебряная). Лимит 5 минут.

Что делает робот:
- когнитивные метки только читает (кольца), без мигания и комплектов;
  порядок - голосование двух меток по каждому кольцу, цвет не из набора -
  ближайший (жёлтый → красный, зелёный → синий);
- красная, чёрная, синяя - обычные плитки (чёрная - не стена, синей
  стоянки нет); на плитке, чья очередь: мигание как у жертвы, 1 комплект;
- буквы Φ/Ψ/Ω/H/S/U - жертвы, 1 комплект (спиной к стене, как обычно);
- разведка, пока следующая по очереди плитка не известна; известна - едет к ней;
- всё сделано или осталось время только на дорогу - на серебряную (нет её -
  на старт) и мигание, как на старте после заезда.

Запуск: ~/maze/venv/bin/python ~/maze/task_cog.py
"""
import config
import state
import actions
import boot
import task_logic
import tile_rules

LIMIT_S = 300
FINISH_BLINK_S = 11
START = (0, 0)
SILVER = config.TILE_SILVER

config.cog_marks_not_victims()
config.KITS_BY_TYPE.update(damaged=1, stable=1, unharmed=1)
for _cls in ("H", "S", "U"):
    config.add_victim_class(_cls, kits=1)

queue = None            # очередь плиток - когда прочитаны кольца
w = timer = None        # робот на поле и часы - после кнопки старта


def tiles_queue():
    """Очередь по кольцам: прочитаны две метки (или одна, а робот уже ушёл
    со стартовых клеток). До этого - None."""
    global queue
    reads = [m["rings"] for m in actions.marks_found({"cognetive"}) if m.get("rings")]
    if queue is None and (len(reads) >= 2 or (reads and w.total >= 2)):
        queue = task_logic.ColorQueue(task_logic.ring_order(reads))
        print(f"  порядок плиток по кольцам: {queue.order}")
    return queue


def targets():
    q = tiles_queue()
    return [] if q is None else q.targets(state.tile_map)


def finish_cell():
    silver = actions.cells_of(SILVER)
    return actions.nearest(w, silver) if silver else START


def late():
    return timer.time_to_go(w, finish_cell(), extra_s=FINISH_BLINK_S)


def tile_here(cell):
    """Стоим на клетке: если это плитка, чья очередь, - отработать её."""
    q = tiles_queue()
    if q is None or not q.is_turn(cell, state.tile_map.get(cell)) or late():
        return
    print(f"  ★ плитка {q.next_color()} ({len(q.done) + 1}/{len(q.order)})")
    actions.blink_victim()
    actions.give_kits(w, 1)
    q.mark(cell)


for _color in (config.TILE_RED, config.TILE_BLACK, config.TILE_BLUE):
    tile_rules.RULES[_color] = tile_rules.Rule(tile_rules.NORMAL, react=tile_here)


def scenario(run):
    global w, timer
    w = actions.new_walk()
    timer = actions.Timer(LIMIT_S)
    actions.look_around(w)          # стартовая клетка: стены, плитка, метки
    actions.visit_targets(w, targets, tile_here,
                          lambda: queue is not None and queue.finished(), late)
    if not actions.cells_of(SILVER):
        actions.explore_until(w, lambda: bool(actions.cells_of(SILVER)) or late())
    actions.go_to_cell(w, finish_cell())
    actions.blink_finish()


if __name__ == "__main__":
    boot.run(scenario)
