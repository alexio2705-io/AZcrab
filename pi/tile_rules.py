"""Правила плиток: что робот делает на плитке каждого цвета.

Одна строка таблицы RULES - один цвет:
  роль на ходу -
    NORMAL      обычная, едем как по белой;
    WALL        заезжать нельзя: откат в центр прошлой клетки, ребро закрыто
                (maze_map.block_direction) - как чёрная;
    STOP        встать в её центре, не проезжать насквозь - как синяя;
    CHECKPOINT  отсюда перезапуск - как серебряная;
  реакция по приезде - функция react(клетка) или None;
  every_visit - реагировать на каждом визите (True) или один раз (False).

Задание меняет таблицу до старта, например:
    tile_rules.RULES[config.TILE_RED] = tile_rules.Rule(tile_rules.STOP, react=моя_функция)

Карта цветов state.tile_map (клетка -> цвет) пишется каждый раз, когда
цвет клетки известен: стоя по приезде, на ходу и при откате от чёрной.
"""
from __future__ import annotations

import config
import state


NORMAL = "normal"
WALL = "wall"
STOP = "stop"
CHECKPOINT = "checkpoint"


class Rule:
    def __init__(self, role=NORMAL, react=None, every_visit=True):
        self.role = role
        self.react = react
        self.every_visit = every_visit


def _blue_stop(cell):
    import floor_tiles      # здесь, а не наверху: floor_tiles сам читает эту таблицу
    floor_tiles.blue_stop(cell)


# Обычный заезд.
RULES = {
    config.TILE_BLACK: Rule(WALL),
    config.TILE_BLUE: Rule(STOP, react=_blue_stop),
    config.TILE_SILVER: Rule(CHECKPOINT),
    config.TILE_RED: Rule(NORMAL),
}


def use_checkpoint(color):
    """Чекпоинт - плитка color (тренировка с красной: --red-checkpoint)."""
    RULES[state.CHECKPOINT_TILE] = Rule(NORMAL)
    RULES[color] = Rule(CHECKPOINT)
    state.CHECKPOINT_TILE = color


def role(color):
    rule = RULES.get(color)
    return NORMAL if rule is None else rule.role


def must_stop(colors):
    """Среди цветов, прочитанных над клеткой на ходу, есть плитка-стоп."""
    return any(role(c) == STOP for c in colors)


def remember(cell, color):
    """Записать цвет клетки в карту цветов (если он известен)."""
    if color is not None and color != config.TILE_UNKNOWN:
        state.tile_map[cell] = color


def on_arrive(cell, live):
    """Стоим в центре клетки, live - цвет под роботом сейчас. Запомнить его и
    сделать то, что велит правило. Клетка, где реакция уже была, реагирует и
    тогда, когда датчик сейчас читает другое: однажды подтверждённому факту
    доверяем больше, чем одному живому замеру (как с синей, см. blue_stop)."""
    known = state.tile_map.get(cell)
    remember(cell, live)
    color = live if _reacts(live) else state.react_cells.get(cell)
    if color is None and _reacts(known):
        color = known  # Seen on approach, possibly no longer under the floor sensor.
    rule = None if color is None else RULES.get(color)
    if rule is None or rule.react is None:
        return
    if not rule.every_visit and cell in state.react_cells:
        return
    state.react_cells[cell] = color
    rule.react(cell)


def _reacts(color):
    rule = RULES.get(color)
    return rule is not None and rule.react is not None


def cells_of(color):
    """Клетки, которые по карте цветов этого цвета."""
    return [c for c, col in state.tile_map.items() if col == color]
