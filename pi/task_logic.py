"""Чистая логика заданий, без железа - проверяется тестами
(tests/test_task_logic.py). Сами задания - task_*.py."""
from __future__ import annotations


class ColorQueue:
    """Плитки по очереди цветов: Томск - синяя → красная → чёрная,
    когнитивное - по кольцам. Каждая клетка отрабатывается один раз;
    плитка не по очереди только запоминается (в карте цветов)."""

    def __init__(self, order):
        self.order = list(order)
        self.done = []          # отработанные клетки, по порядку

    def next_color(self):
        """Цвет, чья очередь, или None - всё сделано."""
        k = len(self.done)
        return self.order[k] if k < len(self.order) else None

    def finished(self):
        return self.next_color() is None

    def is_turn(self, cell, color):
        """На клетке cell цвета color сейчас надо отработать."""
        return color is not None and color == self.next_color() and cell not in self.done

    def mark(self, cell):
        self.done.append(cell)

    def targets(self, tile_map):
        """Клетки карты цветов, чья сейчас очередь."""
        return [c for c, color in tile_map.items() if self.is_turn(c, color)]


# Цвет кольца не из набора плиток -> ближайший из набора.
NEAREST_TILE = {"yellow": "red", "green": "blue"}


def tile_of_ring(color, allowed):
    """Цвет плитки для цвета кольца: свой, если он в наборе, иначе ближайший;
    None - кольцо не прочитано."""
    if color in allowed:
        return color
    return NEAREST_TILE.get(color)


def ring_order(reads, allowed=("red", "black", "blue")):
    """Порядок плиток по кольцам когнитивных меток. reads - по списку на
    метку: цвета колец от центра наружу. Каждое кольцо - голосование меток
    (цвет приводится к набору allowed); ничья - решает метка, прочитанная
    первой (у старта, стоя)."""
    order = []
    for i in range(max((len(r) for r in reads), default=0)):
        votes = [tile_of_ring(r[i], allowed) for r in reads if i < len(r)]
        votes = [v for v in votes if v is not None]
        if votes:
            order.append(max(votes, key=lambda v: (votes.count(v), -votes.index(v))))
    return order
