"""Горки: наклон, замер длины, запись в карту."""
from __future__ import annotations

import math

import watch_logic

import config
import state
import esp_link
import maze_map


# Где в карте клетка за гребнем. На один и тот же островок могут вести
# горки РАЗНОЙ длины, а высота у островка одна - значит, и уклон у них
# разный. Островок лежит на своём месте в плане, поэтому клетка за горкой -
# это round(путь в плане от центра клетки у подножия до центра первой
# клетки за гребнем / 300), а длина горки в плане = высота уровня / tan(уклона).
# Колёса здесь не годятся: одна горка в заездах 09.11 намерена от 347 до 1328мм.
# Если дробная часть дальше RAMP_PLAN_WARN_FRAC от целого - островок в
# плане сдвинут почти на полклетки, и две горки могут округлиться по-разному:
# печатаем, а record_ramp() ещё сверяет стены на приезде.
RAMP_PLAN_WARN_FRAC = 0.35
RAMP_LEVEL_CM = 22.0           # высота уровня поля (замер заезда 09.11: 221мм)
# Уклон для клетки - верхняя четверть наклона: на коротком спуске заезд и съезд
# занижают медиану (заезд 09.11 17:26: 25,6° при настоящих ~29° - 3 клетки вместо 2).
RAMP_PLAN_SLOPE_QUANTILE = 0.75
RAMP_GAP_CM = 3.0              # провалы наклона короче этого - дрожь, не конец горки

# ── Поворот, стоя на препятствии ────────────────────────────
# Корпус перекошен (одна сторона на препятствии) - колёса то цепляют, то
# проскальзывают, а гироскоп ловит раскачку корпуса. На ровном полу это не
# так, и обычные настройки поворота там давали тряску:
#  - TURN_DONE_RATE_DPS=20 на раскачке почти не выполняется, "доехал" не
#    засчитывался, и робот подёргивал колёсами до TURN_TIMEOUT_S;
#  - подстройка TURN_FF по проскальзывающим колёсам улетала к TURN_FF_MAX,
#    и следующие повороты (уже на полу) шли с перелётом и качанием у цели;
#  - после поворота ESP держит колёса на 0 dps ещё 3с (BRAKE_HOLD_US), а
#    на перекошенном корпусе вес давит на колёса, и регуляторы моторов
#    раскачиваются на люфте редуктора - тряска на месте все эти 3с.
# Поэтому на перекосе: допуски шире, подстройка TURN_FF заморожена. Удержание
# после остановки теперь не снимается: отпущенный робот скатывался (settle_brake).
TILT_PITCH_DEG = config.OBSTACLE_PITCH_DEG   # |pitch - ровный пол| - "на препятствии"
TILT_ROLL_DEG = 4.0


def tilt_deg():
    """Наибольший перекос корпуса относительно ровного пола, градусы."""
    return max(abs(esp_link.get_pitch() - state.pitch_level), abs(esp_link.get_roll() - state.roll_level))


def is_tilted():
    """Стоим перекошенно - на препятствии, а не на полу."""
    return (abs(esp_link.get_pitch() - state.pitch_level) >= TILT_PITCH_DEG
            or abs(esp_link.get_roll() - state.roll_level) >= TILT_ROLL_DEG)


def record_ramp(cx, cy, d, info, measure=None):
    """drive_cells() проехал горку из (cx,cy) курсом d - заносит её в карту
    и возвращает клетку, где робот теперь стоит.

    Этаж - текущий ±1 (горка меняет этаж ровно на один), клетка - по пути
    в плане (RAMP_PLAN_WARN_FRAC): так две горки разной длины на один
    островок приводят в одни и те же клетки. Если этаж уже разведан, округление
    проекции проверяется замером стен (measure() -> как _measure_cell):
    из клеток на k-1, k, k+1 берётся посещённая, чьи стены совпали.

    Если эта горка уже известна (едем по ней второй раз или обратно) -
    берём записанный конец, а не новый замер: карта не должна расползаться
    от того, что энкодеры в этот раз насчитали иначе."""
    known = state.ramps.get((cx, cy, d))
    if known is not None:
        if info is not None and info["tiles"] != known[2]:
            print(f"    [WARN] горка: сейчас ~{info['tiles']} плит., в карте "
                  f"{known[2]} - верю карте")
        return known[0], known[1]
    k, sign = info["cells"], info["sign"]
    z0 = level_of(cy)
    z = z0 + sign
    floor_known = any(level_of(v[1]) == z for v in state.visited)
    y0 = cy - config.LEVEL_STRIDE * z0      # y внутри этажа

    def dest(kk):
        return cx + config._DX[d] * kk, y0 + config._DY[d] * kk + config.LEVEL_STRIDE * z

    bx, by = dest(k)
    cands = [kk for kk in (k, k - 1, k + 1) if kk >= 1]
    if floor_known and measure is not None and any(dest(kk) in state.visited for kk in cands):
        walls = measure()
        pick = next((kk for kk in cands
                     if dest(kk) in state.visited and maze_map._walls_match(dest(kk), d, walls)), None)
        if pick is None:
            if dest(k) in state.visited:
                print(f"    [WARN] горка: этаж {z} знакомый, но стены в "
                      f"{dest(k)} не сходятся с замером - ставлю как посчитано")
        elif pick != k:
            print(f"    горка: по стенам поправил клетку на {pick - k:+d}")
            bx, by = dest(pick)
        else:
            print(f"    горка: знакомый этаж {z}, стены в {dest(k)} сошлись")

    # Замер стоя у подножия видел спереди "открыто" и завёл соседа - это
    # первая плитка горки, клеткой она не является. То же на другом конце,
    # если этот конец видели раньше с островка. Убираем, иначе BFS считал бы
    # их неразведанными и искал бы к ним путь вечно.
    for ph in ((cx + config._DX[d], cy + config._DY[d]), (bx - config._DX[d], by - config._DY[d])):
        if ph not in state.visited:
            state.maze.pop(ph, None)
    n = info["tiles"]
    state.ramps[(cx, cy, d)] = (bx, by, n, sign)
    state.ramps[(bx, by, config._OPP[d])] = (cx, cy, n, -sign)
    maze_map.open_edge(cx, cy, d)      # по _next_cell это уже ребро до (bx, by)
    print(f"  ⛰ ГОРКА {'вверх' if sign > 0 else 'вниз'}: ~{n} плит., "
          f"({cx},{cy}) → ({bx},{by}), этаж {z}"
          f"{' (знакомый)' if floor_known else ''}, высота {info['rise_cm']:+.0f}см")
    return bx, by


def level_of(cy):
    """Этаж клетки по её cy - см. LEVEL_STRIDE."""
    return watch_logic.level_of(cy, config.LEVEL_STRIDE)


# ══════════════════════════════════════════════════════════════
#  Непрерывный проезд: клетки подряд, без остановки между ними
# ══════════════════════════════════════════════════════════════
def _ramp_measure(log, sign):
    """Длина горки по записи (одометрия_см, pitch) за проезд.

    Подножие и гребень - по пересечению ПОЛОВИНЫ установившегося наклона:
    у подножия нос проходит половину уклона, когда центр робота примерно
    над изломом (передние колёса уже на горке, задние ещё на полу), у
    гребня - так же, и смещения на полбазы с обеих сторон взаимно гасятся.
    Порог по половине реального уклона, а не константой, чтобы горки 25° и
    30° мерились одинаково.

    Возвращает dict: sign, tiles, len_cm, slope, foot_cm, crest_cm,
    start_known (False - поехали уже с горки, длина занижена)."""
    ps = [p * sign for _, p in log]
    ds = [d for d, _ in log]
    on = sorted(p for p in ps if p >= config.RAMP_MAP_DEG)
    slope = on[len(on) // 2] if on else config.RAMP_MAP_DEG
    half = slope * 0.5
    # Участки выше половины уклона. У порога наклон дрожит (вибрация,
    # стыки плиток), и один участок рвётся на куски - склеиваем разрывы
    # короче RAMP_GAP_CM и берём самый длинный: это и есть горка.
    runs = []
    for i, p in enumerate(ps):
        if p < half:
            continue
        if runs and (runs[-1][1] == i - 1 or ds[i] - ds[runs[-1][1]] < RAMP_GAP_CM):
            runs[-1][1] = i
        else:
            runs.append([i, i])
    i_start, i_end = max(runs, key=lambda r: ds[r[1]] - ds[r[0]])
    len_cm = ds[i_end] - ds[i_start]
    cell_cm = config.CELL_MM / 10.0
    tiles = max(1, int(round(len_cm / cell_cm)))   # только для лога
    # Высота - по всему наклонному участку, включая заезд и съезд (там,
    # где pitch выше RAMP_FLAT_DEG), а не по всему проезду: смещение нуля
    # pitch на ровном накопило бы ошибку.
    j0, j1 = i_start, i_end
    while j0 > 0 and ps[j0 - 1] >= config.RAMP_FLAT_DEG:
        j0 -= 1
    while j1 < len(ps) - 1 and ps[j1 + 1] >= config.RAMP_FLAT_DEG:
        j1 += 1
    rise_cm = sign * sum((ds[i] - ds[i - 1]) * math.sin(math.radians(ps[i]))
                         for i in range(j0 + 1, j1 + 1))
    # От центра клетки до подножия и от гребня до центра следующей - по
    # полклетки, между ними горка в плане.
    plan_slope = on[int(len(on) * RAMP_PLAN_SLOPE_QUANTILE)] if on else slope
    plan_cells = RAMP_LEVEL_CM / math.tan(math.radians(plan_slope)) / cell_cm + 1.0
    k = max(1, int(round(plan_cells)))
    print(f"\n    горка: {len_cm*10:.0f}мм по уклону (~{len_cm/cell_cm:.2f} плит.), "
          f"уклон {slope:.1f}°, высота {rise_cm*10:+.0f}мм, "
          f"островок через {plan_cells:.2f} кл. в плане -> {k}")
    if abs(plan_cells - k) > RAMP_PLAN_WARN_FRAC:
        print("    [WARN] горка: островок в плане сдвинут почти на полклетки - "
              "с другой горки клетки могут не сойтись")
    if i_start == 0:
        print("    [WARN] горка: подножие не видно (поехали уже с наклона) - "
              "длина замерена не вся")
    return {"sign": sign, "tiles": tiles, "cells": k, "plan_cells": plan_cells,
            "rise_cm": rise_cm, "len_cm": len_cm, "slope": slope,
            "foot_cm": ds[i_start], "crest_cm": ds[i_end],
            "start_known": i_start > 0}
