"""Цвет плитки под роботом: белая, синяя, чёрная, чекпоинт; калибровка белого."""
from __future__ import annotations

import time

import runplan
import tile_classify
import tile_rules

import config
import state
import timing
import esp_link


# ЗАМЕРЕНО на этом роботе (color.py, все пять случаев подряд):
#
#     поверхность   C     r/c    g/c    b/c   (b-r)/c
#     белая        335   0.245  0.358  0.403   +0.158
#     чёрная        26   0.269  0.346  0.346   +0.077
#     красная       92   0.424  0.207  0.402   -0.022
#     синяя         87   0.148  0.310  0.557   +0.409
#     ВОЗДУХ       134   0.504  0.317  0.201   -0.302
#
# Порог синей был 0.32 от старых замеров: до белой 0.25, до синей всего
# 0.09 - несимметрично. 0.28 стоит посередине, по 0.12 в обе стороны.
BLUE_RATIO_THRESH = 0.28   # (b-r)/c выше этого - синяя

# Порог чёрной. Изначально был долей от белой (C < _c_white*FRACTION) - идея
# в том, что абсолютная яркость плывёт с освещением зала, а доля - нет. На
# практике оказалось наоборот: два реальных замера на этом роботе дали
# ПОЧТИ ОДИНАКОВУЮ чёрную (C=156 и C=157) при совсем разной калибровке белой
# (313 и ~205) - чёрная плитка здесь не масштабируется вниз вместе с общим
# освещением так же, как белая (видимо, засветка датчика собственным
# отражением/подсветкой не падает пропорционально). Доля 0.55 от белой=205
# даёт порог 113 - НИЖЕ самой чёрной (157), то есть в тусклом зале чёрную
# гарантированно бы пропустили. Поэтому порог теперь в первую очередь
# АБСОЛЮТНЫЙ (с запасом над наблюдавшимися 156-157), а доля от белой -
# только подстраховка на случай совсем тёмного зала, где даже белая ниже
# этого абсолютного потолка (тогда берём меньшее из двух).
# Замеры на ЭТОМ роботе в текущем зале (усреднение 3с, color.py):
#     белая  C=335  чёрная C=27  красная C=97
# Доля чёрной от белой здесь 0.081, и прежние 0.85 к этому не имеют никакого
# отношения - при белой 335 они дают порог 285, то есть под "чёрную" попадает
# и красная плитка (97), и почти наверняка синяя. Абсолютный потолок 180 не
# лучше: он тоже выше красной.
#
# Порог теперь стоит геометрически посередине между чёрной и КРАСНОЙ - она
# самая тусклая из "не чёрных", а не белая: 27*97 под корнем это 51, что и
# даёт долю 0.15 от белой. Запас вниз 1.9x, вверх 1.9x.
#
# Абсолютный потолок оставлен как min(): он связывает только в очень ярком
# зале (при белой выше ~1200), где доля от белой уехала бы слишком высоко.
BLACK_ABS_MAX_C = 180
BLACK_FRACTION = 0.15

# Когда передние колёса заезжают на препятствие, нос поднимается и датчик
# цвета отдаляется от пола. Малый C в этот момент — не чёрная плитка.
# После возврата в горизонт даём подвеске/датчику короткое время опуститься.
PITCH_COLOR_GUARD_DEG = 3.0
COLOR_RECOVER_S = 0.35

BLUE_STOP_S = 6.0          # стоянка на синей плитке

# ── Чекпоинт: красная плитка ─────────────────────────────────
# Красный проверяется ПЕРЕД чёрным, и это не косметика: красная плитка
# тусклая, её C вполне может оказаться ниже BLACK_ABS_MAX_C, и тогда робот
# принял бы чекпоинт за запретную клетку и сдал назад.
#
# Признак взят по ЗАМЕРУ настоящей плитки, и он оказался не тем, что
# напрашивался. Ожидалось, что красная выделяется по (r-b)/c, как синяя по
# (b-r)/c. На деле у этой плитки синий канал почти равен красному (R=43,
# B=39), и (r-b)/c даёт жалкие +0.041 против -0.074 у чёрной - разделять по
# нему нельзя. Зато красная резко ГАСИТ ЗЕЛЁНЫЙ: g/c=0.206 против 0.355 у
# белой и 0.370 у чёрной. Поэтому признак - (r-g)/c:
#
#     красная +0.237 | чёрная -0.074 | белая -0.116
#
# Порог 0.10 оставляет 0.137 до красной и 0.174 до ближайшей чужой.
RED_RG_THRESH = 0.10
# Одной доли мало. У чёрной C=27, и при таких крошечных отсчётах один-два
# кванта шума двигают (r-g)/c на 0.07 - то есть шум способен закинуть чёрную
# за порог. Ошибка тут дорогая: чёрную (запретную) приняли бы за чекпоинт.
# Поэтому второе условие - яркость не ниже доли от белой. Доля, а не число:
# при смене освещения C всех плиток едет вместе. 0.15 от белой это 50 при
# белой 335 - чёрная (27) не дотягивает вдвое, красная (97) вдвое выше.
RED_MIN_C_FRACTION = 0.15


def black_threshold():
    """Порог 'C ниже этого - чёрная': см. BLACK_ABS_MAX_C. None, пока нет
    калибровки белой (calibrate_white() ещё не отработала)."""
    if state.c_white is None:
        return None
    return min(BLACK_ABS_MAX_C, state.c_white * BLACK_FRACTION)


def tile_color():
    """Цвет плитки под роботом по сырым каналам APDS9960."""
    state.tile_last = _tile_color()
    return state.tile_last


def _tile_color():
    if not state.USE_COLOR_SENSOR:
        return config.TILE_UNKNOWN
    now = time.monotonic()
    if abs(esp_link.get_pitch() - state.pitch_level) > PITCH_COLOR_GUARD_DEG:
        state.color_ignore_until = now + COLOR_RECOVER_S
        return config.TILE_UNKNOWN
    if now < state.color_ignore_until:
        return config.TILE_UNKNOWN
    r, g, b, c = esp_link.get_rgbc()
    if c < config.TILE_MIN_C:
        return config.TILE_UNKNOWN
    if state.tile_samples:
        # "air" (робот на весу) и "не знаю" - одинаково не плитка.
        v = tile_classify.classify(state.tile_samples, r, g, b, c)[0]
        return v if v in (config.TILE_WHITE, config.TILE_BLACK, config.TILE_BLUE, config.TILE_RED, config.TILE_SILVER) \
            else config.TILE_UNKNOWN
    # Перед всем остальным - не плитка ли это вообще. На весу датчик видит
    # тёплый свет комнаты и БЕЗ этой проверки классифицируется как КРАСНАЯ
    # (замерено (r-g)/c=+0.164 при пороге красного 0.10), то есть поднятый
    # робот записывал бы себе фальшивый чекпоинт. См. LIFT_AIR_RB.
    if (r - b) / c > config.LIFT_AIR_RB:
        return config.TILE_UNKNOWN
    # Красная - ДО чёрной, и это не перестраховка: замеренная красная даёт
    # C=97 при пороге чёрной 51, но в зале потемнее она подойдёт к нему
    # вплотную. См. RED_* про то, почему признак по зелёному, а не по синему.
    if ((r - g) / c > RED_RG_THRESH
            and (not state.c_white or c > state.c_white * RED_MIN_C_FRACTION)):
        return config.TILE_RED
    if (b - r) / c > BLUE_RATIO_THRESH:
        return config.TILE_BLUE
    thr = black_threshold()
    if thr is not None and c < thr:
        return config.TILE_BLACK
    return config.TILE_WHITE


@timing._timed("синяя")
def blue_stop(cell):
    """Стоянка на синей плитке - КАЖДЫЙ раз, когда робот на ней оказывается,
    а не только при первом распознавании: если клетка хоть раз была
    распознана как синяя, это помнится навсегда (_blue_cells), и стоянка
    отрабатывает заново на любом повторном визите - в том числе на обратном
    пути (return_to_start), а не только во время разведки.

    _blue_cells используется вызывающим (arrive()/return_to_start()) ещё и
    как подстраховка живого распознавания: если разъезженная плитка на
    повторном проезде вдруг не читается датчиком цвета как синяя (шум,
    угол, засветка), но клетка уже ЕСТЬ в _blue_cells - стоянка всё равно
    произойдёт, потому что однажды подтверждённому факту доверяем больше,
    чем одному живому замеру.

    Фиксацию моторов переподтверждаем: удержание на ESP само отпускает через
    3 секунды (BRAKE_HOLD_US), а стоять надо дольше - на наклонном поле робот
    иначе потихоньку поедет.
    """
    state.blue_cells.add(cell)
    print(f"  ■ СИНЯЯ {cell}: стоим {BLUE_STOP_S:.0f}с")
    end = time.time() + BLUE_STOP_S
    while time.time() < end:
        if esp_link.interrupted():
            return
        esp_link.brake()
        time.sleep(min(0.5, max(0.0, end - time.time())))


def load_tile_calib():
    """Образцы плиток из tile_calib.py -> _tile_samples. Вызывать на старте,
    до calibrate_white(). Без белой и чёрной калибровка не берётся."""
    s = tile_classify.load()
    if not {"white", "black"} <= s.keys():
        state.tile_samples = {}
        print(f"  [WARN] нет калибровки цвета ({tile_classify.CALIB_PATH}, нужны "
              f"хотя бы белая и чёрная) - старые пороги, белая по стартовой "
              f"клетке, ЧЕКПОИНТЫ НЕ РАСПОЗНАЮТСЯ. Запусти tile_calib.py.")
        return
    state.tile_samples = s
    print("  калибровка цвета: " + ", ".join(f"{k}×{len(v)}" for k, v in s.items()))
    if state.CHECKPOINT_TILE not in s:
        print(f"  [WARN] нет образца {state.CHECKPOINT_TILE} (чекпоинт) - чекпоинты "
              f"не распознаются")
    for a, b, d in tile_classify.separation(s):
        if d < tile_classify.SEP_WARN:
            print(f"  [WARN] {a} и {b} почти неразличимы ({d:.1f}) - переснять")


def calibrate_white(seconds=0.6):
    """Замер стоя на старте: pitch на ровном полу и, если калибровки по
    образцам нет, прозрачный канал белой.

    С калибровкой (load_tile_calib) цвет стартовой клетки НЕ берётся: старт
    может быть на чекпоинте, и зеркальная записалась бы белой. Без неё -
    как раньше: чёрную от белой по долям каналов не отличить, а абсолютная
    яркость зависит от освещения зала, так что опора - стартовая клетка.
    """
    acc = pitch_acc = roll_acc = n = 0
    end = time.time() + seconds
    while time.time() < end:
        acc += esp_link.get_rgbc()[3]
        pitch_acc += esp_link.get_pitch()
        roll_acc += esp_link.get_roll()
        n += 1
        time.sleep(0.02)
    state.pitch_level = pitch_acc / max(n, 1)
    state.roll_level = roll_acc / max(n, 1)
    state.color_ignore_until = 0.0
    if not state.USE_COLOR_SENSOR:
        print(f"  pitch0={state.pitch_level:.1f}° roll0={state.roll_level:.1f}°, датчик цвета отключён")
        return
    if state.tile_samples:
        w = state.tile_samples["white"]
        state.c_white = sum(x["c"] for x in w) / len(w)
        print(f"  pitch0={state.pitch_level:.1f}° roll0={state.roll_level:.1f}°, белая из калибровки C={state.c_white:.0f}, "
              f"под роботом сейчас: {tile_color()}")
        return
    state.c_white = acc / max(n, 1)
    print(f"  калибровка белой: C={state.c_white:.0f}  "
          f"(порог чёрной {black_threshold():.0f}), "
          f"pitch0={state.pitch_level:.1f}° roll0={state.roll_level:.1f}°")
    if state.c_white < 150:
        print("  [WARN] маловато света - чёрную от белой различит ненадёжно")


def check_floor_while_driving(trip):
    """Цвет плитки под роботом: роль по таблице tile_rules - стена (чёрная)
    - назад, стоп (синяя) - встать в центре, чекпоинт."""
    # ── Чёрная плитка: заезжать нельзя ────────────────────
    # Реагируем на первое же показание, без подтверждения вторым: датчик
    # обновляется ~10Гц, и одно лишнее подтверждение стоит ещё ~5см хода
    # вглубь чёрного. Запас по порогу большой (C=100 против порога ~210),
    # так что ложные срабатывания маловероятны.
    tile = tile_color()
    if tile_rules.role(tile) == tile_rules.WALL:
        trip.reason = "black"
        state.last_wall_tile = tile
        # Назад - в центр клетки перед чёрной. Без остановок чёрная бывает
        # и над клеткой, которую план уже засчитал пройденной.
        trip.cells, trip.black_back_cm = runplan.black_retreat(trip.traveled_cm, trip.boundary_cm,
                                                     trip.cells, trip.cell_cm)
        print(f"\n    ЧЁРНАЯ ({tile}, C={esp_link.get_rgbc()[3]}) на {trip.traveled_cm*10:.0f}мм - "
              f"откат {trip.black_back_cm*10:.0f}мм")
        return "stop"
    if trip.plan is not None and tile != config.TILE_UNKNOWN:
        # Над какой клеткой датчик: N - уже за её границей (+15мм).
        k_under = trip.cells + 1 if trip.traveled_cm >= trip.boundary_cm - trip.cell_cm / 2.0 + 1.5 else trip.cells
        trip.tiles.setdefault(k_under, set()).add(tile)
        tile_rules.remember(trip.plan.cell(k_under), tile)
        if (tile_rules.role(tile) == tile_rules.STOP and k_under == trip.cells >= 1 and not trip.stopping
                and trip.traveled_cm < trip.boundary_cm - trip.cell_cm - 3.0):
            # Синюю заметили уже после решения "прямо": её центр ещё
            # впереди - встаём там (правила: стоянка на синей).
            trip.boundary_cm -= trip.cell_cm
            trip.stopping, trip.pending, trip.reason = True, False, "stop"
            print(f"\n    → кл.{trip.cells}: синяя после решения - стоп в её центре")
        if tile_rules.role(tile) == tile_rules.CHECKPOINT and 1 <= k_under <= trip.cells and k_under not in trip.cp_done:
            trip.cp_done.add(k_under)
            trip.plan.checkpoint(k_under)
