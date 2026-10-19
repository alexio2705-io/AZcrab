#!/usr/bin/env python3
"""
Детектор жертв для RoboCup Rescue Maze.
Какой cam_idx на какой стороне - watch_logic.CAM_IDX.

Модель: YOLO26n maze_v26n_C3.pt (B + ложные метки и нарисованные жертвы,
docs/model_C_report.md; 320x320). Классы:
  'phi'       → Φ → damaged  → 2 аптечки
  'psi'       → Ψ → stable   → 1 аптечка
  'omega'     → Ω → unharmed → 0 аптечек
  'cognetive' → когнитивная цель: цвета 5 колец (ring_colors.py), сумма
                2 → damaged, 1 → stable, 0 → unharmed, иначе фейк (не жертва)
Модели C/C3 знают ещё ложные метки 'H', 'S', 'U', 'letter' - они не жертвы
и отбрасываются в _best (watch_logic.victim_boxes).

Скан стены: быстрый проход по обычному кадру 320x320; если что-то есть —
отдельный снимок 960x960 с выдержкой 500 мс (та же область сенсора, втрое
детальнее, меньше шума) и чтение метки по нему. Всё как на странице
victim_web.py, где это отлаживалось. Камеры — с настройками
~/maze/camera_settings.json, пороги колец — ~/maze/ring_colors.json.
"""

import os
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import cv2  # noqa: E402

import ring_colors as rc  # noqa: E402
import watch_logic as wl  # noqa: E402
from dataset_web import STILL_SIZE, Camera  # noqa: E402

# Модель можно переключить без правки кода: MAZE_MODEL=B ~/maze/venv/bin/python nav.py
# C3 (по умолчанию) — B + приманки (H/S/U/прочие буквы) и нарисованные жертвы:
# приманки за жертву 0 из 1244 фото-приманок (у B 18%), свои кадры как у B.
# C — то же без нарисованных жертв: чистую печать иногда зовёт приманкой.
# B — только Φ/Ψ/Ω/цели, A — только свои кадры.
YOLO_MODEL_PATH = f"/home/madness/maze/models/maze_v26n_{os.environ.get('MAZE_MODEL', 'C3')}.pt"
SAVE_DIR        = '/home/madness/maze/detections'

IMGSZ        = 320
CONF_FAST    = 0.25     # проход 1: есть ли вообще что-то (меньше пропусков)
CONF         = 0.45     # проход 2: чем считать метку
# Выдержка снимка — по автоэкспозиции обычного кадра (та же яркость, но при
# усилении 1), не длиннее этого. С подсветкой выходит короткой, в темноте — до
# 500 мс. Фиксированные 500 мс под подсветкой пересвечивали цель.
STILL_MAX_EXP_US = 500_000
# Метка на соседней стене (робот в центре клетки) крупная; мельче этого
# (сторона рамки на кадре 320) — метка на дальней стене, её не репортим:
# за идентификацию дальше 15 см от жертвы штраф.
MIN_BOX_PX   = 40

# Маппинг имён классов YOLO → тип жертвы (буквенные метки)
CLASS_TO_VICTIM = {
    'phi':   'damaged',    # Φ
    'psi':   'stable',     # Ψ
    'omega': 'unharmed',   # Ω
}
CLASS_LABEL = {
    'damaged':  'Φ',
    'stable':   'Ψ',
    'unharmed': 'Ω',
}
COG_CLASS = 'cognetive'
SUM_TO_TYPE = {2: 'damaged', 1: 'stable', 0: 'unharmed'}   # иначе — фейк
RING_SCORE = rc.COLOR_SCORE                                # баллы колец (для nav.py)


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0


class VictimDetector:
    """Детектор жертв на одной камере: YOLO + чтение колец когнитивной цели."""

    def __init__(self, cam_idx: int = 0):
        from ultralytics import YOLO

        os.makedirs(SAVE_DIR, exist_ok=True)
        side = wl.CAM_NAMES.get(cam_idx, '?')
        self.cam_idx = cam_idx

        print('[VD] Загрузка YOLO...')
        self.model = YOLO(YOLO_MODEL_PATH)

        print(f'[VD] Инициализация камеры ({side}, idx={cam_idx})...')
        # Без фонового потока: кадр снимается только в scan()
        self.cam = Camera(cam_idx, stream=False)
        self.ring_cfg = rc.load_config()
        time.sleep(0.8)   # прогрев экспозиции

        self.scanned = set()   # (клетка, стена) уже просканированы
        self.log = []
        # Последнее read_wall(): когнитивная цель в кадре, а кольца не
        # прочитались - то, что о ней известно (type 'unread'), иначе None.
        self.last_unread = None
        # Все такие чтения подряд (nav.py чистит перед серией и решает по
        # самому надёжному) и что нашёл проход 1 последнего чтения.
        self.unread_hist = []
        self.last_seen = None
        print(f'[VD] Готов ({side})')

    def _best(self, frame, conf):
        """Лучшая рамка жертвы: (класс, уверенность, xyxy в координатах frame)
        или None. Ложные метки (H/S/U/letter у модели C) отбрасываются вместе
        с рамками жертв под ними - см. watch_logic.victim_boxes."""
        small = frame if frame.shape[1] == IMGSZ else cv2.resize(
            frame, (IMGSZ, IMGSZ), interpolation=cv2.INTER_AREA)
        boxes = self.model.predict(small, imgsz=IMGSZ, conf=conf,
                                   save=False, verbose=False)[0].boxes
        found = wl.victim_boxes([(str(self.model.names[int(b.cls[0])]), float(b.conf[0]),
                                  b.xyxy[0].tolist()) for b in boxes])
        if not found:
            return None
        cls_name, c, xyxy = max(found, key=lambda b: b[1])
        k = frame.shape[1] / IMGSZ
        return cls_name, c, [float(v) * k for v in xyxy]

    def scan(self, cell_x: int, cell_y: int, wall_dir=None):
        """
        Проход 1: свежий обычный кадр 320x320, conf=0.25. Ничего нет → None.
          Класс берём отсюда, если уверенность >= 0.45.
        Проход 2: снимок 960x960 с выдержкой 500 мс той же области — по нему
          читаются кольца и сохраняется картинка. Класс со снимка берётся,
          только если на обычном кадре метка была неуверенной (0.25..0.45).

        wall_dir — абсолютное направление стены (0..3), на которую смотрит
        камера. Дедупликация по (клетка, стена): после поворота робот видит
        ДРУГУЮ стену той же клетки — её тоже надо просканировать.
        """
        self.last_unread = None
        key = (cell_x, cell_y, wall_dir)
        if key in self.scanned:
            return None
        self.scanned.add(key)
        return self.read_wall(cell_x, cell_y, wall_dir)

    def read_wall(self, cell_x: int, cell_y: int, wall_dir=None, expect=False):
        """То же чтение, что в scan(), но без дедупа по (клетка, стена): по
        сигналу наблюдателя на ходу стену исходной клетки читаем ещё раз -
        стоя в центре её уже смотрели, но метка была вне кадра. В результате
        дополнительно 'box_y' - (y1, y2) рамки в долях высоты кадра, по нему
        nav.py считает, где метка вдоль стены (дедуп по положению).

        expect=True - метку здесь уже видели (на ходу, с поворота, этой же
        камерой минуту назад): неуверенную рамку подтверждаем с порогом
        CONF_FAST, а не CONF. Иначе стоя напротив цели с conf 0.31 робот
        считал «не подтвердилось» и ездил вдоль стены."""
        # ── Проход 1: обычный кадр — по нему решаем, что за метка ──
        # (модель обучена на таких кадрах; на снимке 500 мс она может,
        # например, принять крупную Ω за когнитивную цель)
        self.last_unread = None
        self.last_seen = None
        self.cam.latest()                  # сброс кадра, снятого ещё на ходу
        frame = self.cam.latest()
        found = self._best(frame, CONF_FAST)
        if found is None:
            return None
        cls_name, conf, box = found
        self.last_seen = (cls_name, conf)
        side_px = max(box[2] - box[0], box[3] - box[1])
        if side_px < MIN_BOX_PX:
            print(f'[VD] клетка=({cell_x},{cell_y}): {cls_name} {conf:.2f} слишком мелкая '
                  f'({side_px:.0f}px < {MIN_BOX_PX}) — дальняя стена, пропускаю')
            return None

        # ── Проход 2: снимок с длинной выдержкой — для колец/картинки ──
        # Уверенной букве (Φ/Ψ/Ω, conf >= CONF) снимок не нужен вовсе: класс в
        # этом случае всё равно берётся с обычного кадра, а колец у буквы нет.
        # Снимок - это перенастройка камеры, 500мс выдержки и возврат режима,
        # то есть 1.5-2с стоянки ради одной лишь картинки в лог.
        #
        # Возврат камеры в обычный режим после снимка идёт в фоне (settle_async):
        # следующий кадр с этой камеры понадобится не раньше следующей клетки,
        # а latest() до тех пор сам подождёт на cap_lock.
        t0 = time.time()
        if conf >= CONF and cls_name in CLASS_TO_VICTIM:
            still, md = None, {}
        else:
            still, md = self.cam.capture_long(None, None, size=STILL_SIZE, settle_async=True,
                                              auto_max_us=STILL_MAX_EXP_US)
        still_found = None
        if still is not None:
            k = still.shape[1] / frame.shape[1]
            xyxy = [v * k for v in box]
            still_found = self._best(still, CONF_FAST if expect else CONF)
            if still_found and _iou(still_found[2], xyxy) > 0.3:
                xyxy = still_found[2]          # та же метка — рамка точнее
            shot = (f'{still.shape[1]}x{still.shape[0]} {md.get("ExposureTime", 0) / 1000:.0f}мс'
                    f' x{md.get("AnalogueGain", 1):.1f}')
        else:
            still, xyxy, shot = frame, box, 'обычный кадр'
        dt = time.time() - t0
        if conf < CONF:
            # на обычном кадре неуверенно — тогда решает снимок
            if still_found is not None:
                cls_name, conf, xyxy = still_found
            elif not expect:
                print(f'[VD] клетка=({cell_x},{cell_y}): что-то похожее ({cls_name} {conf:.2f}), '
                      f'но не подтвердилось ({dt:.1f}с)')
                return None
            # expect: метку здесь уже видели - верим проходу 1
        elif still_found and still_found[0] != cls_name:
            shot += f', на снимке {still_found[0]} {still_found[1]:.2f}'

        rings = checksum = None
        shown = still.copy()
        if cls_name == COG_CLASS:
            ring_info, _, _ = rc.read_rings(still, xyxy, self.ring_cfg, draw_on=shown)
            rings = [r['color'] for r in ring_info]
            ring_ok = [bool(r['reliable']) for r in ring_info]
            ring_agree = [r['agree'] for r in ring_info]
            checksum = rc.checksum(ring_info)
            victim_type = None if checksum is None else SUM_TO_TYPE.get(checksum, 'fake')
            label = '?' if checksum is None else f'{checksum:+d}'
        elif cls_name in CLASS_TO_VICTIM:
            victim_type = CLASS_TO_VICTIM[cls_name]
            label = CLASS_LABEL[victim_type]
        else:
            print(f'[VD] Неизвестный класс: {cls_name} ({conf:.2f})')
            return None

        # Кадр сохраняем всегда — и когда кольца не прочитались (для отладки)
        fname = (f'v_{cell_x}_{cell_y}_{victim_type or "unread"}_'
                 f'{time.strftime("%H-%M-%S")}.jpg')
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.rectangle(shown, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(shown, f'{cls_name}={victim_type}({conf:.2f})', (x1, max(16, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6 * shown.shape[1] / IMGSZ / 2, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(SAVE_DIR, fname), shown)

        box_y = (round(xyxy[1] / still.shape[0], 4), round(xyxy[3] / still.shape[0], 4))
        if victim_type is None:
            print(f'[VD] ○ когнитивная цель, кольца не прочитались {rings}  '
                  f'клетка=({cell_x},{cell_y})  conf={conf:.2f}  [{shot}, {dt:.1f}с]')
            self.last_unread = {'type': 'unread', 'label': '?', 'image': fname,
                                'cell': (cell_x, cell_y), 'class': cls_name,
                                'conf': round(conf, 3), 'blink': False, 'rings': rings,
                                'ring_ok': ring_ok, 'ring_agree': ring_agree,
                                'checksum': None, 'box_y': box_y}
            self.unread_hist.append(self.last_unread)
            return None

        # Мигаем только для настоящих жертв; фейк фиксируем без мигания.
        blink = victim_type in ('damaged', 'stable', 'unharmed')
        result = {
            'type':     victim_type,
            'label':    label,
            'image':    fname,
            'cell':     (cell_x, cell_y),
            'class':    cls_name,
            'conf':     round(conf, 3),
            'blink':    blink,
            'rings':    rings,      # цвета колец от центра наружу (когнитивная) или None
            'checksum': checksum,
            'box_y':    box_y,
        }
        self.log.append(result)
        tag = '★ ЖЕРТВА' if blink else '○ МЕТКА (без мигания)'
        rings_str = (' кольца=[' + ', '.join(rings) + ']') if rings else ''
        print(f'[VD] {tag} {victim_type.upper()} ({label})  '
              f'клетка=({cell_x},{cell_y})  класс={cls_name}  conf={conf:.2f}{rings_str}'
              f'  [{shot}, {dt:.1f}с]')
        return result

    def best_fast(self):
        """Стоя, после остановки по сигналу наблюдателя: свежий кадр 320,
        лучшая рамка conf >= CONF_FAST не мельче MIN_BOX_PX. Возвращает
        (класс, уверенность, (y1, y2) в долях высоты кадра) или None."""
        self.cam.latest()                  # сброс кадра, снятого ещё на ходу
        frame = self.cam.latest()
        found = self._best(frame, CONF_FAST)
        if found is None:
            return None
        cls_name, conf, box = found
        if max(box[2] - box[0], box[3] - box[1]) < MIN_BOX_PX:
            return None
        h = frame.shape[0]
        return cls_name, conf, (box[1] / h, box[3] / h)

    def close(self):
        self.cam.close()
