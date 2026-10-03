#!/usr/bin/env python3
"""
Детектор жертв для RoboCup Rescue Maze.
cam_idx=0 — правая камера, cam_idx=1 — левая.

Модель: YOLO26n maze_v26n_B.pt (обучена 26.09.2026 на кадрах этих же камер
+ буквы из датасета robocup-ffq6y/maze, CC BY 4.0; 320x320). Классы:
  'phi'       → Φ → damaged  → 2 аптечки
  'psi'       → Ψ → stable   → 1 аптечка
  'omega'     → Ω → unharmed → 0 аптечек
  'cognetive' → когнитивная цель: цвета 5 колец (ring_colors.py), сумма
                2 → damaged, 1 → stable, 0 → unharmed, иначе фейк (не жертва)

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
from dataset_web import STILL_SIZE, Camera  # noqa: E402

# Модель можно переключить без правки кода: MAZE_MODEL=A ~/maze/venv/bin/python nav.py
# B (по умолчанию) — свои кадры + чужие буквы: на проверке mAP50 0.966, как у A,
# но на живых камерах заметно лучше. A — только свои кадры.
YOLO_MODEL_PATH = f"/home/madness/maze/models/maze_v26n_{os.environ.get('MAZE_MODEL', 'B')}.pt"
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
        side = 'правая' if cam_idx == 0 else 'левая'
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
        print(f'[VD] Готов ({side})')

    def _best(self, frame, conf):
        """Лучшая рамка: (класс, уверенность, xyxy в координатах frame) или None."""
        small = frame if frame.shape[1] == IMGSZ else cv2.resize(
            frame, (IMGSZ, IMGSZ), interpolation=cv2.INTER_AREA)
        boxes = self.model.predict(small, imgsz=IMGSZ, conf=conf,
                                   save=False, verbose=False)[0].boxes
        if len(boxes) == 0:
            return None
        b = max(boxes, key=lambda b: float(b.conf[0]))
        k = frame.shape[1] / IMGSZ
        return (str(self.model.names[int(b.cls[0])]), float(b.conf[0]),
                [float(v) * k for v in b.xyxy[0].tolist()])

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
        key = (cell_x, cell_y, wall_dir)
        if key in self.scanned:
            return None
        self.scanned.add(key)

        # ── Проход 1: обычный кадр — по нему решаем, что за метка ──
        # (модель обучена на таких кадрах; на снимке 500 мс она может,
        # например, принять крупную Ω за когнитивную цель)
        self.cam.latest()                  # сброс кадра, снятого ещё на ходу
        frame = self.cam.latest()
        found = self._best(frame, CONF_FAST)
        if found is None:
            return None
        cls_name, conf, box = found
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
            still_found = self._best(still, CONF)
            if still_found and _iou(still_found[2], xyxy) > 0.3:
                xyxy = still_found[2]          # та же метка — рамка точнее
            shot = (f'{still.shape[1]}x{still.shape[0]} {md.get("ExposureTime", 0) / 1000:.0f}мс'
                    f' x{md.get("AnalogueGain", 1):.1f}')
        else:
            still, xyxy, shot = frame, box, 'обычный кадр'
        dt = time.time() - t0
        if conf < CONF:
            # на обычном кадре неуверенно — тогда решает снимок
            if still_found is None:
                print(f'[VD] клетка=({cell_x},{cell_y}): что-то похожее ({cls_name} {conf:.2f}), '
                      f'но не подтвердилось ({dt:.1f}с)')
                return None
            cls_name, conf, xyxy = still_found
        elif still_found and still_found[0] != cls_name:
            shot += f', на снимке {still_found[0]} {still_found[1]:.2f}'

        rings = checksum = None
        shown = still.copy()
        if cls_name == COG_CLASS:
            ring_info, _, _ = rc.read_rings(still, xyxy, self.ring_cfg, draw_on=shown)
            rings = [r['color'] for r in ring_info]
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

        if victim_type is None:
            print(f'[VD] ○ когнитивная цель, кольца не прочитались {rings}  '
                  f'клетка=({cell_x},{cell_y})  conf={conf:.2f}  [{shot}, {dt:.1f}с]')
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
        }
        self.log.append(result)
        tag = '★ ЖЕРТВА' if blink else '○ МЕТКА (без мигания)'
        rings_str = (' кольца=[' + ', '.join(rings) + ']') if rings else ''
        print(f'[VD] {tag} {victim_type.upper()} ({label})  '
              f'клетка=({cell_x},{cell_y})  класс={cls_name}  conf={conf:.2f}{rings_str}'
              f'  [{shot}, {dt:.1f}с]')
        return result

    def close(self):
        self.cam.close()
