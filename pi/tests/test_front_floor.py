"""front_floor: геометрия пола и цвет следующей клетки (нужны numpy и cv2)."""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
try:
    import cv2
    import numpy as np

    import front_floor as ff
except ImportError:
    np = None

# «Камера»: пол (мм, x вперёд, y влево) -> пиксели кадра 720x970. Ближнее
# внизу, дальнее у горизонта, левое слева.
G = None
if np is not None:
    G = cv2.getPerspectiveTransform(
        np.float32([[150, 150], [150, -150], [1050, -150], [1050, 150]]),
        np.float32([[60, 900], [660, 900], [470, 450], [250, 450]]))


def floor_to_img(pts):
    return cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), G).reshape(-1, 2)


def scene(colors_by_cell, size=(970, 720)):
    """Кадр: пол белый, клетки впереди (1, 2, ...) - заданного цвета (BGR)."""
    img = np.full((size[0], size[1], 3), (210, 215, 220), np.uint8)
    for ahead, bgr in colors_by_cell.items():
        x0, x1 = 300 * ahead - 150, 300 * ahead + 150
        poly = floor_to_img([[x1, 150], [x1, -150], [x0, -150], [x0, 150]])
        cv2.fillPoly(img, [np.round(poly).astype(np.int32)], bgr)
    return img


@unittest.skipIf(np is None, 'нужен numpy')
class TestGeometry(unittest.TestCase):
    def test_calibrate_recovers_mapping(self):
        quad = floor_to_img([[450, 150], [450, -150], [150, -150], [150, 150]])
        h = ff.calibrate_h(quad.tolist(), 150)
        poly = ff.to_image(h, ff.cell_polygon())
        want = floor_to_img(ff.cell_polygon())
        self.assertLess(np.abs(poly - want).max(), 0.5)

    def test_cell_polygon_pose(self):
        near = min(p[0] for p in ff.cell_polygon((100.0, 0.0, 0.0)))
        self.assertAlmostEqual(near, 150 + ff.MARGIN_MM - 100)
        # робот повёрнут влево на 90°: клетка «вперёд по сетке» у него справа
        ys = [p[1] for p in ff.cell_polygon((0.0, 0.0, math.pi / 2))]
        self.assertLess(max(ys), -100)

    def test_model_matches_wall_measurement(self):
        # 18.10.2026: стена по лидару x = 509 + 0.47*y (мм от центра робота),
        # её стык со столом на кадре - v=520 у левого края, v=575 у правого
        for u_want, v_want in ((0, 520), (720, 575)):
            lo, hi = -600.0, 600.0
            for _ in range(60):
                y = (lo + hi) / 2
                u, v = ff.model_point(509 + 0.47 * y, y)
                lo, hi = (y, hi) if u > u_want else (lo, y)
            self.assertAlmostEqual(v, v_want, delta=8)

    def test_model_h_reads_far_half_of_next_cell(self):
        r = ff.read_cell(scene({}, size=(973, 720)), {'H': None, 'samples': {}})
        self.assertGreaterEqual(r['visible'], ff.MIN_VISIBLE)

    def test_find_quad(self):
        img = scene({1: (20, 20, 20)})
        quad, err = ff.find_quad(img, 'black')
        self.assertIsNone(err)
        want = floor_to_img([[450, 150], [450, -150], [150, -150], [150, 150]])
        self.assertLess(np.abs(np.float32(quad) - want).max(), 6)   # ~1% плитки

    def test_find_quad_rejects_cut_tile(self):
        img = scene({})
        poly = floor_to_img([[450, 150], [450, -150], [0, -150], [0, 150]])
        cv2.fillPoly(img, [np.round(poly).astype(np.int32)], (20, 20, 20))   # ниже края кадра
        quad, err = ff.find_quad(img, 'black')
        self.assertIsNone(quad)
        self.assertIn('края', err)


@unittest.skipIf(np is None, 'нужен numpy')
class TestColor(unittest.TestCase):
    def setUp(self):
        quad = floor_to_img([[450, 150], [450, -150], [150, -150], [150, 150]])
        self.cal = {'H': ff.calibrate_h(quad.tolist(), 150).tolist(), 'samples': {}}
        for color, bgr in (('white', (210, 215, 220)), ('black', (25, 25, 25)),
                           ('blue', (220, 140, 30))):
            self.cal['samples'][color] = [ff.sample_cell(scene({1: bgr}), self.cal)]

    def test_reads_next_cell(self):
        self.assertEqual(ff.read_cell(scene({1: (25, 25, 25)}), self.cal)['color'], 'black')
        self.assertEqual(ff.read_cell(scene({1: (220, 140, 30)}), self.cal)['color'], 'blue')
        self.assertEqual(ff.read_cell(scene({}), self.cal)['color'], 'white')

    def test_reads_from_off_center_pose(self):
        # робот на 100 мм впереди центра и на 30 мм левее: клетка та же
        img = scene({1: (25, 25, 25), 2: (220, 140, 30)})
        self.assertEqual(ff.read_cell(img, self.cal, (100.0, 30.0, 0.0))['color'], 'black')
        # а клетка через одну - синяя
        self.assertEqual(ff.read_cell(img, self.cal, (100.0, 30.0, 0.0), ahead=2)['color'], 'blue')

    def test_thresholds_without_samples(self):
        cal = {'H': self.cal['H'], 'samples': {}}
        self.assertEqual(ff.read_cell(scene({}), cal)['color'], 'white')
        self.assertEqual(ff.read_cell(scene({1: (25, 25, 25)}), cal)['color'], 'black')
        self.assertEqual(ff.read_cell(scene({1: (243, 160, 66)}), cal)['color'], 'blue')
        wood = scene({1: (102, 150, 188)})           # стол: H19 S~100 - белый пол
        self.assertEqual(ff.read_cell(wood, cal)['color'], 'white')

    def test_always_decides_out_of_view(self):
        # клетка «вперёд по сетке» за краем кадра (робот смотрит вбок)
        r = ff.read_cell(scene({}), self.cal, (0.0, 0.0, math.pi / 2))
        self.assertEqual((r['color'], r['conf']), ('white', 0.0))

    def test_mixed_cell_takes_majority(self):
        img = scene({})
        poly = floor_to_img([[450, 150], [450, -60], [150, -60], [150, 150]])
        cv2.fillPoly(img, [np.round(poly).astype(np.int32)], (25, 25, 25))
        r = ff.read_cell(img, self.cal)
        self.assertEqual(r['color'], 'black')
        self.assertLess(r['conf'], 0.9)

    def test_on_ramp_next_is_plain(self):
        r = ff.read_cell(scene({1: (25, 25, 25)}), self.cal, tilt_deg=25.0)
        self.assertEqual((r['color'], r['conf']), ('white', ff.RAMP_CONF))

    def test_pitched_h_matches_model(self):
        cam = dict(ff.DEFAULT_CAM)
        for nose_up in (-4.0, 3.0):
            want = ff.to_image(ff.model_h(dict(cam, pitch_deg=cam['pitch_deg'] - nose_up)),
                               ff.cell_polygon())
            got = ff.to_image(ff.pitched_h(ff.model_h(cam), nose_up), ff.cell_polygon())
            self.assertLess(np.abs(got - want).max(), 1.0)

    def test_small_tilt_still_reads(self):
        r = ff.read_cell(scene({1: (25, 25, 25)}), self.cal, tilt_deg=2.0)
        self.assertEqual(r['color'], 'black')

    def test_save_load(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'f.json')
            ff.save(self.cal, p)
            back = ff.load(p)
            self.assertEqual(set(back['samples']), {'white', 'black', 'blue'})
            self.assertEqual(ff.read_cell(scene({1: (25, 25, 25)}), back)['color'], 'black')

    def test_prep_rotates_and_crops(self):
        raw = np.zeros((720, 1280, 3), np.uint8)
        raw[:, :50] = 255                         # бампер слева на исходном кадре
        out = ff.prep(raw)
        self.assertEqual(out.shape[1], 720)
        self.assertEqual(int(out.max()), 0)       # после поворота он снизу и отрезан


if __name__ == '__main__':
    unittest.main()
