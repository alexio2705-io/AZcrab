import ast
import binascii
import importlib.util
import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
from motion_guard import SlipGuard
from floor_pose import from_snapshot
import config
import runplan


class RouteTests(unittest.TestCase):
    def route(self, blue_penalty, detour=True):
        cells = {(0,0),(1,0),(2,0)} | ({(0,1),(1,1),(2,1)} if detour else set())
        dx, dy = [0,1,0,-1], [1,0,-1,0]
        def nxt(x,y,d): return x+dx[d], y+dy[d]
        def dirs(x,y): return [d for d in range(4) if nxt(x,y,d) in cells]
        def cost(src,dst,h,d):
            return 2 + .6*abs((d-h+2)%4-2) + (blue_penalty if dst == (1,0) else 0)
        return runplan.weighted_path((0,0),1,dirs,nxt,lambda c:c==(2,0),(1,0,3,2),cost)

    def test_avoid_blue_revisit(self):
        self.assertEqual(self.route(13.2), [0,1,1,2])

    def test_blue_only_route_still_allowed(self):
        self.assertEqual(self.route(13.2,False), [1,1])

    def test_challenge_blue_normal_has_no_penalty(self):
        self.assertEqual(self.route(0), [1,1])

    def test_goal_unreachable(self):
        self.assertIsNone(runplan.weighted_path((0,0),0,lambda x,y:[],
            lambda x,y,d:(x,y),lambda c:c==(1,0),(1,0,3,2),lambda *args:1))


class ProgressTests(unittest.TestCase):
    def run_trace(self, motion, pitch=0, missing=False):
        g = SlipGuard()
        results = []
        for i in range(7):
            t = 10 + i * .1
            walls = {} if missing else {"front": 1000 - motion * i, "back": 400 + motion * i}
            results.append(g.update(i, t, t, i * 25, 0, pitch, walls, 25))
        return g, results

    def test_wheels_spin_body_still(self):
        self.assertTrue(any(self.run_trace(0)[1]))

    def test_normal_motion(self):
        self.assertFalse(any(self.run_trace(25)[1]))

    def test_stairs_missing_walls(self):
        g, results = self.run_trace(0, missing=True)
        self.assertFalse(any(results))
        self.assertEqual(g.evidence, "unobservable")

    def test_pitch_not_stall(self):
        self.assertFalse(any(self.run_trace(0, pitch=15)[1]))

    def test_repeated_scan_is_not_evidence(self):
        g = SlipGuard()
        self.assertFalse(g.update(1, 10, 10, 0, 0, 0, {"front": 500}, 20))
        self.assertFalse(g.update(1, 11, 10, 200, 0, 0, {"front": 500}, 20))

    def test_stale_and_turning(self):
        g = SlipGuard()
        self.assertFalse(g.update(1, 10, 8, 0, 0, 0, {"front": 500}, 20))
        self.assertEqual(g.evidence, "unobservable")
        g.update(2, 11, 11, 0, 0, 0, {"front": 500}, 20)
        self.assertFalse(g.update(3, 11.4, 11.4, 100, 45, 0, {"front": 500}, 20))

    def test_disagreeing_surfaces(self):
        g = SlipGuard()
        g.update(1, 10, 10, 0, 0, 0, {"front": 500, "back": 400}, 20)
        self.assertFalse(g.update(2, 10.4, 10.4, 100, 0, 0, {"front": 500, "back": 500}, 20))


class PoseTests(unittest.TestCase):
    def test_offset_and_yaw(self):
        s = {"t": 10, "imu": {"yaw": 100, "pitch": 2}, "pose": {"hdg": 1},
             "drive": {"travel_mm": 360, "hdg": 1}, "walls": {"left": 120, "right": 160}}
        p, pitch = from_snapshot(s, 10.1)
        self.assertAlmostEqual(p[0], 60)
        self.assertAlmostEqual(p[1], 20 * math.cos(math.radians(10)))
        self.assertAlmostEqual(p[2], -math.radians(10))
        self.assertEqual(pitch, 2)
        self.assertIsNone(from_snapshot(s, 11))


class CenteringTests(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(telem={"scan_t": 10})
        esp = SimpleNamespace(get_yaw=lambda:0, get_yaw_and_rate=lambda:(0,0))
        turn = SimpleNamespace(angle_err=lambda a,b:(a-b+180)%360-180)
        self.modules = patch.dict(sys.modules, state=self.state, esp_link=esp, turning=turn)
        self.modules.start()
        spec = importlib.util.spec_from_file_location('_drive_motion_test',PI/'drive_motion.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.clock = patch.object(self.module.time,'monotonic',return_value=10.1)
        self.clock.start()

    def tearDown(self):
        self.clock.stop()
        self.modules.stop()

    def trip(self, left, right, pitch=0):
        return SimpleNamespace(walls={'left':left,'right':right},pitch_delta=pitch,target_yaw=0)

    def test_wall_error_sign_and_deadband(self):
        for l,r,expected in [(120,160,20),(160,120,-20),(138,142,0),(120,None,20),(None,160,20)]:
            t=self.trip(l,r)
            self.module.wall_offset(t)
            self.assertEqual(t.wall_e,expected)

    def test_no_steering_from_stale_or_hidden_walls(self):
        for l,r,p in [(None,None,0),(120,160,15),(90,90,0)]:
            t=self.trip(l,r,p)
            self.module.wall_offset(t)
            self.assertEqual(t.wall_e,0)
        self.state.telem['scan_t']=9
        t=self.trip(120,160)
        self.module.wall_offset(t)
        self.assertEqual(t.wall_e,0)

    def test_smoothing_independent_of_tick_rate(self):
        def run(dt,n):
            t=SimpleNamespace(target_yaw=0,pitch_delta=0,wall_e=20,head_off_f=0,
                              loop_dt=dt,last_steer=0,ss=1,fwd=20,assist_mode=None,yaw_i=0.0)
            for _ in range(n): self.module.steering(t)
            return t.head_off_f
        self.assertAlmostEqual(run(.01,1),run(.005,2),places=10)

    def test_slow_straight_drive_keeps_both_wheels_forward(self):
        for wall_e in (0, 20):
            with self.subTest(wall_e=wall_e):
                t = SimpleNamespace(target_yaw=0, pitch_delta=0, wall_e=wall_e,
                                    head_off_f=0, loop_dt=.01, last_steer=0,
                                    ss=1, fwd=8, assist_mode=None, yaw_i=0.0)
                with patch.object(sys.modules['esp_link'], 'get_yaw_and_rate',
                                  return_value=(5, 0)):
                    self.module.steering(t)
                self.assertLessEqual(abs(t.steer), .6 * t.fwd)
                self.assertEqual(t.yaw_i, 0)
                self.assertGreater(t.fwd - abs(t.steer), 0)

    def test_first_downhill_tick_caps_both_sides_even_with_previous_steer(self):
        for known, phase, sign, pitch in [(True, None, 0, 0), (False, 'cand', -1, -7)]:
            with self.subTest(known=known):
                t = SimpleNamespace(t0=9, v_max=70, loop_dt=.005, pitch_delta=pitch,
                                    known_descent=known, ramp_state=phase, ramp_sign=sign,
                                    assist_mode=None, tilt_lim=70, pending=False, stopping=False,
                                    mark_first=False, target_yaw=0, wall_e=0, head_off_f=0,
                                    last_steer=28, yaw_i=0)
                self.module.speed_profile(t)
                self.module.steering(t)
                self.assertGreater(t.fwd - abs(t.steer), 0)
                self.assertLessEqual(t.fwd + abs(t.steer), self.module.RAMP_DESCENT_CMS)

    def test_integral_does_not_wind_up_at_steering_limit(self):
        t = SimpleNamespace(target_yaw=0, pitch_delta=0, wall_e=0,
                            head_off_f=0, loop_dt=.01, last_steer=0,
                            ss=1, fwd=40, assist_mode=None, yaw_i=0.0)
        with patch.object(sys.modules['esp_link'], 'get_yaw_and_rate',
                          return_value=(10, 0)):
            for _ in range(100):
                t.fwd = 40
                self.module.steering(t)
        self.assertEqual(t.yaw_i, 0)

    def test_camera_speed_cap_changes_gradually_after_first_activation(self):
        t = SimpleNamespace(t0=9.0, v_max=45.0, loop_dt=.05, pitch_delta=0.0,
                            assist_mode=None, tilt_lim=45.0, pending=False,
                            ramp_state=None, stopping=False, mark_first=False,
                            perception_cap=9.0)
        self.module.speed_profile(t)
        self.assertEqual(t.fwd, 9.0)
        t.perception_cap = 45.0
        self.module.speed_profile(t)
        self.assertAlmostEqual(t.fwd, 12.0)
        t.perception_cap = 9.0
        self.module.speed_profile(t)
        self.assertAlmostEqual(t.fwd, 9.0)
        t.perception_cap = float('inf')
        self.module.speed_profile(t)
        self.assertAlmostEqual(t.fwd, 12.0)

    def test_flat_stop_does_not_accelerate_again_before_brake(self):
        t = SimpleNamespace(t0=9.0, v_max=50.0, loop_dt=.01, pitch_delta=0.0,
                            assist_mode=None, tilt_lim=50.0, pending=False,
                            ramp_state=None, stopping=True, mark_first=False,
                            fwd=50.0, perception_cap=float('inf'))
        speeds = []
        # Последовательные измерения остатка, см. Та же проверка прихода,
        # что в aim_at_mark/check_arrived, выполняется до speed_profile.
        for rem in [6, 5, 4, 3] + [i / 10 for i in range(20, 1, -1)]:
            t.rem = rem
            t.rem_eff = rem - t.fwd * config.STOP_LEAD_S
            if t.rem_eff <= 0:
                break
            self.module.speed_profile(t)
            speeds.append(t.fwd)
        self.assertTrue(all(b <= a for a, b in zip(speeds, speeds[1:])), speeds)
        self.assertLessEqual(t.rem_eff, 0)
        self.assertLessEqual(t.fwd, config.MOVE_MIN_CMS)


class WireTests(unittest.TestCase):
    def test_v4_layout_crc_and_unsigned_voltage(self):
        # Physical packet offsets checked independently of the decoder.
        vals = [4, 17] + [1.] * 7 + [1] + [11, 12, 13, 14] + [15., 16.] + [0, 127, 3, 1234, 0]
        vals += [160, 161, 159, 162] + [-128, 25, 26, 27] + [0, 0, 1, 0] + [0]
        body = bytearray(struct.pack(config.FMT, *vals))
        body[-2:] = struct.pack('<H', binascii.crc_hqx(body[:-2], 0xffff))
        pkt = b'\xaa\x55' + body
        self.assertEqual(len(pkt), 74)
        self.assertEqual(struct.unpack_from('<4H', pkt, 56), (160, 161, 159, 162))
        self.assertEqual(struct.unpack_from('<4b', pkt, 64), (-128, 25, 26, 27))
        self.assertEqual(binascii.crc_hqx(pkt[2:72], 0xffff), struct.unpack_from('<H', pkt, 72)[0])

    def test_all_python_sources_parse(self):
        for p in PI.rglob('*.py'):
            with self.subTest(path=str(p)):
                ast.parse(p.read_text(encoding='utf-8-sig'), filename=str(p))


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV required for camera geometry tests')
class FloorTests(unittest.TestCase):
    def test_invalid_calibration_is_unknown(self):
        import numpy as np
        import front_floor as ff
        for h in [np.zeros((3,3)), np.full((3,3),np.nan), np.eye(2)]:
            r=ff.read_cell(np.zeros((973,720,3),np.uint8), {'H':h})
            self.assertEqual(r['color'],'unknown')
            self.assertEqual(r['conf'],0)

    def test_blue_at_offsets_and_angles(self):
        import numpy as np
        import front_floor as ff
        # Blue floor with a physical tile rectangle rendered by inverse mapping.
        # Other pixels red, so sampling the wrong tile cannot silently pass.
        hh = ff.model_h()
        h, w = 973, 720
        yy, xx = np.mgrid[0:h, 0:w]
        p = hh @ np.vstack((xx.ravel(), yy.ravel(), np.ones(h*w)))
        fx, fy = p[0]/p[2], p[1]/p[2]
        for ox, oy, deg in [(0,0,0), (20,15,10), (-60,-35,-15)]:
            a = math.radians(deg)
            gx, gy = fx*math.cos(a)-fy*math.sin(a)+ox, fx*math.sin(a)+fy*math.cos(a)+oy
            inside = (gx >= 150) & (gx <= 450) & (abs(gy) <= 150) & (fx > 95)
            frame = np.full((h*w,3), (0,0,255), np.uint8)
            frame[inside] = (255,0,0)
            r = ff.read_cell(frame.reshape(h,w,3), {'H':hh}, (ox,oy,a))
            self.assertEqual(r['color'], 'blue', (ox,oy,deg,r))
            self.assertGreater(r['conf'], .6)

    def test_out_of_view_and_slope_unknown(self):
        import numpy as np
        import front_floor as ff
        f = np.zeros((973,720,3), np.uint8)
        for pose, tilt in [((0,0,math.pi),0), ((0,30,math.radians(30)),0), ((0,0,0),20)]:
            r = ff.read_cell(f, {}, pose, tilt_deg=tilt)
            self.assertEqual(r['color'], 'unknown')
            self.assertEqual(r['conf'], 0)


if __name__ == '__main__':
    unittest.main()
