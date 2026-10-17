"""Тесты дисплея сзади робота: pi/oled.py (кадр, шрифты, экраны, драйвер
на поддельной шине, сообщения say() и разбор живого снимка)."""
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import oled  # noqa: E402
from oled_font import FONTS  # noqa: E402


class TestFont(unittest.TestCase):
    def test_every_label_is_drawable(self):
        # Символа нет в шрифте - на экране был бы «?». Всё, что экраны
        # выводят из своих таблиц, плюс весь алфавит.
        text = "".join(oled.PHASE.values()) + "".join(oled.TILE.values())
        text += "".join(s for v in oled.WAIT_HINT.values() for s in v)
        text += "".join(oled.ARROW.values()) + oled.DIRS + "●○…×°—·"
        text += "АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдеёжзийклмнопрстуфхцчшщъыьэюя"
        for name, (_, _, glyphs) in FONTS.items():
            missing = sorted(set(text) - set(glyphs))
            self.assertEqual(missing, [], name)

    def test_glyph_rows_match_height(self):
        for name, (fw, fh, glyphs) in FONTS.items():
            for ch, h in glyphs.items():
                rows = bytes.fromhex(h)
                self.assertEqual(len(rows), fh, (name, ch))
                self.assertTrue(all(r < (1 << fw) for r in rows), (name, ch))


class TestCanvas(unittest.TestCase):
    def test_pixel_layout_is_controller_pages(self):
        cv = oled.Canvas()
        cv.pixel(5, 9)                   # страница 1, бит 1
        self.assertEqual(cv.buf[128 + 5], 0b10)
        self.assertTrue(cv.get(5, 9))
        cv.pixel(5, 9, False)
        self.assertEqual(cv.buf[128 + 5], 0)

    def test_text_advances_and_draws(self):
        cv = oled.Canvas()
        x = cv.text(0, 0, "Жук")
        self.assertEqual(x, 18)
        self.assertTrue(any(cv.buf))
        cv2 = oled.Canvas()
        cv2.text(0, 0, "Жук", scale=2)
        self.assertGreater(sum(bin(b).count("1") for b in cv2.buf),
                           sum(bin(b).count("1") for b in cv.buf))

    def test_text_clips_without_error(self):
        cv = oled.Canvas()
        cv.text(120, 60, "очень длинная строка за краем")


class TestTextLayout(unittest.TestCase):
    def test_clip(self):
        self.assertEqual(oled.clip("абв", 21), "абв")
        c = oled.clip("а" * 30, 21)
        self.assertEqual(len(c), 21)
        self.assertTrue(c.endswith("…"))

    def test_wrap_by_words_in_order(self):
        s = "[WARN] камера справа: кадр не пришёл за 1.0с"
        rows = oled.wrap(s, 21)
        self.assertTrue(all(len(r) <= 21 for r in rows))
        self.assertEqual(" ".join(rows), s)          # ничего не потеряно и не переставлено
        self.assertEqual(rows[0], "[WARN] камера справа:")

    def test_wrap_cuts_long_word(self):
        rows = oled.wrap("x" * 30 + " конец", 21)
        self.assertEqual(rows, ["x" * 21, "x" * 9 + " конец"])

    def test_wrap_splits_path_at_slash(self):
        rows = oled.wrap("нет (~/maze/tile_calib.json, нужны", 21)
        self.assertEqual(rows, ["нет", "(~/maze/", "tile_calib.json,", "нужны"])

    def test_paged_flips_pages(self):
        rows = ["1", "2", "3", "4", "5"]
        self.assertEqual(oled.paged(rows, 2, 0.0), ["1", "2"])
        self.assertEqual(oled.paged(rows, 2, oled.PAGE_S), ["3", "4"])
        self.assertEqual(oled.paged(rows, 2, 2 * oled.PAGE_S), ["5"])
        self.assertEqual(oled.paged(rows, 2, 3 * oled.PAGE_S), ["1", "2"])

    def test_alert_tag_replaced(self):
        st = {"wall": 100.0, "alert": ("[WARN] лидар молчит", 99.0)}
        self.assertEqual(oled._alert(st), "! лидар молчит")


def _same(fn, st):
    a, b = oled.Canvas(), oled.Canvas()
    oled.compose(a, st)
    fn(b, st)
    return a.buf == b.buf


class TestScreens(unittest.TestCase):
    def setUp(self):
        self.states = oled.demo_states()

    def test_all_demo_screens_render(self):
        for name, st in self.states.items():
            cv = oled.Canvas()
            oled.compose(cv, st)
            self.assertTrue(any(cv.buf), name)

    def test_nothing_is_cut_on_demo_screens(self):
        # Всё, что на образцах, должно влезать целиком. Исключение - последняя
        # строка лога на экране «nav.py выкл» (одна строка, обрезка с «…»).
        cut = []
        real = oled.clip

        def spy(s, n=oled.COLS):
            r = real(s, n)
            if r != str(s):
                cut.append(str(s))
            return r
        oled.clip = spy
        self.addCleanup(setattr, oled, "clip", real)
        for name, st in self.states.items():
            if name == "9_gone":
                continue
            cut.clear()
            oled.compose(oled.Canvas(), st)
            self.assertEqual(cut, [], name)

    def test_tile_calib_screen_fits_one_page(self):
        m = self.states["10_tile_calib"]["msg"]
        rows = [r for s in m["lines"] for r in oled.wrap(s, oled.COLS_SMALL)]
        self.assertLessEqual(len(rows), 6)

    def test_message_beats_run(self):
        st = dict(self.states["4_run"], msg=self.states["10_tile_calib"]["msg"])
        self.assertTrue(_same(oled.scr_msg, st))

    def test_screen_choice(self):
        s = self.states
        self.assertTrue(_same(oled.scr_idle, s["1_idle"]))
        self.assertTrue(_same(oled.scr_preflight, s["3_preflight"]))
        self.assertTrue(_same(oled.scr_run, s["4_run"]))
        self.assertTrue(_same(oled.scr_wait, s["6_wait_motors"]))
        self.assertTrue(_same(oled.scr_finish, s["8_finish"]))
        self.assertTrue(_same(oled.scr_gone, s["9_gone"]))
        stale = dict(s["4_run"], stale=5.0)
        self.assertTrue(_same(oled.scr_gone, stale))

    def test_victim_screen_while_fresh(self):
        self.assertTrue(_same(oled.scr_victim, self.states["14_victim_phi"]))
        self.assertTrue(_same(oled.scr_victim, self.states["17_victim_cog"]))
        # экран ожидания (моторы выключены) важнее метки
        st = dict(self.states["6_wait_motors"], victim=self.states["14_victim_phi"]["victim"])
        self.assertTrue(_same(oled.scr_wait, st))

    def test_letters_are_drawn_big(self):
        for ch in "ΦΨΩ":
            cv = oled.Canvas()
            oled.draw_glyph(cv, ch)
            ys = [y for y in range(oled.H) for x in range(oled.W) if cv.get(x, y)]
            self.assertGreater(max(ys) - min(ys), 50, ch)       # почти во всю высоту

    def test_kits_columns_fill_from_bottom(self):
        cv = oled.Canvas()
        oled.kits_columns(cv, {"kits": {"L": 1, "R": 4}, "kits_max": 4})
        # середина ячеек столбика Л (x 1..12): залита только нижняя
        filled = [cv.get(7, 10 + j * 11 + 4) for j in range(4)]
        self.assertEqual(filled, [False, False, False, True])
        self.assertTrue(all(cv.get(22, 10 + j * 11 + 4) for j in range(4)))

    def test_fake_targets_are_not_victims(self):
        self.assertEqual(oled._victims({"victims_n": 3, "victims_real": 2}), 2)
        self.assertEqual(oled._victims({"victims_n": 3}), 3)       # старый nav.py

    def test_problems_only_when_broken(self):
        st = dict(self.states["4_run"])
        self.assertIsNone(oled._problems(st))
        st = dict(st, rates={"esp": 40.0, "lidar": 10.0})
        self.assertEqual(oled._problems(st), "! ESP 40Гц")

    def test_old_alert_not_shown(self):
        st = self.states["5_run_alert"]
        self.assertIsNotNone(oled._alert(st))
        old = dict(st, alert=("[WARN] давно", st["wall"] - oled.ALERT_SHOW_S - 1))
        self.assertIsNone(oled._alert(old))

    def test_position_strips_level(self):
        snap = {"pose": {"cx": 1, "cy": 1003, "hdg": 3, "cp": [0, 0, 0], "level": 1}}
        self.assertEqual(oled._pos(snap), "1,3 ←W эт1")

    def test_missing_fields_do_not_crash(self):
        # Снимок старого nav.py (без новых полей) или почти пустой.
        for ph in list(oled.PHASE) + ["что-то новое", None]:
            st = dict(self.states["4_run"], snap={"phase": ph})
            oled.compose(oled.Canvas(), st)


class FakeBus:
    """Ssd1306 на временном файле вместо /dev/i2c-1: ioctl подменён,
    записи ложатся в файл одна за другой."""

    def __init__(self, testcase):
        self.tmp = tempfile.NamedTemporaryFile(delete=False)
        self.tmp.close()
        self.writes = []
        real_write = os.write

        def write(fd, data):
            self.writes.append(bytes(data))
            return real_write(fd, data)
        self._patches = [(oled.fcntl, "ioctl", lambda fd, req, arg: 0),
                         (oled.os, "write", write)]
        self._saved = [(o, n, getattr(o, n)) for o, n, _ in self._patches]
        for o, n, v in self._patches:
            setattr(o, n, v)
        testcase.addCleanup(self.close)

    def close(self):
        for o, n, v in self._saved:
            setattr(o, n, v)
        os.unlink(self.tmp.name)


class TestDriver(unittest.TestCase):
    def test_rotation_in_init(self):
        bus = FakeBus(self)
        d = oled.Ssd1306(dev=bus.tmp.name, rot180=True, log=lambda m: None)
        self.assertTrue(d.show(oled.Canvas()))
        init = next(w for w in bus.writes if 0xAF in w)
        self.assertIn(0xA0, init)
        self.assertIn(0xC0, init)
        self.assertNotIn(0xAE, init)        # без «выкл» - не моргает

        bus2 = FakeBus(self)
        d2 = oled.Ssd1306(dev=bus2.tmp.name, rot180=False, log=lambda m: None)
        d2.show(oled.Canvas())
        init2 = next(w for w in bus2.writes if 0xAF in w)
        self.assertIn(0xA1, init2)
        self.assertIn(0xC8, init2)

    def test_only_changed_pages_are_sent(self):
        bus = FakeBus(self)
        d = oled.Ssd1306(dev=bus.tmp.name, log=lambda m: None)
        cv = oled.Canvas()
        d.show(cv)
        data = [w for w in bus.writes if w[0] == 0x40]
        self.assertEqual(len(data), 8)       # первый кадр - целиком
        bus.writes.clear()
        d.show(cv)
        self.assertEqual(bus.writes, [])     # ничего не поменялось - тишина
        cv.pixel(10, 20)                     # страница 2
        d.show(cv)
        data = [w for w in bus.writes if w[0] == 0x40]
        self.assertEqual(len(data), 1)
        self.assertEqual(len(data[0]), 2)              # только столбец 10
        self.assertIn(bytes((0x00, 0x21, 10, 10, 0x22, 2, 2)), bus.writes)

    def test_contrast_change_is_sent_once(self):
        bus = FakeBus(self)
        d = oled.Ssd1306(dev=bus.tmp.name, log=lambda m: None)
        d.show(oled.Canvas())
        bus.writes.clear()
        d.contrast = oled.DIM_CONTRAST
        d.show(oled.Canvas())
        d.show(oled.Canvas())
        self.assertEqual(bus.writes, [bytes((0x00, 0x81, oled.DIM_CONTRAST))])

    def test_missing_bus_is_not_fatal(self):
        msgs = []
        d = oled.Ssd1306(dev="/nonexistent/i2c-9", log=msgs.append)
        self.assertFalse(d.show(oled.Canvas()))
        self.assertFalse(d.show(oled.Canvas()))      # повтор - не раньше RETRY_S
        self.assertEqual(len(msgs), 1)


class TestSysInfo(unittest.TestCase):
    def test_power_flags(self):
        self.assertEqual(oled._power(0), "питание ок")
        self.assertEqual(oled._power(0x50000), "! были просадки")
        self.assertEqual(oled._power(0x50005), "! НИЗКОЕ ПИТАНИЕ")
        self.assertEqual(oled._power(None), "питание ?")

    def test_uptime(self):
        self.assertEqual(oled._uptime(59), "0мин")
        self.assertEqual(oled._uptime(754), "12мин")
        self.assertEqual(oled._uptime(3 * 3600 + 25 * 60), "3ч25")

    def test_poll_does_not_crash_off_pi(self):
        si = oled.SysInfo(start=False)
        si.poll()                                   # на маке половина - None
        self.assertEqual(set(si.info), {"ip", "ssid", "temp", "mem", "uptime", "throttled"})


class TestVideo(unittest.TestCase):
    def test_file_roundtrip_and_daemon_frames(self):
        import oled_video
        fr = [bytes([k]) * 1024 for k in range(3)]
        with tempfile.NamedTemporaryFile(suffix=".oledv", delete=False) as f:
            f.write(oled_video.MAGIC + bytes([10]) + b"".join(fr))
        self.addCleanup(os.unlink, f.name)
        self.assertEqual(oled_video.load(f.name), (10, fr))

    def test_contours_drop_background_and_stray_blobs(self):
        import oled_video
        w, h = 20, 16
        g = bytearray([255]) * (w * h)                 # белый фон
        for y in range(4, 12):
            for x in range(4, 12):
                g[y * w + x] = 120                     # «кот» 8x8
        g[2 * w + 17] = 100                            # отдельная точка - значок
        out = oled_video.frame_bits(bytes(g), w, h, edges=260)
        cv = oled.Canvas()
        cv.buf[:] = out
        x0, y0 = (oled.W - w) // 2, (oled.H - h) // 2
        self.assertTrue(cv.get(x0 + 4, y0 + 4))        # край силуэта горит
        self.assertFalse(cv.get(x0 + 8, y0 + 8))       # середина - нет (только контуры)
        self.assertFalse(cv.get(x0 + 17, y0 + 2))      # значок убран


class TestCatAndButton(unittest.TestCase):
    def test_button_debounce_one_press_per_hold(self):
        levels = iter([0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0])   # 1 - отпущена; низко с самого начала - не нажатие
        b = oled.Button(read=lambda: bool(next(levels)), start=False)
        for _ in range(13):
            b.poll()
        # одиночный LOW - дребезг; удержание - одно нажатие; потом ещё одно
        self.assertEqual(b.presses, 2)

    def test_pinctrl_parse(self):
        self.assertEqual(oled.parse_pinctrl("27: ip    pu | hi // GPIO27 = input"), (True, True))
        self.assertEqual(oled.parse_pinctrl("27: ip    pu | lo // GPIO27 = input"), (False, True))
        self.assertEqual(oled.parse_pinctrl("27: ip    pd | lo // GPIO27 = input"), (False, False))

    def test_gpio_users_found_by_cmdline(self):
        with tempfile.TemporaryDirectory() as d:
            def proc(pid, cmd):
                os.makedirs(os.path.join(d, str(pid)))
                with open(os.path.join(d, str(pid), "cmdline"), "wb") as f:
                    f.write(cmd.replace(" ", "\0").encode())
            proc(100, "/usr/bin/python3 -u /home/madness/maze/oled.py")
            proc(101, "bash")
            self.assertFalse(oled.gpio_users_running(d))
            proc(102, "/home/madness/maze/venv/bin/python -u /home/madness/maze/nav.py --no-color")
            self.assertTrue(oled.gpio_users_running(d))

    def test_cat_starts_from_first_frame_each_time(self):
        t = oled.restart_clock(None, True, 10.0)
        self.assertEqual(t, 10.0)
        self.assertEqual(oled.restart_clock(t, True, 14.0), 10.0)   # идёт дальше
        t = oled.restart_clock(t, False, 15.0)                       # переключили на IP
        self.assertIsNone(t)
        self.assertEqual(oled.restart_clock(t, True, 20.0), 20.0)   # вернули - с начала

    def test_cat_mode(self):
        c = oled.CatMode()
        self.assertTrue(c.show(False, None, None, False))          # после включения - кот
        c.step(1, busy=False, connected=False)                     # нажали - экран с IP
        self.assertFalse(c.show(False, None, None, False))
        c.step(2, busy=False, connected=False)                     # ещё раз - кот
        self.assertTrue(c.on)
        c.step(3, busy=True, connected=False)                      # nav.py ждёт старта - кнопка его
        self.assertTrue(c.on)
        self.assertFalse(c.show(True, None, None, False))          # и кота не видно
        c.step(3, busy=False, connected=True)                      # заезд идёт
        self.assertFalse(c.show(False, None, {"phase": "exploring"}, True))
        c.step(3, busy=False, connected=False)                     # кончился - итог, не кот
        self.assertFalse(c.on)
        self.assertFalse(c.show(False, {"title": "x"}, None, False))   # say() важнее


class TestMessages(unittest.TestCase):
    def test_say_reaches_inbox(self):
        box = oled.Inbox(port=0)
        oled.say("ШАПКА", ["раз", "два"], alert=True, ttl=0.5, port=box.port)
        end = time.monotonic() + 2.0
        while box.msg is None and time.monotonic() < end:
            time.sleep(0.01)
        m = box.current(time.monotonic())
        assert m is not None
        self.assertEqual(m["title"], "ШАПКА")
        self.assertEqual(m["lines"], ["раз", "два"])
        self.assertTrue(m["alert"])
        self.assertIsNone(box.current(time.monotonic() + 1.0))   # ttl вышел
        box.sock.close()

    def test_say_without_daemon_is_silent(self):
        oled.say("никого", ["нет"], port=9)          # discard - не бросает


class TestFeedVictims(unittest.TestCase):
    def test_victim_shown_while_reporting_only(self):
        f = oled.Feed(start=False)
        si = oled.SysInfo(start=False)
        f.take({"phase": "exploring", "map": {"victims": []}}, 10.0)
        v = {"type": "stable", "label": "Ψ", "blink": True}
        f.take({"phase": "exploring", "act": "жертва", "map": {"victims": [v]}}, 11.0)
        self.assertEqual(oled.build_state(17.0, f, si)["victim"], v)    # мигает и выдаёт
        f.take({"phase": "exploring", "act": "ход"}, 17.5)
        self.assertIsNone(oled.build_state(17.6, f, si)["victim"])      # поехал дальше
        fake = {"type": "fake", "label": "-4", "blink": False}
        f.take({"phase": "exploring", "act": "ход", "map": {"victims": [v, fake]}}, 20.0)
        self.assertEqual(oled.build_state(21.0, f, si)["victim"], fake)
        self.assertIsNone(oled.build_state(20.0 + oled.VICTIM_MIN_S + 0.1, f, si)["victim"])

    def test_only_new_victims_are_shown(self):
        f = oled.Feed(start=False)
        si = oled.SysInfo(start=False)
        old = {"type": "damaged", "label": "Φ"}
        f.take({"phase": "exploring", "map": {"victims": [old]}}, 10.0)
        self.assertIsNone(f.victim)                 # была до подключения - не показываем
        new = {"type": "stable", "label": "Ψ"}
        f.take({"phase": "exploring", "map": {"victims": [old, new]}}, 11.0)
        f.connected = True
        self.assertEqual(oled.build_state(11.5, f, si)["victim"], new)
        self.assertIsNone(oled.build_state(11.0 + oled.VICTIM_SHOW_S + 0.1, f, si)["victim"])
        f.take({"phase": "boot", "map": {"victims": []}}, 30.0)     # nav.py перезапустили
        f.take({"phase": "exploring", "map": {"victims": [old]}}, 31.0)
        self.assertEqual(oled.build_state(31.5, f, si)["victim"], old)


class TestFeed(unittest.TestCase):
    def test_rates_and_alerts(self):
        f = oled.Feed(start=False)
        wall = time.time()
        f.take({"telem_n": 0, "scan_n": 0, "log": [[1, wall, "  Ожидание лидара..."]]}, 10.0)
        self.assertEqual(f.lastline, "Ожидание лидара...")
        self.assertIsNone(f.alert)
        f.take({"telem_n": 200, "scan_n": 10, "map": {}, "scan": [1, 2],
                "log": [[2, wall, "  [WARN] лидар: неполный оборот"]]}, 11.0)
        esp, lid = f.rates["esp"], f.rates["lidar"]
        assert esp is not None and lid is not None and f.alert is not None
        self.assertAlmostEqual(esp, 200.0)
        self.assertAlmostEqual(lid, 10.0)
        self.assertEqual(f.alert[0], "[WARN] лидар: неполный оборот")
        snap = f.snap
        assert snap is not None
        self.assertNotIn("map", snap)                # тяжёлое не держим
        self.assertNotIn("scan", snap)


if __name__ == "__main__":
    unittest.main()
