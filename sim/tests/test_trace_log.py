"""Строки лога со временем модели и разбор их в события (sim/sim_trace.py)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim_trace import TimedLines, parse_line, parse_log  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TimedLinesTest(unittest.TestCase):
    def test_line_gets_time_of_its_start(self):
        c = Clock()
        out = TimedLines(c)
        c.t = 1.0
        out.write("\n    → кл.(1, 0): ...")
        c.t = 2.0
        out.write(" ещё\n")
        self.assertEqual(out.lines, [(1.0, "    → кл.(1, 0): ... ещё")])

    def test_progress_lines_split_on_cr(self):
        c = Clock()
        out = TimedLines(c)
        c.t = 1.0
        out.write("\r    кл.1 ост. 300мм  spd=0   ")
        c.t = 2.0
        out.write("\r    кл.1 ост. 297мм  spd=21   ")
        out.write("\n")
        self.assertEqual([t for t, _ in out.lines], [1.0, 2.0])
        self.assertEqual(out.lines[1][1], "    кл.1 ост. 297мм  spd=21")

    def test_text_kept_verbatim(self):
        out = TimedLines(Clock())
        out.write("a\rb\n\nc")
        self.assertEqual(out.getvalue(), "a\rb\n\nc")

    def test_blank_lines_dropped_and_tail_kept_on_finish(self):
        out = TimedLines(Clock())
        out.write("\n   \n\rхвост без перевода")
        self.assertEqual(out.lines, [])
        out.finish()
        self.assertEqual(out.lines, [(0.0, "хвост без перевода")])


class ParseLineTest(unittest.TestCase):
    def check(self, text, typ, **fields):
        ev = parse_line(text)
        self.assertEqual(ev["type"], typ, text)
        for k, v in fields.items():
            self.assertEqual(ev[k], v, f"{text}: {k}")
        return ev

    def test_step(self):
        self.check("─── (2,1004) N → W  4/30 ───", "step", cell=[2, 1004], **{"from": "N", "to": "W"})

    def test_decide_straight(self):
        self.check("    → кл.(1, 0): R=есть(68т,175мм) L=есть(68т,175мм) F=нет(0:5) → прямо (новая)",
                   "decide", cell=[1, 0], straight=True, reason="новая",
                   sides={"R": "есть", "L": "есть", "F": "нет"})

    def test_decide_stop_unknown_front(self):
        self.check("→ кл.(0, 1003): R=нет(0т,165мм) L=нет(0т,165мм) F=?(1:1) → стоп (известна)",
                   "decide", cell=[0, 1003], straight=False, reason="известна",
                   sides={"R": "нет", "L": "нет", "F": "?"})

    def test_post(self):
        self.check("[odo] столбик right у границы кл.1: -27мм - цель -13мм", "post",
                   side="right", cell_no=1, err=-27, fix=-13)
        self.check("[odo] столбик left у границы кл.1: +15мм - цель +8мм", "post",
                   side="left", cell_no=1, err=15, fix=8)

    def test_progress(self):
        self.check("кл.2 ост.-126мм  yaw=37.0°(e=+0.0)  we=+0→+0.0°  spd=35", "progress",
                   cell_no=2, rem=-126, spd=35)

    def test_brake_and_wall(self):
        self.check("торможение с 55см/с: путь 50мм из 110мм в запасе", "brake", v=55)
        self.check("у стены: f=147мм", "at_wall", f=147)

    def test_turn(self):
        self.check("поворот -90°  цель=37.0°  yaw=127.0°", "turn", angle=-90, target=37.0)
        self.check("итог yaw=126.9°  ошибка=-0.1°  за 0.96с  TURN_FF=0.221", "turn_done",
                   yaw=126.9, err=-0.1)
        self.check("движение  курс=127°", "move", hdg=127)

    def test_ramps(self):
        self.check("горка: pitch=+20.2°, момент 1.9A + коррекция курса", "ramp_detect",
                   pitch=20.2, amps=1.9)
        self.check("горка: 612мм по уклону (~2.04 плит.), уклон 13.1°, высота +136мм, "
                   "островок через 3.03 кл. в плане -> 3", "ramp_measure",
                   length=612, slope=13.1, rise=136)
        self.check("ГОРКА ВВЕРХ (+13.0°) - еду без остановок до гребня", "ramp_up", pitch=13.0)
        self.check("ГОРКА ВНИЗ (-20.2°) - еду без остановок до гребня", "ramp_down", pitch=-20.2)
        self.check("⛰ ГОРКА вверх: ~1 плит., (0,1) → (0,1003), этаж 1, высота +14см",
                   "ramp_done", up=True, **{"from": [0, 1], "to": [0, 1003]})
        self.check("не горка: -3.9°, наклон держался 21мм", "not_ramp", pitch=-3.9)

    def test_obstacle_bumper_backoff_align(self):
        self.check("препятствие #1: pitch=+5.7°, v=25/52см/с, момент 2.5A (потолок 35см/с)",
                   "obstacle", n=1, pitch=5.7, amps=2.5)
        self.check("БАМПЕР (both) на 463мм", "bumper", side="both", mm=463)
        self.check("откат 70мм - вплотную стену не видно", "backoff", mm=70)
        self.check("[выр] ок, 140мм", "align")

    def test_other(self):
        self.check("⏱ 2.9с: ход 1.4, поворот 1.0, лидар 0.6", "other")


class ParseLogTest(unittest.TestCase):
    def test_time_relative_and_text_stripped(self):
        evs = parse_log([(1001.23456, "    у стены: f=147мм   ")], t0=1000.0)
        self.assertEqual(evs, [{"t": 1.235, "text": "у стены: f=147мм", "type": "at_wall", "f": 147}])


if __name__ == "__main__":
    unittest.main()
