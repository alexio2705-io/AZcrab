"""Тесты живого потока для пульта: pi/live.py (публикатор, перехват stdout)
и сборки состояния в pi/dash_web.py (Hub)."""
import gzip
import io
import json
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import dash_web  # noqa: E402
import live  # noqa: E402


class TestLogTee(unittest.TestCase):
    def setUp(self):
        self.out = io.StringIO()
        self.tee = live.LogTee(self.out, clock=lambda: 1.0)

    def test_passes_through_and_splits_lines(self):
        self.tee.write("один\nдва")
        self.tee.write(" три\n")
        self.assertEqual(self.out.getvalue(), "один\nдва три\n")
        self.assertEqual([t for _, _, t in self.tee.lines], ["один", "два три"])

    def test_carriage_return_is_status_not_log(self):
        self.tee.write("\r    кл.1 ост. 120мм")
        self.tee.write("\r    кл.1 ост.  80мм")
        self.assertEqual(self.tee.status, "кл.1 ост.  80мм")
        self.assertEqual(len(self.tee.lines), 0)
        # строка с переводом после статуса - в лог идёт то, что видно глазами
        self.tee.write("\r    клетка 1 пройдена с ходу      \n")
        self.assertEqual([t for _, _, t in self.tee.lines], ["    клетка 1 пройдена с ходу"])
        self.assertEqual(self.tee.status, "")

    def test_since_returns_only_new(self):
        for i in range(5):
            self.tee.write(f"л{i}\n")
        self.assertEqual([t for _, _, t in self.tee.since(3)], ["л3", "л4"])
        self.assertEqual(self.tee.since(5), [])

    def test_keep_limit(self):
        tee = live.LogTee(io.StringIO(), keep=3)
        for i in range(10):
            tee.write(f"{i}\n")
        self.assertEqual([t for _, _, t in tee.lines], ["7", "8", "9"])
        self.assertEqual(tee.seq, 10)


class TestPublisher(unittest.TestCase):
    def test_roundtrip_with_log_backlog(self):
        tee = live.LogTee(io.StringIO())
        tee.write("до подключения\n")
        n_calls = []

        def snap(n):
            n_calls.append(n)
            return {"n": n, "s": {1, 2}}        # множество - через _json_default

        pub = live.LivePublisher(snap, tee=tee, port=0, hz=50)
        try:
            time.sleep(0.05)
            self.assertEqual(n_calls, [], "без клиента снимок не собирается")
            gen = live.read_lines(port=pub.port, timeout=2.0)
            first = next(gen)
            self.assertEqual(first["s"], [1, 2])
            self.assertEqual([t for _, _, t in first["log"]], ["до подключения"])
            tee.write("после\n")
            got = []
            for m in gen:
                got += [t for _, _, t in m["log"]]
                if "после" in got:
                    break
            self.assertEqual(got, ["после"], "старые строки второй раз не шлются")
            gen.close()
        finally:
            pub.close()

    def test_snapshot_error_does_not_kill_thread(self):
        state = {"fail": True}

        def snap(n):
            if state["fail"]:
                raise RuntimeError("dict changed size")
            return {"ok": n}

        out = io.StringIO()
        old, sys.stdout = sys.stdout, out
        pub = live.LivePublisher(snap, port=0, hz=50)
        try:
            gen = live.read_lines(port=pub.port, timeout=2.0)
            t = threading.Timer(0.1, lambda: state.update(fail=False))
            t.start()
            self.assertIn("ok", next(gen))
            gen.close()
        finally:
            pub.close()
            sys.stdout = old
        self.assertEqual(out.getvalue().count("[WARN]"), 1)


class TestHub(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.old_runs = dash_web.RUNS_DIR
        dash_web.RUNS_DIR = self.tmp

    def tearDown(self):
        dash_web.RUNS_DIR = self.old_runs

    def snap(self, t, **kw):
        s = {"t": t, "imu": {"pitch": 1.0}, "walls": {"front": 100}, "odo": [10.0, 12.0]}
        s.update(kw)
        return s

    def test_client_gets_map_once_and_log_incrementally(self):
        h = dash_web.Hub(record=False)
        h.on_connect()
        h.on_snapshot(self.snap(1.0, map={"maze": {}}, log=[[1, 0.0, "а"]]))
        c = {"log_seq": 0, "map_ver": -1, "n": -1}
        m1 = h.message(c)
        self.assertEqual(m1["map"], {"maze": {}})
        self.assertEqual([t for _, _, t in m1["log"]], ["── пульт: связь с nav.py ──", "а"])
        h.on_snapshot(self.snap(1.1, map={"maze": {}}))       # та же карта
        m2 = h.message(c)
        self.assertNotIn("map", m2)
        self.assertNotIn("log", m2)
        self.assertNotIn("log", h.snap, "лог не хранится в последнем снимке")

    def test_history_window(self):
        h = dash_web.Hub(record=False)
        for i in range(200):
            h.on_snapshot(self.snap(i * 0.5))
        span = h.hist[-1][0] - h.hist[0][0]
        self.assertLessEqual(span, dash_web.HIST_S)
        self.assertEqual(h.hist[-1][9], 11.0)       # средний путь колёс

    def test_recording_has_map_and_log_and_empty_is_removed(self):
        h = dash_web.Hub(record=True)
        h.on_connect()
        h.on_snapshot(self.snap(1.0, map={"maze": {"0,0": [1, 0, 0, 0]}}, log=[[5, 0.0, "x"]]))
        h.on_snapshot(self.snap(1.1))
        path = h.rec_path
        h.on_disconnect()
        rows = [json.loads(ln) for ln in gzip.open(path, "rt")]
        self.assertEqual(len(rows), 2)
        self.assertIn("map", rows[0])
        self.assertEqual(rows[0]["log"], [[5, 0.0, "x"]])
        self.assertNotIn("map", rows[1])
        # сеанс без снимков - файла не остаётся
        h.on_connect()
        empty = h.rec_path
        h.on_disconnect()
        self.assertFalse(os.path.exists(empty))
        self.assertEqual([r["name"] for r in dash_web.list_runs()], [os.path.basename(path)])


if __name__ == "__main__":
    unittest.main()
