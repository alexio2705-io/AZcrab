"""Генератор страниц просмотрщика (sim/viewer/build.py)."""
import json
import os
import re
import sys
import tempfile
import unittest

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(SIM, "viewer"))

import build  # noqa: E402

TEMPLATE = ('<title>__TITLE__</title><script id="names" type="application/json">__SCENARIOS__'
            '</script><script id="trace" type="application/json">__TRACE__</script>')


def block(html, ident):
    m = re.search(rf'<script id="{ident}" type="application/json">(.*?)</script>', html, re.S)
    assert m, ident
    return json.loads(m.group(1))


class PageTest(unittest.TestCase):
    def test_data_embedded_and_escaped(self):
        nasty = "лог </script><!-- <b>"
        html = build.page_html(TEMPLATE, json.dumps({"text": nasty}, ensure_ascii=False),
                               ["flat", "ramp"], "ramp")
        self.assertNotIn(nasty, html)
        self.assertEqual(block(html, "trace"), {"text": nasty})
        self.assertEqual(block(html, "names"), ["flat", "ramp"])
        self.assertIn("<title>ramp", html)

    def test_index_links_every_scenario(self):
        rows = [{"name": "flat", "status": "explored", "ok": True, "sim_time": 20.1, "visited": 6,
                 "ramps": [], "map_errors": [], "crashes": 0},
                {"name": "ramp", "status": "explored", "ok": False, "sim_time": 30.0, "visited": 5,
                 "ramps": [[0, 1, 0]], "map_errors": ["(0,1)N: x"], "crashes": 1}]
        html = build.index_html(rows)
        self.assertIn('href="flat.html"', html)
        self.assertIn('href="ramp.html"', html)
        self.assertIn("(0,1)N: x", html)

    def test_write_site(self):
        tr = {"meta": {"name": "flat", "status": "explored", "ok": True, "sim_time": 1.0,
                       "visited": 1, "ramps": [], "map_errors": [], "crashes": 0}}
        with tempfile.TemporaryDirectory() as d:
            idx = build.write_site(d, {"flat": tr}, template=TEMPLATE)
            self.assertEqual(idx, os.path.join(d, "index.html"))
            self.assertTrue(os.path.exists(os.path.join(d, "flat.html")))
            with open(os.path.join(d, "data", "flat.json")) as f:
                self.assertEqual(json.load(f), tr)


if __name__ == "__main__":
    unittest.main()
