"""Страницы просмотрщика трасс: sim/viewer/<сценарий>.html и index.html.

Трасса вставляется прямо в страницу (браузер не даёт странице с file://
читать соседние файлы), three.js - локальная копия в vendor/. Запуск -
python3 sim/run_scenarios.py --html.
"""
from __future__ import annotations

import html
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from sim_trace import dump_trace  # noqa: E402


def _embed(js):
    # "<" бывает только внутри строк JSON - < там то же самое, а
    # </script> и <!-- в тексте лога не закроют блок данных
    return js.replace("<", "\\u003c")


def page_html(template, trace_json, names, name):
    return (template.replace("__TITLE__", html.escape(name))
            .replace("__SCENARIOS__", _embed(json.dumps(names)))
            .replace("__TRACE__", _embed(trace_json)))


_INDEX_CSS = """
:root{color-scheme:light dark;--bg:#f6f6f3;--fg:#1d1d1b;--mut:#6b6b66;--line:#d9d9d2;
--ok:#2f7d32;--bad:#c62828;--card:#fff}
@media (prefers-color-scheme:dark){:root{--bg:#16171a;--fg:#e8e8e3;--mut:#9a9a94;
--line:#2c2e33;--ok:#66bb6a;--bad:#ef5350;--card:#1e1f23}}
body{margin:0;padding:32px 40px;background:var(--bg);color:var(--fg);
font:14px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Text",sans-serif}
h1{font-size:20px;margin:0 0 4px}p{color:var(--mut);margin:0 0 20px}
table{border-collapse:collapse;background:var(--card);border:1px solid var(--line)}
th,td{padding:8px 14px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
th{font-weight:600;color:var(--mut);font-size:12px}
td.n{font-variant-numeric:tabular-nums;text-align:right}
a{color:inherit;font-weight:600}.ok{color:var(--ok)}.bad{color:var(--bad)}
"""


def index_html(rows):
    """Оглавление: строка на сценарий (meta трассы)."""
    def row(m):
        ok = m.get("ok")
        errs = m.get("map_errors") or []
        return ("<tr>"
                f'<td><a href="{html.escape(m["name"])}.html">{html.escape(m["name"])}</a></td>'
                f'<td>{html.escape(str(m.get("status")))}</td>'
                f'<td class="n">{m.get("sim_time")} с</td>'
                f'<td class="n">{m.get("visited")}</td>'
                f'<td>{html.escape(", ".join(str(tuple(r)) for r in m.get("ramps") or [])) or "—"}</td>'
                f'<td class="n">{m.get("crashes")}</td>'
                f'<td class="{"ok" if ok else "bad"}">{"нет" if not errs else html.escape("; ".join(errs))}</td>'
                "</tr>")
    body = "".join(row(m) for m in rows)
    return ("<!doctype html><html lang=\"ru\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
            f"<title>Сценарии симулятора</title><style>{_INDEX_CSS}</style></head><body>"
            "<h1>Сценарии симулятора</h1><p>Прогон настоящего nav.py в модели; "
            "страница сценария - анимация, карта, графики и лог.</p>"
            "<table><tr><th>сценарий</th><th>статус</th><th>время</th><th>клеток</th>"
            "<th>горки</th><th>упёрся</th><th>ошибки карты</th></tr>"
            f"{body}</table></body></html>")


ROBOT3D_SRC = os.path.join(os.path.dirname(os.path.dirname(HERE)), "pi", "dash", "robot3d.js")


def copy_robot3d(out_dir):
    """Модель робота - одна на пульт и просмотрщик: исходник в pi/dash,
    сюда - копия рядом с three.js."""
    os.makedirs(os.path.join(out_dir, "vendor"), exist_ok=True)
    shutil.copyfile(ROBOT3D_SRC, os.path.join(out_dir, "vendor", "robot3d.js"))


def write_site(out_dir, traces, template=None):
    """traces {имя: трасса} -> data/<имя>.json, <имя>.html, index.html;
    возвращает путь к index.html."""
    copy_robot3d(out_dir)
    if template is None:
        with open(os.path.join(HERE, "template.html"), encoding="utf-8") as f:
            template = f.read()
    os.makedirs(os.path.join(out_dir, "data"), exist_ok=True)
    names = list(traces)
    for name, tr in traces.items():
        js = dump_trace(tr)
        with open(os.path.join(out_dir, "data", f"{name}.json"), "w", encoding="utf-8") as f:
            f.write(js)
        with open(os.path.join(out_dir, f"{name}.html"), "w", encoding="utf-8") as f:
            f.write(page_html(template, js, names, name))
    idx = os.path.join(out_dir, "index.html")
    with open(idx, "w", encoding="utf-8") as f:
        f.write(index_html([tr["meta"] for tr in traces.values()]))
    return idx
