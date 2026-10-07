"""Фейковый watch_proc.py для test_victim_watch.py: тот же протокол, без YOLO.
FAKE_SILENT=1 - готов, но на запросы не отвечает (воркер завис)."""
import json
import os
import sys

print(json.dumps({'ready': True}), flush=True)
for line in sys.stdin:
    idx, fid, arm = (int(v) for v in line.split())
    if os.environ.get('FAKE_SILENT'):
        continue
    print(json.dumps({'cam': idx, 'id': fid, 'arm': arm, 'ms': 1.0, 'img': 'w_test.jpg',
                      'boxes': [{'cls': 'phi', 'conf': 0.9, 'xyxy': [100, 100, 160, 160]}]}),
          flush=True)
