#!/usr/bin/env python3
"""Сырой скан LD19 без фильтров: CSV + сводка по ближним точкам.

Нужен, чтобы увидеть, где именно лидар видит леску ручки: на каких углах,
на каких дальностях, с какой интенсивностью и как часто (доля оборотов).
Ставим робота на открытое место, где в радиусе ~30 см нет стен, и снимаем.

Запуск на малине:  ~/maze/venv/bin/python ~/maze/raw_scan.py -n 30
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from collections import defaultdict

from ld19 import DEFAULT_PORT, BAUDRATE, ZERO_DIR, LD19


def main() -> None:
    ap = argparse.ArgumentParser(description="сырой скан LD19")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT)
    ap.add_argument("-b", "--baud", type=int, default=BAUDRATE)
    ap.add_argument("--zero-dir", type=float, default=ZERO_DIR)
    ap.add_argument("-n", type=int, default=30, help="сколько оборотов снять")
    ap.add_argument("-o", "--out", default="", help="CSV (по умолчанию scan_<время>.csv)")
    ap.add_argument("--near", type=float, default=250.0,
                    help="в сводку идут точки ближе этого, мм")
    ap.add_argument("--bin", type=float, default=5.0, help="ширина углового бина сводки, град")
    args = ap.parse_args()

    out = args.out or time.strftime("scan_%Y%m%d_%H%M%S.csv")
    # бин -> [(rev, dist, intensity)]
    near = defaultdict(list)
    total_rev = 0

    with LD19(args.port, args.baud, zero_dir=args.zero_dir) as lidar, \
            open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rev", "angle", "dist_mm", "intensity"])
        for rev_no, rev in enumerate(lidar.revolutions()):
            for p in rev:
                w.writerow([rev_no, f"{p.angle:.2f}", p.distance, p.intensity])
                if 0 < p.distance <= args.near:
                    near[int(p.angle // args.bin)].append((rev_no, p.distance, p.intensity))
            total_rev = rev_no + 1
            if total_rev >= args.n:
                break

    print(f"снято {total_rev} оборотов -> {out}")
    if not near:
        print(f"точек ближе {args.near:.0f} мм нет")
        return

    print(f"\nточки ближе {args.near:.0f} мм (углы в системе робота: 0 перёд, 90 право, 270 лево)")
    print(f"{'угол':>9} {'точек':>6} {'в обор.':>8} {'дальн. мин/мед/макс, мм':>26} {'интенс. мин/сред/макс':>24}")
    for b in sorted(near):
        pts = near[b]
        dists = [d for _, d, _ in pts]
        ints = [i for _, _, i in pts]
        revs = len({r for r, _, _ in pts})
        lo, hi = b * args.bin, (b + 1) * args.bin
        print(f"{lo:4.0f}-{hi:<4.0f} {len(pts):6d} {revs:4d}/{total_rev:<3d} "
              f"{min(dists):8d}/{statistics.median(dists):6.0f}/{max(dists):<6d}"
              f"{min(ints):9d}/{statistics.mean(ints):5.0f}/{max(ints):<5d}")


if __name__ == "__main__":
    main()
