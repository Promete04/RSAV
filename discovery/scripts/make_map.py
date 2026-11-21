#!/usr/bin/env python3
"""Turn a run CSV from track.py into a self-contained interactive HTML map.
Usage: make_map.py <run.csv> <out.html>"""
import csv, json, math, sys
src, out = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(src)))
PH = ["run", "hmUsrDock", "hmMidMsn", "hmPostMsn", "stop", "stuck", "charge", "other"]
pts = []
for r in rows:
    ph = r["phase"] if r["phase"] in PH else "other"
    pts.append([float(r["t"]), int(r["x_cm"]), int(r["y_cm"]), int(r["theta_deg"]), PH.index(ph), int(r["batPct"]), int(r["error"] or 0)])
# distance + swept-area estimate (35 cm robot footprint, 5 cm grid, linear interpolation between samples)
dist, cells, R, G = 0.0, set(), 17.5, 5
for a, b in zip(pts, pts[1:]):
    d = math.hypot(b[1] - a[1], b[2] - a[2]); dist += d
    n = max(1, int(d // 2))
    for i in range(n + 1):
        x = a[1] + (b[1] - a[1]) * i / n; y = a[2] + (b[2] - a[2]) * i / n
        for gx in range(int((x - R) // G), int((x + R) // G) + 1):
            for gy in range(int((y - R) // G), int((y + R) // G) + 1):
                if math.hypot(gx * G + G / 2 - x, gy * G + G / 2 - y) <= R: cells.add((gx, gy))
xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
stats = dict(file=src.rsplit("/", 1)[-1], dur=pts[-1][0], n=len(pts), dist_m=round(dist / 100, 1),
             area_m2=round(len(cells) * G * G / 10000, 1), w_m=round((max(xs) - min(xs)) / 100, 1),
             h_m=round((max(ys) - min(ys)) / 100, 1), bat0=pts[0][5], bat1=pts[-1][5])
tpl = open(__file__.replace("make_map.py", "map_template.html")).read()
open(out, "w").write(tpl.replace("__DATA__", json.dumps(pts, separators=(",", ":")))
                         .replace("__PHASES__", json.dumps(PH)).replace("__STATS__", json.dumps(stats)))
print(json.dumps(stats))
