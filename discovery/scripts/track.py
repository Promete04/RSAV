#!/usr/bin/env python3
"""Log Roomba pose + mission state once a second to data/run-<timestamp>.csv.
Stops when back on the dock charging (cycle 'none', phase 'charge'), on 'stop' phase >5 min, or after 3 h."""
import csv, json, os, sys, time, urllib.request
URL = "http://127.0.0.1:3000/api/local/info/state"
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out = os.path.join(root, "data", time.strftime("run-%Y-%m-%d_%H%M%S.csv"))
f = open(out, "w", newline=""); w = csv.writer(f)
w.writerow(["t", "x_cm", "y_cm", "theta_deg", "phase", "cycle", "batPct", "sqft", "mssnM", "error", "rssi"])
print("logging to", out, flush=True)
t0, seen_run, stop_since, last = time.time(), False, None, None
while time.time() - t0 < 3 * 3600:
    try:
        d = json.load(urllib.request.urlopen(URL, timeout=10))
    except Exception as e:
        print("poll error:", e, flush=True); time.sleep(2); continue
    p, m = d.get("pose", {}), d.get("cleanMissionStatus", {})
    row = [round(time.time() - t0, 1), p.get("point", {}).get("x"), p.get("point", {}).get("y"),
           p.get("theta"), m.get("phase"), m.get("cycle"), d.get("batPct"), m.get("sqft"),
           m.get("mssnM"), m.get("error"), d.get("signal", {}).get("rssi")]
    w.writerow(row); f.flush()
    if (m.get("phase"), m.get("cycle")) != last:
        print(f"[{row[0]:7.0f}s] phase={m.get('phase')} cycle={m.get('cycle')} bat={d.get('batPct')}% err={m.get('error')}", flush=True)
        last = (m.get("phase"), m.get("cycle"))
    if m.get("cycle") not in (None, "none"): seen_run = True
    # a user 'dock' briefly reports cycle=none/phase=stop before a new 'dock' cycle,
    # so only finish once the robot is back charging on the dock
    if seen_run and m.get("cycle") == "none" and m.get("phase") == "charge":
        print("mission finished, robot charging"); break
    stop_since = (stop_since or time.time()) if m.get("phase") == "stop" else None
    if stop_since and time.time() - stop_since > 300:
        print("stopped for 5 min, ending log"); break
    time.sleep(1)
f.close(); print("saved", out)
