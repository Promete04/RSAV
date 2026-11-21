#!/usr/bin/env python3
"""Find iRobot Roombas on the LAN (UDP broadcast 'irobotmcs' on port 5678)."""
import json, socket, sys, time
targets = sys.argv[1:] or ["255.255.255.255"]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.settimeout(1.0)
for t in targets:
    s.sendto(b"irobotmcs", (t, 5678))
seen, end = set(), time.time() + 5
while time.time() < end:
    try:
        data, addr = s.recvfrom(4096)
    except socket.timeout:
        continue
    if data == b"irobotmcs" or addr[0] in seen:
        continue
    seen.add(addr[0])
    try:
        print(json.dumps(json.loads(data), indent=2))
    except ValueError:
        print(addr[0], data[:200])
if not seen:
    print("no robots answered", file=sys.stderr); sys.exit(1)
