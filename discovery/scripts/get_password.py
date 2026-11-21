#!/usr/bin/env python3
"""Fetch a Roomba 900/i-series local MQTT password.
Usage: get_password.py <robot_ip> <blid> <env_file>
Robot must be on the dock; hold HOME ~2 s until it beeps / Wi-Fi light blinks."""
import socket, ssl, sys, time, os
ip, blid, out = sys.argv[1:4]
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
ctx.set_ciphers("AES128-SHA256:AES256-SHA:@SECLEVEL=0")
ctx.maximum_version = ssl.TLSVersion.TLSv1_2
ctx.options |= getattr(ssl, "OP_LEGACY_SERVER_CONNECT", 0x4)
MAGIC = bytes.fromhex("f005efcc3b2900")
deadline = time.time() + 120
print("waiting for HOME-button pairing mode (120 s)...", flush=True)
while time.time() < deadline:
    try:
        with socket.create_connection((ip, 8883), timeout=5) as raw, ctx.wrap_socket(raw) as s:
            s.sendall(MAGIC); s.settimeout(3); buf = b""
            try:
                while True:
                    d = s.recv(1024)
                    if not d: break
                    buf += d
            except socket.timeout:
                pass
        i = buf.find(bytes.fromhex("efcc3b29"))
        if i >= 0 and len(buf) > i + 5 + 4:
            pw = buf[i + 5:].split(b"\x00")[0].decode("latin1")
            fd = os.open(out, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as f:
                f.write(f"BLID={blid}\nPASSWORD={pw}\nROBOT_IP={ip}\nFIRMWARE_VERSION=2\n"
                        "ENABLE_LOCAL=true\nENABLE_CLOUD=false\nKEEP_ALIVE=false\n")
            print(f"OK: password ({len(pw)} chars) saved to {out}"); sys.exit(0)
    except (OSError, ssl.SSLError) as e:
        last = e
    time.sleep(2)
print("timed out - HOME button not pressed (or robot not docked)", file=sys.stderr); sys.exit(1)
