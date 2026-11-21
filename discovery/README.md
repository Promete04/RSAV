# Discovery phase (AI-assisted)

> This folder is the **exploration** part of RSAV. It was built with an AI coding
> assistant (Claude Code) to learn how the Roomba 980 can be reached and controlled.
> The code is a prototype for learning. It runs on a Linux PC, and the ESP32 firmware
> will be designed separately.

Robot: Roomba 900 series (980, SKU R976040).

## Layout

    config/rest980.env.example   template for robot credentials (real rest980.env is git-ignored)
    scripts/start.sh             (re)start the rest980 podman container (API on 127.0.0.1:3000)
    scripts/stop.sh              stop and remove the container (config and image are kept)
    scripts/cleanup.sh           remove the container and the image
    scripts/state.sh [-s]        print the full robot state as JSON (-s also saves it to data/)
    scripts/discover.py          UDP discovery (was blocked by firewalld; used a port scan instead)
    scripts/get_password.py      get the local password (robot docked, hold HOME for about 2 s)
    scripts/track.py             log position and phase every 1 s to data/run-*.csv until the robot is charging again
    scripts/make_map.py          turn a run CSV into an interactive HTML map (uses map_template.html)
    scripts/control.sh           web remote control over the micro-USB Open Interface (webapp/)
    webapp/                      server.py (Python standard library only) and index.html
    examples/python/             small OI driver and motion examples
    examples/c/                  portable OI protocol code and a Linux command-line tool
    examples/esp32/              ESP-IDF USB-host firmware for ESP32-S3 (not tested yet)
    data/                        your runs and state snapshots (git-ignored: they contain Wi-Fi info and the shape of the house)

## Path 1: Wi-Fi, through the local API (rest980)

    python3 scripts/get_password.py <robot_ip> <blid> config/rest980.env
    scripts/start.sh
    curl -s http://127.0.0.1:3000/api/local/info/mission

Useful endpoints:

    GET  /api/local/info/state     everything (54 keys)
    GET  /api/local/info/mission   phase, battery, bin, position
    GET  /api/local/action/start | stop | pause | resume | dock

Notes:
- The 980 allows only **one** local MQTT client. KEEP_ALIVE=yes keeps that connection
  while the container runs, so run `stop.sh` if another local client needs the robot.
- rest980 flags take yes/no, not true/false.
- This path only starts, stops and docks cleaning missions. It **cannot drive the wheels directly**.

Map a run:

    scripts/track.py &   then   curl -s http://127.0.0.1:3000/api/local/action/start
    scripts/make_map.py data/run-XXXX.csv data/map-XXXX.html

## Path 2: Open Interface over micro-USB (direct control)

    scripts/control.sh          then open http://127.0.0.1:8980
    scripts/control.sh --lan    reachable from a phone on the same Wi-Fi (no password!)

- Port: `/dev/ttyACM0`, USB ID 27a6:0002 (CDC-ACM), 115200 baud. The port carries the binary OI and a text log on the same line.
- The wheels stop by themselves if the page stops sending commands for 0.35 s.
- Ctrl+C on the server puts the robot back in Passive mode and closes the Open Interface.
- Only one program can use the serial port at a time.

This is the path the ESP32 will take over: the OI gives direct control of the wheels plus the sensor stream.
