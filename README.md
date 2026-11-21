# RSAV: Roomba Semi-Autonomous Vehicle

Turning an iRobot Roomba 980 into a semi-autonomous vehicle you can drive by hand,
controlled by an ESP32.

## Objectives

1. **ESP32 controller**: program an ESP32 to talk to the Roomba through its Open Interface (OI).
2. **PlayStation controller over Bluetooth**: drive the robot directly with a DualShock/DualSense paired to the ESP32.
3. **Wi-Fi control**: drive the robot from a phone or PC on the same network through a web page served by the ESP32.
4. **Camera (stretch goal)**: add a camera so the robot can be driven remotely with a live video feed.
5. **Semi-autonomy**: keep the robot's own safety reflexes (bumpers, cliff and wheel-drop sensors) active while it is driven by hand, and add simple automatic moves (go straight X cm, turn X°, return to dock).

## Status

The project is in the **discovery phase**. Nothing runs on the ESP32 yet.

### Discovery (AI-assisted)

The [`discovery/`](discovery/) folder holds the first round of exploration. I did it with
an AI coding assistant (Claude Code) to learn how the robot can be reached and
controlled before I design the real firmware. The code is a prototype, written to learn,
not the final architecture. It runs on a Linux PC, not on the ESP32.

### What has been tested

| Area | Result |
|---|---|
| **Local Wi-Fi API** (rest980 in a podman container, local MQTT, no cloud) | ✅ Works. Got the robot password, read the full state, and sent start, stop, pause, resume and dock. |
| **Tracking a cleaning run** | ✅ Logged pose, phase and battery once a second for three runs, and rendered one as an interactive HTML map (distance, swept area, battery, error events). |
| **LAN discovery** (UDP broadcast) | ⚠️ Blocked by the firewall on the PC. Found the robot by port scan instead. |
| **Open Interface over micro-USB** (`/dev/ttyACM0`, USB ID 27a6:0002, 115200 baud) | ✅ The robot accepts binary OI commands and streams sensor data. A text debug log shares the same line. |
| **Web remote control from a PC or phone** (`webapp/`) | ✅ Manual driving with a dead-man watchdog (wheels stop 0.35 s after the last command), precise moves using the wheel encoders, and stops on bumps or wheel drops. |
| **Python and C OI libraries** (`examples/python`, `examples/c`) | ✅ Built on Linux. Includes examples for sensors, forward and back moves, turns, a square, a circle and a beep. |
| **ESP32-S3 USB-host firmware** (`examples/esp32`) | ❌ Written but **not built or flashed** yet. |

## Open questions for the ESP32 phase

- **How the ESP32 reaches the robot.** The board is an ESP32 DevKit V1 (original ESP32).
  It has Bluetooth Classic, which a PlayStation controller needs (for example with
  [Bluepad32](https://github.com/ricardoquesada/bluepad32)), and Wi-Fi. It has **no USB host**,
  though. Its USB port goes to a USB-to-serial chip and acts as a device, just like the
  Roomba's micro-USB port. Connecting the two by cable does nothing, because USB needs one
  host and one device.

  **Recommended: add a MAX3421E USB host module** ("USB Host Shield" or "mini USB host") on SPI.
  The [USB Host Shield 2.0](https://github.com/felis/USB_Host_Shield_2.0) library supports the
  ESP32 and has a CDC-ACM driver for USB serial devices like the Roomba. This keeps the
  micro-USB link that has already been tested, and `oi_proto.c` can be reused as is.

  ```
  PS controller ──BT Classic──▶ ESP32 DevKit ──SPI──▶ MAX3421E ──USB──▶ Roomba micro-USB
  phone / PC    ──Wi-Fi──────▶
  ```

  The module must supply 5 V on its USB port (from the board or from a buck converter).

  Other options:
  - a serial (UART) connection to the Roomba, if the 980 has a serial port (needs a 5 V ↔ 3.3 V level shifter);
  - two boards: this ESP32 for the controller and Wi-Fi, plus an ESP32-S3 for USB host, linked by ESP-NOW or UART;
  - an ESP32-S3 alone: USB host and Wi-Fi, but only Bluetooth Low Energy, so no DualShock 4 or DualSense.
- **Power**: run the ESP32 (and later the camera) from the Roomba battery through a buck converter.
- **Camera**: an ESP32-CAM or ESP32-S3 camera module sending MJPEG over Wi-Fi, or a separate camera board.
- **Security**: the Wi-Fi remote must require authentication before it leaves the local network.

## Repository layout

```
discovery/     AI-assisted exploration: Wi-Fi API, USB Open Interface, web remote, examples
LICENSE        MIT
```

## Safety

The robot moves. When you test motion commands, keep space around it and keep it away
from stairs. Keep a way to stop it within reach (the web remote's stop button, Ctrl+C, or
picking the robot up, which triggers the wheel-drop sensors).

## License

[MIT](LICENSE)
