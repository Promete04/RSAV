#!/usr/bin/env bash
# Start the Roomba web control (USB Open Interface). Add --lan to use it from a phone.
exec python3 -I "$(dirname "$0")/../webapp/server.py" "$@"
