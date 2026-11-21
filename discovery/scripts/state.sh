#!/usr/bin/env bash
# Print the robot's full raw state; with -s also save a timestamped copy in data/
cd "$(dirname "$0")/.."
J=$(curl -s -m 20 http://127.0.0.1:3000/api/local/info/state) || exit 1
[ "$1" = "-s" ] && echo "$J" > "data/state-$(date +%F_%H%M%S).json"
echo "$J" | python3 -m json.tool
