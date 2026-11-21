#!/usr/bin/env bash
# Start rest980 (API on http://127.0.0.1:3000, localhost only)
set -e; cd "$(dirname "$0")/.."
podman rm -f rest980 >/dev/null 2>&1 || true
podman run -d --name rest980 --label project=roomba \
  --env-file config/rest980.env -p 127.0.0.1:3000:3000 \
  docker.io/koalazak/rest980:latest
