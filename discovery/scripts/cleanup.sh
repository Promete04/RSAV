#!/usr/bin/env bash
# Remove the rest980 container and image (repo files and config are kept)
podman rm -f rest980 2>/dev/null
podman rmi docker.io/koalazak/rest980:latest 2>/dev/null
echo "rest980 container and image removed."
