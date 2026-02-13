#!/usr/bin/env bash
set -euo pipefail

can_port="${1:-can0}"

sudo ip link set "${can_port}" down
sudo ip link set "${can_port}" type can bitrate 1000000
sudo ip link set "${can_port}" up

echo "CAN ${can_port} configured (bitrate 1,000,000)."
