#!/usr/bin/env bash
# Expand a xacro file and check that the result is a valid URDF.
# Usage: check_urdf.sh <file.urdf.xacro> [xacro args...]
set -euo pipefail

urdf="$(mktemp --suffix=.urdf)"
trap 'rm -f "$urdf"' EXIT

xacro "$@" > "$urdf"
check_urdf "$urdf"
