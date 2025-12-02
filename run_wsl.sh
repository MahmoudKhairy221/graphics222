#!/bin/bash
# Helper script to compile and (optionally) run Chrono Heist inside WSL.
# Usage from Windows PowerShell (with Ubuntu installed):
#   wsl -d Ubuntu -- bash -lc "./run_wsl.sh"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "Compiling Chrono Heist from $SCRIPT_DIR ..."
g++ -std=c++17 -O2 ChronoHeist.cpp -o ChronoHeist -lGL -lGLU -lglut -lm

echo "Compilation successful."

if [ -z "${DISPLAY:-}" ]; then
    cat <<'EOF'
DISPLAY is not set, so the game will not auto-run.
Start your Windows X server (e.g. VcXsrv), export DISPLAY (for example: export DISPLAY=:0),
then run ./ChronoHeist from this directory when ready.
EOF
else
    echo "Running game..."
    ./ChronoHeist
fi

