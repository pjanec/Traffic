#!/usr/bin/env bash
# City3D demo — Xvfb software-rendered screenshot pipeline (task T1.3 part C).
#
# Packs the local feed, builds the Viewer (Debug), resolves the Godot 4 (.NET/mono) engine binary via
# fetch-godot.sh, then runs it UNDER Xvfb with software GL (LIBGL_ALWAYS_SOFTWARE=1,
# GALLIUM_DRIVER=llvmpipe) so a real (CPU-rendered, Mesa llvmpipe) OpenGL context initializes without a
# GPU. Deliberately NOT --headless: Xvfb itself provides the display Godot renders into; Main.cs's
# `--shot=<path>` path (docs/DEMO-CITY3D-TASKS.md T1.3 part C) needs a real rendering driver
# (RenderingServer.FramePostDraw + GetViewport().GetTexture().GetImage()) to produce a non-trivial PNG --
# under --headless (the dummy renderer run-smoke.sh uses) GetImage() returns null and Main.cs reports the
# gap instead of crashing (see run-smoke.sh, which stays a --headless smoke, unrelated to this script).
#
# Usage:
#   demos/City3D/screenshot.sh [output-png-path]     # default: /tmp/city3d-roads.png
#
# The PNG is a scratch artifact (written under /tmp by default) -- never commit it.
#
# Requires: .NET 8 SDK on PATH; xvfb-run; a software GL driver (Mesa llvmpipe) on the system.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
DEMO="$ROOT/demos/City3D"
VIEWER="$DEMO/Viewer"

OUT="${1:-/tmp/city3d-roads.png}"
mkdir -p "$(dirname "$OUT")"
rm -f "$OUT"

echo "==> [1/4] packing the local NuGet feed"
bash "$DEMO/build.sh" --pack-only

echo "==> [2/4] building the Viewer Godot C# assembly (Debug -- what the runtime loads)"
dotnet build "$VIEWER" -c Debug

echo "==> [3/4] resolving the Godot 4 (.NET/mono) engine binary"
GODOT_BIN="$("$DEMO/fetch-godot.sh" | tail -1)"
echo "    godot binary: $GODOT_BIN"

echo "==> [4/4] running the Viewer under Xvfb with software GL, capturing a screenshot to $OUT"
LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT

set +e
LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe xvfb-run -a -s "-screen 0 1600x900x24" \
  "$GODOT_BIN" --path "$VIEWER" --rendering-driver opengl3 --resolution 1600x900 \
  -- --shot="$OUT" > "$LOG" 2>&1
STATUS=$?
set -e

cat "$LOG"

if [[ $STATUS -ne 0 ]]; then
  echo "FAIL: godot exited with status $STATUS"
  exit 1
fi

if [[ ! -s "$OUT" ]]; then
  echo "FAIL: screenshot PNG '$OUT' was not produced (or is empty)"
  exit 1
fi

SIZE_BYTES=$(stat -c%s "$OUT" 2>/dev/null || stat -f%z "$OUT")
if [[ "$SIZE_BYTES" -lt 1024 ]]; then
  echo "FAIL: screenshot PNG '$OUT' is suspiciously small ($SIZE_BYTES bytes) -- likely a blank/failed capture"
  exit 1
fi

echo "PASS: screenshot saved to $OUT ($SIZE_BYTES bytes)"
