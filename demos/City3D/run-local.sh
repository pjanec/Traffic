#!/usr/bin/env bash
# City3D demo — launch the LOCAL single-viewport viewer (task T1.7).
#
# The public-facing local demo: engine co-hosted in-process, published into an InMemoryReplicationBus,
# reconstructed via SumoSharp.Viewer.Motion, rendered by Godot. Consumes SumoSharp.* from the local feed.
#
# On a desktop (DISPLAY set) this opens an interactive window. In a headless environment it auto-wraps in
# xvfb-run + software GL so it still runs (and can screenshot). Pass any Main.cs user args through after
# the script's own flags.
#
# Usage:
#   ./run-local.sh                                   # default scenario (09-traffic-light), interactive
#   ./run-local.sh --scenario=_bench/city-mixed-1k   # a bigger signalized city (~1k vehicles)
#   ./run-local.sh --scenario=_bench/city-organic --camera=close
#   ./run-local.sh --scenario=_bench/city-30 --shot=/tmp/city30.png --frames=120   # headless-friendly capture
#
# Flags handled here: --frames=<N> (append --quit-after and --fixed-fps for a bounded/headless run).
# Everything else (--scenario=, --camera=, --shot=, --shot-delay=) is passed through to Main.cs.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
DEMO="$ROOT/demos/City3D"
VIEWER="$DEMO/Viewer"

FRAMES=""
PASS_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --frames=*) FRAMES="${arg#--frames=}" ;;
    *) PASS_ARGS+=("$arg") ;;
  esac
done

echo "==> packing local feed + building the Viewer (Debug -- what Godot's runtime loads)"
bash "$DEMO/build.sh" --pack-only >/dev/null
dotnet build "$VIEWER" -c Debug >/dev/null
GODOT_BIN="$("$DEMO/fetch-godot.sh" | tail -1)"

GODOT_ARGS=(--path "$VIEWER")
BOUNDED=()
if [[ -n "$FRAMES" ]]; then
  BOUNDED=(--fixed-fps 60 --quit-after "$FRAMES")
fi

# User args (--scenario=, --camera=, --shot=, --shot-delay=, ...) go after Godot's own `--`.
USER_ARGS=()
if [[ ${#PASS_ARGS[@]} -gt 0 ]]; then
  USER_ARGS=(-- "${PASS_ARGS[@]}")
fi

if [[ -n "${DISPLAY:-}" ]]; then
  echo "==> DISPLAY=$DISPLAY -> interactive window"
  exec "$GODOT_BIN" "${GODOT_ARGS[@]}" "${BOUNDED[@]}" "${USER_ARGS[@]}"
else
  echo "==> no DISPLAY -> headless via Xvfb + software GL (llvmpipe)"
  exec env LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
    xvfb-run -a -s "-screen 0 1600x900x24" \
    "$GODOT_BIN" "${GODOT_ARGS[@]}" --rendering-driver opengl3 --resolution 1600x900 \
    "${BOUNDED[@]}" "${USER_ARGS[@]}"
fi
