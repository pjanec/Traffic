#!/usr/bin/env bash
# City3D demo — fetch the Godot 4 (.NET/mono) engine binary EPHEMERALLY (never committed).
#
# The engine binary is heavy (~100 MB) and platform-specific, so it is downloaded by tooling on demand
# (committed-vs-ephemeral rule; the repo stays lean). We pull from downloads.godotengine.org — the
# project's own (non-GitHub) mirror — because a cloud session's GitHub token is repo-scoped and gets
# auto-injected into github.com requests, which makes GitHub-hosted release downloads 403. The mirror
# host has no such issue.
#
# Prints the absolute path of the runnable editor binary on the LAST line, and writes it to
# $GODOT_HOME/BIN_PATH so run scripts can `source`/read it. Idempotent: skips the download if already
# present (pass --force to re-download).
#
# Usage:
#   demos/City3D/fetch-godot.sh              # fetch if missing, print binary path
#   GODOT_HOME=/opt/godot demos/City3D/fetch-godot.sh
#   eval "export GODOT_BIN=$(demos/City3D/fetch-godot.sh | tail -1)"
set -euo pipefail

GODOT_VERSION="${GODOT_VERSION:-4.7.1-stable}"
GODOT_FLAVOR="${GODOT_FLAVOR:-mono_linux_x86_64}"     # .NET (mono) build, Linux x86_64
GODOT_HOME="${GODOT_HOME:-/opt/godot}"                # OUTSIDE the repo by default (ephemeral)
FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

slug_ver="${GODOT_VERSION%-stable}"                   # 4.7.1
dir="$GODOT_HOME/Godot_v${GODOT_VERSION}_${GODOT_FLAVOR}"
# the executable inside the zip uses a dot before the arch: ..._mono_linux.x86_64
bin="$dir/Godot_v${GODOT_VERSION}_mono_linux.x86_64"

if [[ "$FORCE" == "0" && -x "$bin" ]]; then
  echo "==> Godot already present: $bin" >&2
  echo "$bin"
  exit 0
fi

url="https://downloads.godotengine.org/?version=${slug_ver}&flavor=stable&slug=${GODOT_FLAVOR}.zip"
tmp="$(mktemp -d)"
echo "==> downloading Godot ${GODOT_VERSION} (${GODOT_FLAVOR}) from downloads.godotengine.org" >&2
curl -fL --retry 3 -o "$tmp/godot.zip" "$url"
echo "==> extracting to $GODOT_HOME" >&2
mkdir -p "$GODOT_HOME"
rm -rf "$dir"
unzip -q "$tmp/godot.zip" -d "$GODOT_HOME"
rm -rf "$tmp"
chmod +x "$bin"

echo "==> verifying headless launch" >&2
"$bin" --headless --version >&2

echo "$bin"
