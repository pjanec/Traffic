#!/usr/bin/env bash
# City3D demo — one-command local package feed + build.
#
# Populates ./local-nuget by `dotnet pack`-ing the SumoSharp.* packages the demo consumes, then (if the
# Godot Viewer project exists) restores + builds it against that local feed via ./nuget.config. Nothing is
# published; nothing heavy is committed. Re-run any time you change the packages in src/ — this is the
# "iterate on the packages without a round-trip through GitHub" path.
#
# Usage:
#   ./build.sh            # pack the pure-C# packages the LOCAL viewer needs, then restore+build the demo
#   ./build.sh --remote   # ALSO pack SumoSharp.Replication.Dds (native) for the remote/DDS path
#   ./build.sh --pack-only # just (re)populate ./local-nuget, don't touch the demo project
#
# Requires: .NET 8 SDK on PATH. (SUMO is NOT needed — this never runs the engine or the parity loop.)
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
DEMO="$ROOT/demos/City3D"
FEED="$DEMO/local-nuget"

REMOTE=0
PACK_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --remote)    REMOTE=1 ;;
    --pack-only) PACK_ONLY=1 ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

# Pure-managed packages the LOCAL, in-process viewer consumes. No native deps here on purpose — the local
# demo must read as the clean "any C# engine consumes SumoSharp" reference.
PACKAGES=(
  "src/Sim.Core/Sim.Core.csproj"
  "src/Sim.Ingest/Sim.Ingest.csproj"
  "src/Sim.Replication/Sim.Replication.csproj"
  "src/Sim.Viewer.Motion/Sim.Viewer.Motion.csproj"
  "src/Sim.Host/Sim.Host.csproj"
  "src/Sim.Pedestrians/Sim.Pedestrians.csproj"
)
# The remote/DDS path additionally needs the native transport binding.
if [[ "$REMOTE" == "1" ]]; then
  PACKAGES+=("src/Sim.Replication.Dds/Sim.Replication.Dds.csproj")
fi

echo "==> Refreshing local feed: $FEED"
rm -rf "$FEED"
mkdir -p "$FEED"

for proj in "${PACKAGES[@]}"; do
  echo "==> pack $proj"
  dotnet pack "$ROOT/$proj" -c Release -o "$FEED"
done

echo "==> local feed now contains:"
ls -1 "$FEED"/*.nupkg

if [[ "$PACK_ONLY" == "1" ]]; then
  echo "==> --pack-only: done (demo project not built)."
  exit 0
fi

# Build the demo against the local feed, if it exists yet (added in later tasks: demos/City3D/Viewer).
VIEWER="$DEMO/Viewer/Viewer.csproj"
if [[ -f "$VIEWER" ]]; then
  echo "==> restore + build the demo viewer against the local feed"
  dotnet restore "$VIEWER"          # nuget.config in $DEMO pins SumoSharp.* to the local feed
  dotnet build "$VIEWER" -c Release --no-restore
  echo "==> demo viewer built OK."
else
  echo "==> (no demos/City3D/Viewer project yet — feed populated; skipping demo build.)"
fi
