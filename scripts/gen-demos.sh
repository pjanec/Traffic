#!/usr/bin/env bash
# gen-demos.sh
# ------------
# Single source of truth for the auto-generated interactive-demo gallery: builds a CURATED set of
# self-contained `replay.html`-style pages (vanilla Canvas 2D, no server, no SUMO — see
# `src/Sim.Viz`) into `site/<slug>.html`, plus a `site/index.html` landing page. Both the
# `demos` GitHub Actions workflow (deploy to Pages) and humans run exactly this script — see
# docs/DEMOS.md.
#
#   scripts/gen-demos.sh
#   open site/index.html
#
# Never modifies committed scenario files: any `replay.html` the tools write into a scenario dir
# is copied out to `site/` and then restored to its committed content (`git checkout --`); any
# other regenerated scenario artifact (`engine.fcd.xml`, ...) is deleted afterwards. Resilient: a
# single broken demo is SKIPped (logged, not faked) and does not abort the rest of the gallery.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

SITE="$ROOT/site"
LOGDIR="$SITE/.logs"
rm -rf "$SITE"
mkdir -p "$SITE" "$LOGDIR"

echo "==> Building Sim.Viz / Sim.Run / Sim.ExtDemo (Release)…"
dotnet build src/Sim.Viz -c Release -v q
dotnet build src/Sim.Run -c Release -v q
dotnet build src/Sim.ExtDemo -c Release -v q

run() { dotnet run -c Release --no-build --project "$1" -- "${@:2}"; }

produced_slugs=()
produced_titles=()
produced_descs=()
skipped_slugs=()
skipped_reasons=()

# try <slug> <title> <description> <fn>
# Runs <fn>; on success records <slug>/<title>/<description> as produced, on failure records a SKIP.
# <fn>'s own commands run under `set -e`, but as the tested command of this `if`, a failure partway
# through does not abort the whole script — only that one demo is skipped.
try() {
  local slug="$1" title="$2" desc="$3" fn="$4"
  local log="$LOGDIR/$slug.log"
  if "$fn" >"$log" 2>&1; then
    echo "OK   $slug"
    produced_slugs+=("$slug")
    produced_titles+=("$title")
    produced_descs+=("$desc")
  else
    echo "SKIP $slug (generation failed — see $log)"
    skipped_slugs+=("$slug")
    skipped_reasons+=("generation failed")
  fi
}

# --- demo bodies -----------------------------------------------------------------------------
# Each renders straight from committed scenario inputs (net + golden FCD, or the engine run fresh)
# and never leaves the scenarios/ tree modified.

demo_single_free_flow() {
  local d="scenarios/01-single-free-flow"
  run src/Sim.Viz "$d"
  cp "$d/replay.html" "$SITE/single-free-flow.html"
  git checkout -- "$d/replay.html"
}

demo_traffic_light() {
  local d="scenarios/09-traffic-light"
  run src/Sim.Viz "$d"
  cp "$d/replay.html" "$SITE/traffic-light.html"
  git checkout -- "$d/replay.html"
}

demo_priority_junction() {
  local d="scenarios/11-priority-junction"
  run src/Sim.Viz "$d"
  cp "$d/replay.html" "$SITE/priority-junction.html"
  git checkout -- "$d/replay.html"
}

demo_external_agents() {
  local d="scenarios/_bench/ext-showcase"
  run src/Sim.ExtDemo "$d"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/external-agents.html"
  rm -f "$d/replay.html" "$d/engine.fcd.xml" "$d/combined.fcd.xml" "$d/playwright_screenshot.png"
}

demo_evac_organic() {
  run src/Sim.Viz --evac-organic "$SITE/evac-organic.html"
}

demo_city_30() {
  local d="scenarios/_bench/city-30"
  run src/Sim.Run "$d"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/city-30.html"
  git checkout -- "$d/replay.html"
  rm -f "$d/engine.fcd.xml"
}

# --- run the curated set ----------------------------------------------------------------------

echo "==> Generating curated demo gallery…"
try single-free-flow  "Single free-flow"          "A single vehicle cruising free-flow on an open road — the simplest parity scenario." demo_single_free_flow
try traffic-light      "Traffic light"             "Vehicles queuing and releasing at a signalized intersection with SUMO-native signal heads." demo_traffic_light
try priority-junction  "Priority junction"         "Right-of-way negotiation at an unsignalized priority junction." demo_priority_junction
try external-agents    "External agents showcase" "Five external (non-SUMO) agent reactions — stop, swerve, spill, follow, junction-yield — injected alongside engine traffic." demo_external_agents
try evac-organic       "Evacuation (organic town)" "A realistic organic town under panic evacuation: congestion plus a large local foot exodus." demo_evac_organic
try city-30            "City-30 (scaled town)"     "A 3x3-grid town at ~30 concurrent vehicles — engine run rendered against the SUMO aggregate-parity reference." demo_city_30

# --- summary ---------------------------------------------------------------------------------

echo ""
echo "==> Summary: ${#produced_slugs[@]} produced, ${#skipped_slugs[@]} skipped."
for i in "${!produced_slugs[@]}"; do echo "  OK   ${produced_slugs[$i]}"; done
for i in "${!skipped_slugs[@]}"; do echo "  SKIP ${skipped_slugs[$i]} (${skipped_reasons[$i]})"; done

if [ "${#produced_slugs[@]}" -eq 0 ]; then
  echo "::error::no demos were produced — refusing to write an empty gallery"
  exit 1
fi

# --- index.html --------------------------------------------------------------------------------

echo "==> Writing site/index.html…"

{
  cat <<'HTML_HEAD'
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SumoSharp — interactive demo gallery</title>
<style>
  :root { color-scheme: light dark; }
  body {
    margin: 0; padding: 2.5rem 1.5rem 4rem; min-height: 100vh;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: #f6f7f9; color: #1b1f24;
  }
  @media (prefers-color-scheme: dark) {
    body { background: #14171a; color: #e6e8eb; }
  }
  header { max-width: 960px; margin: 0 auto 2.5rem; }
  h1 { font-size: 1.9rem; margin: 0 0 0.5rem; }
  p.lede { margin: 0; opacity: 0.75; line-height: 1.5; }
  .grid {
    max-width: 960px; margin: 0 auto; display: grid;
    grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 1.1rem;
  }
  a.card {
    display: block; padding: 1.25rem 1.35rem; border-radius: 0.75rem; text-decoration: none;
    color: inherit; background: #ffffff; border: 1px solid rgba(0,0,0,0.08);
    box-shadow: 0 1px 2px rgba(0,0,0,0.04); transition: transform 0.12s ease, box-shadow 0.12s ease;
  }
  @media (prefers-color-scheme: dark) {
    a.card { background: #1e2226; border-color: rgba(255,255,255,0.08); box-shadow: none; }
  }
  a.card:hover { transform: translateY(-2px); box-shadow: 0 6px 16px rgba(0,0,0,0.12); }
  a.card h2 { margin: 0 0 0.4rem; font-size: 1.05rem; }
  a.card p { margin: 0; font-size: 0.9rem; opacity: 0.75; line-height: 1.4; }
  footer { max-width: 960px; margin: 2.5rem auto 0; font-size: 0.8rem; opacity: 0.6; }
  footer a { color: inherit; }
</style>
</head>
<body>
<header>
  <h1>SumoSharp — interactive demo gallery</h1>
  <p class="lede">
    Self-contained, browser-only traffic-simulation replays (vanilla Canvas 2D — no install, no
    server, no SUMO required). Click a demo to open it; each page has its own play / pause / scrub
    / speed / zoom &amp; pan controls.
  </p>
</header>
<div class="grid">
HTML_HEAD

  for i in "${!produced_slugs[@]}"; do
    slug="${produced_slugs[$i]}"
    title="${produced_titles[$i]}"
    desc="${produced_descs[$i]}"
    printf '  <a class="card" href="%s.html">\n    <h2>%s</h2>\n    <p>%s</p>\n  </a>\n' "$slug" "$title" "$desc"
  done

  cat <<'HTML_TAIL'
</div>
<footer>
  Generated by <code>scripts/gen-demos.sh</code> — see <code>docs/DEMOS.md</code> in the repo.
</footer>
</body>
</html>
HTML_TAIL
} > "$SITE/index.html"

echo ""
echo "Done. Produced ${#produced_slugs[@]} demo(s) in $SITE/."
echo "open $SITE/index.html"
echo "(CI deploys this same directory to GitHub Pages — see .github/workflows/demos.yml)"
