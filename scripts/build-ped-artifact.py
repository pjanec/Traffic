#!/usr/bin/env python3
"""Assemble the pedestrian-demo gallery artifact.

Runs the Sim.Viz --ped-* scenes (plus the P5-1(B) --evac-district scene), base64-embeds each
self-contained HTML into a single gallery page (a scene picker over a viewport-filling iframe),
and writes it to the output path. The gallery is body-content-only (no <html>/<head>/<body>) so it
publishes cleanly as a claude.ai Artifact, which wraps it in its own skeleton.

Usage:  python3 scripts/build-ped-artifact.py <out.html>
Keep the SCENES list in sync with the --ped-*/--evac-district modes in src/Sim.Viz/Program.cs and
the "Pedestrians"/"Panic evacuation" categories in scripts/gen-demos.sh.
"""
import base64
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# (mode flag, chip label, one-line subtitle)
SCENES = [
    ("crossing-gate", "Crossing gate", "Car halts for a pedestrian on its green — emergent, not scripted"),
    ("lod-promotion", "Sim-LOD promotion", "Low-power walkers promoted to full ORCA near an interest source"),
    ("od-routing", "O–D routed crowd", "Poisson demand routed across real sidewalks and crossings"),
    ("dodge", "Obstacle dodge", "A stream arcs through the corridor beside a static box, ORCA-clear"),
    ("reroute", "Crossing reroute", "A blocker appears mid-crossing; the affected ped detours around it"),
    ("parking", "Parking lot", "A car parks among parked cars while pedestrians weave the aisle"),
    ("liveliness", "Liveliness", "Deterministic activity-timeline replay: sip, sit, go inside, re-emerge"),
    ("social", "Meet & talk", "Pre-scheduled two-ped interaction: step aside, face each other, talk, resume"),
    ("waiter", "Waiter", "Templated micro-scenario actor: serves tables in rotation, goes inside between rounds"),
    ("lively-crowd", "Lively crowd", "The routed O-D crowd, now with seeded Pause beats along real routes"),
    ("remote", "Remote (over the wire)", "Reconstructed from the real DR-error-gated multicast stream, not the sim"),
    ("evac-district", "Evac district", "Panicked peds route to the nearest safe zone along real sidewalks, forced high-power"),
]


def gen_demo(mode: str, out: Path) -> str:
    """Render one demo and return its HTML. `--evac-district` (P5-1(B)) is its own top-level
    Sim.Viz flag, not a `--ped-<mode>` mode; every other entry uses the `--ped-<mode>` convention."""
    flag = f"--{mode}" if mode.startswith("evac-") else f"--ped-{mode}"
    subprocess.run(
        ["dotnet", "run", "--project", "src/Sim.Viz", "-c", "Release", "--no-build",
         "--", flag, str(out)],
        cwd=REPO, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    return out.read_text(encoding="utf-8")


def build(out_path: Path) -> None:
    scenes_js = []
    with tempfile.TemporaryDirectory() as td:
        for mode, label, sub in SCENES:
            html = gen_demo(mode, Path(td) / f"{mode}.html")
            b64 = base64.b64encode(html.encode("utf-8")).decode("ascii")
            scenes_js.append(
                "{slug:%r,label:%r,sub:%r,b64:%r}" % (mode, label, sub, b64)
            )
            print(f"  embedded {mode}  ({len(html):,} B html -> {len(b64):,} B base64)")
    data_js = "const SCENES=[\n" + ",\n".join(scenes_js) + "\n];"
    out_path.write_text(PAGE.replace("/*__SCENES__*/", data_js), encoding="utf-8")
    print(f"wrote {out_path}  ({out_path.stat().st_size:,} B)")


PAGE = r"""<title>SumoSharp — Pedestrian demos</title>
<style>
  :root {
    --bg: #0f1116; --surface: #171a21; --surface-2: #1f2430;
    --line: rgba(255,255,255,.09); --line-strong: rgba(255,255,255,.16);
    --text: #e8e8ea; --muted: #868c9c; --accent: #a855f7; --accent-ink: #f5ecff;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --bg: #f4f5f8; --surface: #ffffff; --surface-2: #eef0f5;
      --line: rgba(15,17,22,.10); --line-strong: rgba(15,17,22,.20);
      --text: #1a1d24; --muted: #5b6270; --accent: #7c3aed; --accent-ink: #ffffff;
    }
  }
  :root[data-theme="dark"] {
    --bg: #0f1116; --surface: #171a21; --surface-2: #1f2430;
    --line: rgba(255,255,255,.09); --line-strong: rgba(255,255,255,.16);
    --text: #e8e8ea; --muted: #868c9c; --accent: #a855f7; --accent-ink: #f5ecff;
  }
  :root[data-theme="light"] {
    --bg: #f4f5f8; --surface: #ffffff; --surface-2: #eef0f5;
    --line: rgba(15,17,22,.10); --line-strong: rgba(15,17,22,.20);
    --text: #1a1d24; --muted: #5b6270; --accent: #7c3aed; --accent-ink: #ffffff;
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; height: 100%; }
  body {
    background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
  }
  #app { position: fixed; inset: 0; display: flex; flex-direction: column; }
  header#bar {
    flex: none; display: flex; flex-direction: column; gap: 8px;
    padding: 10px 12px calc(10px) 12px;
    background: var(--surface); border-bottom: 1px solid var(--line);
  }
  .brand { display: flex; align-items: baseline; gap: 9px; min-width: 0; }
  .brand .mark {
    width: 11px; height: 11px; border-radius: 50%; background: var(--accent);
    flex: none; align-self: center; box-shadow: 0 0 0 3px color-mix(in srgb, var(--accent) 22%, transparent);
  }
  .brand .name { font-weight: 650; font-size: 15px; letter-spacing: .01em; white-space: nowrap; }
  .brand .name .dim { color: var(--muted); font-weight: 500; }
  .brand .sub {
    margin-left: auto; font-size: 11px; color: var(--muted);
    white-space: nowrap; overflow: hidden; text-overflow: ellipsis; padding-left: 8px;
  }
  nav#scenes {
    display: flex; gap: 7px; overflow-x: auto; scrollbar-width: thin;
    -webkit-overflow-scrolling: touch; padding-bottom: 2px;
  }
  nav#scenes::-webkit-scrollbar { height: 5px; }
  nav#scenes::-webkit-scrollbar-thumb { background: var(--line-strong); border-radius: 3px; }
  .chip {
    flex: none; display: flex; flex-direction: column; align-items: flex-start; gap: 1px;
    background: var(--surface-2); color: var(--text);
    border: 1px solid var(--line); border-radius: 9px;
    padding: 6px 11px; font: inherit; cursor: pointer; text-align: left;
    transition: border-color .12s ease, background .12s ease, transform .06s ease;
  }
  .chip:hover { border-color: var(--line-strong); }
  .chip:active { transform: translateY(1px); }
  .chip .idx {
    font-size: 9.5px; letter-spacing: .14em; text-transform: uppercase;
    color: var(--muted); font-variant-numeric: tabular-nums;
  }
  .chip .t { font-size: 13px; font-weight: 550; white-space: nowrap; }
  .chip[aria-pressed="true"] {
    background: color-mix(in srgb, var(--accent) 18%, var(--surface-2));
    border-color: var(--accent);
  }
  .chip[aria-pressed="true"] .idx { color: var(--accent); }
  .chip:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
  main#viewport { position: relative; flex: 1 1 auto; min-height: 0; background: var(--bg); }
  iframe#frame { position: absolute; inset: 0; width: 100%; height: 100%; border: 0; display: block; }
  @media (min-width: 720px) {
    header#bar { flex-direction: row; align-items: center; gap: 16px; }
    .brand { flex: none; max-width: 46%; }
    nav#scenes { flex: 1 1 auto; }
  }
</style>

<div id="app">
  <header id="bar">
    <div class="brand">
      <span class="mark" aria-hidden="true"></span>
      <span class="name">SumoSharp <span class="dim">· Pedestrians</span></span>
      <span class="sub" id="brandSub"></span>
    </div>
    <nav id="scenes" aria-label="Pedestrian demo scenes"></nav>
  </header>
  <main id="viewport">
    <iframe id="frame" title="Pedestrian simulation replay" allow="fullscreen"></iframe>
  </main>
</div>

<script>
/*__SCENES__*/

(function () {
  function decode(b64) {
    var bin = atob(b64), bytes = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new TextDecoder("utf-8").decode(bytes);
  }
  var nav = document.getElementById("scenes");
  var frame = document.getElementById("frame");
  var sub = document.getElementById("brandSub");
  var chips = [];

  function select(i) {
    for (var k = 0; k < chips.length; k++) chips[k].setAttribute("aria-pressed", k === i ? "true" : "false");
    frame.srcdoc = decode(SCENES[i].b64);
    sub.textContent = SCENES[i].sub;
    try { chips[i].scrollIntoView({ inline: "center", block: "nearest" }); } catch (e) {}
  }

  SCENES.forEach(function (s, i) {
    var b = document.createElement("button");
    b.className = "chip";
    b.type = "button";
    b.setAttribute("aria-pressed", "false");
    b.innerHTML = '<span class="idx">' + String(i + 1).padStart(2, "0") + "</span>" +
                  '<span class="t"></span>';
    b.querySelector(".t").textContent = s.label;
    b.addEventListener("click", function () { select(i); });
    nav.appendChild(b);
    chips.push(b);
  });

  select(0);
})();
</script>
"""


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    build(Path(sys.argv[1]).resolve())
