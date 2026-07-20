# Weave demo (gallery source)

`index.html` is a self-contained page demonstrating the deterministic pedestrian weave (PED-REALISM-1 /
`docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md`). No build step, no dependencies, no network.

- **`index.html`** — the on/off corridor demo: two counterflowing streams; toggle the weave to see the
  pass-through count collapse, with density + sidewalk-width sliders. Runs a faithful in-browser port of the
  engine's `Sim.Pedestrians.Lod.LateralWeave` (SplitMix64 seeded), self-checked against the C# engine on load.

The **city demo** — a "realworld" intersection with cars driving cross-traffic AND a routed pedestrian crowd
weaving on the real sidewalks/crossings — is **not** a static page here: it is a native Sim.Viz replay scene,
`SceneGen.BuildWeaveCity` (`Sim.Viz --ped-weave-city`), built the same way as every other gallery demo (a
`ScenePayload` rendered through the shared replay template). See that scene / the `ped-weave-city` gallery entry.

## How they reach GitHub Pages

The Pages site is the **auto-generated demo gallery** (`scripts/gen-demos.sh` → `site/`, deployed by the
`demos` GitHub Actions workflow — see `docs/DEMOS.md`), *not* this folder directly. `index.html` is registered
in `gen-demos.sh` under the **Pedestrians** category via `demo_static` (copied to `site/ped-weave.html`); the
city demo is registered right after it via `demo_ped weave-city ped-weave-city` (rendered to
`site/ped-weave-city.html`). To regenerate locally: `scripts/gen-demos.sh` then open `site/index.html`.

Because each ped's pose is a pure function of `(route, seed, width, time)`, both pages replay identically on
reload — the same determinism that makes `server == image-generator` bit-for-bit over the wire.
