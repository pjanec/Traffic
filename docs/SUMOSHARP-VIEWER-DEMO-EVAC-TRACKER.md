# Native viewer demo tool + live evac — tracker

Checklist for `docs/SUMOSHARP-VIEWER-DEMO-EVAC-TASKS.md` (design: `…-DESIGN.md`). A box is ticked ONLY
after Opus confirms the task's success conditions first-hand (diff read + gate re-run + Xvfb screenshot).

Standing gate on every tick: `dotnet test Traffic.sln` = 446/3/0 · `Sim.Bench` hash `909605E965BFFE59`
(single + parallel).

## Stage A — seam + evac data path
- [x] **T1** `SimulationRunner.OnAfterStep` hook (in-solution; byte-identical hash + suite) — Opus-verified: hook inert, gate 446/3/0, hash `909605E965BFFE59` single+parallel.
- [x] **T2** `EvacRenderSnapshot` + `EngineHost` evac path — Opus-verified: harness re-run first-hand, cascade fires for grid-tls (panic→abandon→peds), organic (39 panicked/32 peds), city (472 panicked, 5090 fear-tracked @10k).

## Stage B — catalog + switching
- [x] **T3** `DemoCatalog` — Opus-verified: 25 usable entries, 7 categories, all 3 evac kinds, rbl-left-turns present.
- [x] **T4** live demo switching + `--demo` — Opus-verified first-hand: --demo-smoke (Priority→Traffic-light→Evac, distinct edge counts, evac present, clean) + Xvfb `--demo "Roundabout"` screenshot renders the net + vehicle + HUD; ad-hoc `--mode local <path>` path preserved.

## Stage C — rendering + UI
- [ ] **T5** ImGui "Demos" picker + non-evac polish
- [ ] **T6** evac draw pass (incident zone / peds / abandoned / pushers / fear tint) + place-incident click

## Stage D — close-out
- [ ] **T7** docs (native-viewer doc + README) + final gate re-confirm + screenshots attached

Status: **Batches 1–2 (T1–T4) landed + Opus-verified. Batch 3 (T5–T6, ImGui picker + evac render) next.**
