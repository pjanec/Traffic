# CLAUDE.md — Operating rules for coding sessions

This repo reimplements SUMO's microscopic traffic-simulation algorithms in C# / .NET 8
(ECS, data-oriented, parallel-ready) with **behavioral parity to SUMO** as the
non-negotiable correctness bar. Performance matters, but a faster wrong answer is still
wrong. Read `DESIGN.md` for the architecture of record; read `TASKS.md` for the current
work queue. This file is the rules of the road.

## Prime directives

1. **Work from the repo root.** Resolve it with `git rev-parse --show-toplevel`. Never
   hardcode an absolute VM path — the VM is volatile and its mount path is not stable.

2. **The VM is volatile. Only committed files persist.** If something must survive the
   session, it must be committed. Build artifacts, NuGet caches, and the SUMO install are
   all ephemeral and must never be relied upon by the test loop.

3. **Parity tolerance is the iron law.** No change — especially no performance
   optimization — may push any scenario's trajectory outside its committed
   `tolerance.json`. If a change moves a scenario out of tolerance, it is reverted or
   gated behind an explicit opt-in "fast mode" flag, never silently accepted.

4. **Follow SUMO on anything behavioral; deviate only where ECS parallelism structurally
   forces it.** Copy SUMO's algorithms and their calculation ordering faithfully. Rebuild
   only the memory layout and the *timing of structural mutations* (deferred to a command
   buffer). When in doubt, read the vendored source and match it.

## Environment bootstrapping

The .NET 8 SDK itself is **not committed** — it is ephemeral, provisioned by the cloud
environment's setup script via `apt-get install -y dotnet-sdk-8.0`, and reinstalled from
scratch on every fresh VM. Microsoft's `dotnet-install.sh` endpoint
(`builds.dotnet.microsoft.com`) is blocked by the egress proxy policy; use the Ubuntu
archive mirror through `apt` instead. Never rely on the SDK being pre-committed or
pre-existing in the offline test loop — a fresh session gets it from the setup script, not
from the repo.

## The committed-vs-ephemeral split (memorize this)

**Committed (the project — survives VM death):**
- all C# source, the harness, the test projects
- scenario *inputs*: `*.net.xml`, `*.rou.xml`, `*.sumocfg`
- golden *outputs*: `golden.fcd.xml`, `golden.state.xml`, `provenance.txt`
- per-scenario `tolerance.json`
- vendored SUMO source at `/sumo/` (read-only reference)
- `SUMO_VERSION`, the `scripts/`, this file, `DESIGN.md`, `TASKS.md`, `.claude/`

**Ephemeral (regenerated, never trusted by tests):**
- the pip-installed SUMO binary
- `bin/`, `obj/`, `dotnet` build output
- the NuGet restore cache

## Two loops, kept strictly separate

**Offline test loop (constant, no network):**
```
dotnet test
```
This runs the engine against committed goldens. **SUMO is NOT needed here.** Never try to
install SUMO or reach the network inside this loop — it will stall.

**Golden regeneration (rare, network-enabled, ends in a commit):**
```
scripts/regen-goldens.sh      # installs SUMO fresh, regenerates FCD + state + provenance
git add ... && git commit      # goldens are committed, not computed at test time
```
Only run this when scenario inputs change or the SUMO version is bumped. Goldens are
**regenerated and committed**, never produced on the fly during testing.

## SUMO source and version

- `/sumo/` is the **read-only** algorithm reference. Port from it; never edit it.
- The pinned version lives in `SUMO_VERSION` (pip form, e.g. `1.20.0`). The matching git
  tag form is `v${SUMO_VERSION//./_}` (e.g. `v1_20_0`). Vendoring `/sumo/` at that tag is
  a manual, network-side step done by a human outside the offline loop:
  ```
  git clone https://github.com/eclipse-sumo/sumo.git
  cd sumo && git checkout v1_20_0 && rm -rf .git   # match SUMO_VERSION
  ```
- Source and goldens **must** come from the same version. `provenance.txt` records which
  version produced each golden; a mismatch against `SUMO_VERSION` means goldens are stale.

## Determinism (phase 1)

Parity is *exact* in phase 1 because randomness is stripped: `sigma=0`, fixed depart,
`actionStepLength=1`, teleport off, Euler integration. These are set in each scenario's
config, not overridden by scripts. Statistical parity (with `sigma>0`) comes much later
and is declared per scenario in `tolerance.json` via its parity mode. Never introduce a
`System.Random`; use per-entity seeded RNG so results are independent of thread order.

## Build / test commands

- Build: `dotnet build`
- Test: `dotnet test`
- A fresh clone into a blank VM must pass `dotnet test` **without** SUMO installed. If it
  doesn't, that's a bug in the harness, not a missing dependency.

## Subagents

Use the definitions in `.claude/agents/`. Orchestration keeps the expensive model on
planning and final-gate parity review; routine porting and test-running go to cheaper
models. Because a subagent starts from near-zero context, every delegation must name: the
exact `/sumo/` source file to read, the target C# file(s), the scenario, the command to
run, and the numeric done-condition. Nothing crosses the boundary except the prompt.

## Reporting a parity failure

When a scenario is out of tolerance, report: scenario name, first-divergence step,
per-attribute max-abs error and RMSE, and the suspected cause (init/vType vs
algorithm vs integration/ordering). Prefer diffing `golden.state.xml` first to rule out a
vType-default init bug before chasing the trajectory.
