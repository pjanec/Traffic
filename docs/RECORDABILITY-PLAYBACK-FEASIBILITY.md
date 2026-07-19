# RECORDABILITY-PLAYBACK-FEASIBILITY.md — recording a run for later playback

**Status: feasibility study (no commitment to build).** Evaluates how to record a simulation run for later
playback, and specifically tests the hypothesis **"recording the DDS stream is all we might need."** Like
`DISTRIBUTED-COUPLING-FEASIBILITY.md`, this is a return-to-later artifact + a set of guardrails so current
work keeps recordability cheap.

**Bottom line up front:** the hypothesis is **right for the common case** — if the goal is *watch/scrub/
review/demo a past run*, recording the replication (DDS) stream is nearly free and sufficient, because the
wire already carries everything a viewer needs. It is **not** sufficient for *exact reproduction, internal
analysis, or what-if branching* — that needs recording the **inputs** and re-simulating. The two are cheap
and complementary; pick per need.

---

## 1. Two levels of recordability (they answer different questions)

| | **Wire recording** (record the replication stream) | **Input recording** (record inputs, re-simulate) |
|---|---|---|
| What you store | The broadcast poses/TL/lifecycle/geometry (FrameCodec bytes) | scenario (net+rou+cfg) + version + seed + live-command log |
| Playback = | Feed recorded frames back into the viewer/IG path | Re-run the deterministic engine |
| Fidelity | The **IG's reconstructed view** (quantized, DR-gated, render-projection only) | **Bit-identical full state**, incl. internal engine state the wire never carries |
| Size | Moderate (compressed by LOD + DR gating) | Tiny (a few files + a command log) |
| Playback cost | Cheap (no engine; just replay + reconstruct) | Re-runs the engine (CPU) |
| Enables | watch, scrub, demo, "what did the operator see" | exact repro, analyse internal decisions, branch/what-if, debug parity |
| New code | a file binding of the existing sink/source seam | full input capture + a deterministic replay harness |

The user's hypothesis is level 1. It is the *right default* for playback; level 2 is the *reproducibility/
analysis* tool. They compose (see §4).

## 2. Why wire recording is nearly free (the hypothesis is well-founded)

The replication layer was built exactly right for this:

- **The wire already carries a complete viewable run.** Geometry (once, durable), per-step vehicle + ped
  crowd frames, TL state, and lifecycle (spawn/despawn/DR-model switch), plus ped path-arcs and activity
  timelines. That *is* everything a viewer draws — nothing extra needs recording to reproduce the view.
- **The serialized form already exists and is canonical.** `FrameCodec` produces the one packed byte layout
  used across DDS/TCP/UDP; `InMemoryPedReplicationBus` already frames **every publish as `(Topic, byte[])`**.
  A recorder is therefore "**append `[sim-time][wall-time][topic][length][FrameCodec bytes]` to a file**" —
  **no new codec**, just framing the bytes that already cross the wire.
- **It's a third binding of an existing seam.** `IReplicationSink`/`IReplicationSource` (and the ped pair)
  are transport-neutral with three intended bindings: InMemory, DDS, and — naturally — **file**. A
  `RecordingSink` tees every publish to disk (and forwards to the real sink); a `ReplaySource` reads the
  file and re-emits into the *unchanged* viewer/reconstructor. The IG code does not change.
- **It's already compressed by design.** DR-error gating makes predictable movers go silent (zero per-step
  bytes), and the LOD split records ~90k ambient peds as **one PathArc each** rather than per-step samples.
  The same math that keeps the live stream at ~tens of Mbit/s keeps a recording's size manageable; ped
  positions are int32-cm quantized on the wire, so the recording inherits that compactness.
- **Durable topics are the built-in keyframes.** Geometry, lifecycle, path-arcs, and activity timelines are
  keyed/durable — replaying every durable sample with time ≤ *t*, plus the latest volatile frame at *t*,
  reconstructs the full scene at an arbitrary seek point. Scrubbing does not need separately-authored
  keyframes; the durable topics *are* the state snapshot.

## 3. What wire recording cannot do (the honest limits)

- **It is the reconstructed view, not the authoritative trajectory.** Playback feeds recorded frames through
  the *same* reconstruction (DrClock / PedRemoteReconstructor extrapolation) the live IG uses — so you
  replay *what a viewer saw* (server == IG within render tolerance), not the server's exact ground-truth
  poses. Great for watching; not a source of truth for analysis.
- **It's lossy and render-scoped.** cm-quantized positions, DR-gated updates, and only *render* projections
  — no car-following gaps, lane-change intent, ORCA neighbour sets, junction-request state. You can watch
  the run; you cannot inspect *why* an agent did what it did.
- **Best-effort gaps if recorded off the network.** The high-rate topics are BEST_EFFORT; a recorder reading
  off DDS could miss dropped samples. **Fix: record at the *sink* (co-located with the publisher, tapping
  before DDS)** — lossless, and it also avoids needing a live DDS subscriber at all.
- **No branching / what-if.** You cannot change an input and re-run from a wire recording.

None of these matter for "watch it later"; all of them matter for "reproduce/analyse/branch."

## 4. The complement: input recording for exact reproduction

The engine is **deterministic in phase 1** (`CLAUDE.md`: `sigma=0`, fixed depart, `actionStepLength=1`,
Euler, per-entity **seeded** RNG, *no* `System.Random`), so a run is a pure function of its inputs. Recording
the inputs — the committed scenario files (`net.xml` / `rou.xml` / `sumocfg`), the `SUMO_VERSION`, the seed,
and a **log of any live commands/injections** (the `DdsViewerCommand` reverse channel: pause/speed/restart/
drop-obstacle) — lets you **re-run and reproduce the entire run bit-identically**, including everything the
wire drops. It is tiny to store and is the natural artifact for regression debugging, parity investigation,
and what-if branching.

Caveats: it must capture *every* nondeterminism source (today just seed + live commands, since phase-1 is
deterministic); a future `sigma>0` statistical mode or a distributed/real-time run (see
`DISTRIBUTED-COUPLING-FEASIBILITY.md`) would only reproduce exactly if seed *and* ordering are captured, which
distribution cannot guarantee — so for those regimes, wire recording is the *only* faithful playback.

**Hybrid (the sweet spot):** keep the **input log** as the canonical, tiny, exact "project file", and
optionally the **wire recording** as the instant-scrub "video" that needs no re-simulation. For most
operational playback, the wire recording alone is enough; add input recording when exact repro/analysis is
required.

## 5. Design sketch (if built)

**Format:** a flat append-only file of framed records: `[u64 sim-time-µs][u64 wall-time-µs][u8 topic][u32
len][FrameCodec/Geometry/Tl bytes]`. No new codec — the payload is the existing wire bytes. One file per run
(or split per hour). Optionally gzip; the DR-gated stream compresses well.

**Record:** `RecordingReplicationSink : IReplicationSink` (and the ped pair) that writes each publish to the
file and forwards to the inner sink — a decorator, dropped in at the publisher. Lossless (taps before DDS).

**Replay:** `ReplayReplicationSource : IReplicationSource` (and ped pair) that reads the file and exposes the
same `Pump()`-then-read surface, pacing by **sim-time** (so playback speed is controllable: 1×, fast, step).
The viewer/reconstructor is **unchanged** — it cannot tell a file source from a DDS source.

**Seek/scrub:** to jump to *t*, replay all **durable** samples with time ≤ *t* (geometry + latest lifecycle/
path-arc/activity per key) then the newest volatile frame — the durable topics are the keyframe state.

**Out of `Traffic.sln`?** The file binding itself is pure managed (FrameCodec bytes + file I/O) and could
live in `Sim.Replication` *in* the gate (it needs no DDS/native dep) — unlike the DDS binding. That's a plus:
recording/replay can be hermetically unit-tested (record a synthetic run, replay it, assert frame equality)
in the offline gate, with **no** SUMO or DDS.

## 6. Recordability-preserving guardrails (do these now — they're already true)

Cheap invariants so current work keeps playback free:

1. **Everything a viewer needs stays on the wire** (geometry, lifecycle, TL, poses, path-arcs, timelines).
   Already true — it's the replication contract. Don't add a render input that bypasses the wire.
2. **The canonical `FrameCodec` bytes remain the single serialized form.** A recording *is* wire bytes;
   never introduce a viewer-only side channel that isn't in the codec, or recordings become incomplete.
3. **Keep durable topics keyed** (geometry by chunk, lifecycle/leg by handle). This is what makes seek/late-
   join/scrub work — already the design.
4. **Carry sim-time (and wall-time where available) on every frame.** Already true (`FrameCodec` header
   `time`; DDS `SourceTimestamp`). Playback pacing and seek depend on it.
5. **Prefer recording at the sink (pre-DDS), not off the network** — lossless capture of the exact broadcast.
6. **Keep the run reproducible from inputs** if exact re-sim is ever wanted: preserve phase-1 determinism (no
   `System.Random`, seeded per-entity RNG), and if a live-command reverse channel is used, make its commands
   **loggable** (timestamped, replayable) — so an input recording can reproduce an interactive run.

As with the distribution study, these coincide with disciplines the project already keeps (transport-neutral
seams, canonical codec, determinism), so "keep runs recordable" is **free**.

## 7. Effort sketch & recommendation

- **Wire recorder/replayer:** small. A decorator sink + a file source + a framing format, all reusing
  `FrameCodec`; hermetically testable in the gate. This is the high-value, low-cost piece and directly
  realises "recording DDS is all we need" for observation/playback.
- **Input recorder + deterministic replay harness:** small-to-medium (input capture is trivial; the
  command-log + replay driver is the work). Add when exact repro / analysis / branching is required.

**Recommendation:** the hypothesis holds — for *playback/observation*, a **wire (replication-stream)
recorder** is sufficient and cheap, and it's a clean third binding of the existing sink/source seam (record
at the sink for lossless capture; durable topics give free scrub). Treat **input recording** as the separate,
also-cheap tool for exact reproduction/analysis/what-if, and keep the §6 guardrails so both stay available.
Build the wire recorder first if/when a "review past runs" need appears; it needs no DDS and no SUMO and can
be gate-tested.
