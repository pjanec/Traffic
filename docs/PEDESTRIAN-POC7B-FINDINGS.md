# POC-7b findings — quantized crowd transport + PathArc wire format + single-stream bandwidth

Status: **done**. This is the networking half of POC-7 (`docs/PEDESTRIAN-POC-PLAN.md` POC-7;
`docs/PEDESTRIAN-DESIGN.md` §7). It adds a compact quantized pedestrian `FreeKinematic` record and a
`PathArc` wire record to `Sim.Replication.FrameCodec`, and measures the **real encoded bytes** for the
target population against the owner's single-multicast-stream budget.

> **On-target confirmation (P6-1):** re-run on the 24-core Core Ultra 9 275HX target box, `Sim.BenchPedNet`
> is **byte-identical across 3 runs and identical to the figures below** — 36.75 / 182.4 / 294.4 Mbit/s
> (typical / worst-case spike / naive), all under the 500 Mbit/s budget. This is byte-counting through the
> real `FrameCodec`, so it is machine-independent by construction; the on-target run confirms it on the
> target toolchain. The *live* CycloneDDS transport binding does not exist yet (dev-session work); this is
> the single-stream codec figure, which is the one that matters. See `docs/PEDESTRIAN-P6-1-RESULTS.md`. `Sim.Replication`/`Sim.Core` are
parity-exempt per `CLAUDE.md`; this task touched neither the lane engine, `Sim.Ingest`, `OrcaCrowd`, nor
`MixedTrafficCrowd`.

**The owner's corrected framing** (carried over from `PEDESTRIAN-DESIGN.md` §7): DDS multicast is **one**
stream every IG channel subscribes to — there is no per-channel culling to build. Bandwidth is therefore a
single global figure, and the budget is **~500 Mbit/s** (50% of a 1 Gbit link, keeping the other half free
for spikes).

## What was built

### 1. `PedFreeKinematicRecord` — the quantized high-power ped record

New record kind (`FrameCodec.KindPedFreeKinematic = 3`), additive alongside the existing 32 B
`CrowdRecord` (unchanged, byte-for-byte — other consumers may already depend on its layout). Logical type
`Sim.Replication.PedFreeKinematicRecord` (handle, X, Y, Vx, Vy, Radius — all `double`, no `Z`); the codec
(`WritePedFreeKinematicFrame`/`ReadPedFreeKinematicFrame`) does the cm-precision packing.

**Wire layout — 18 B/record:**

| Field | Type | Bytes | Precision / range |
|---|---|---|---|
| handle index | `u32` | 4 | — (generation dropped, see below) |
| x | `i32`, cm | 4 | 1 cm; ±21,474,836 m |
| y | `i32`, cm | 4 | 1 cm; ±21,474,836 m |
| vx | `i16`, cm/s | 2 | 1 cm/s; ±327.67 m/s |
| vy | `i16`, cm/s | 2 | 1 cm/s; ±327.67 m/s |
| radius | `u16`, cm | 2 | 1 cm; 0–655.35 m |
| **total** | | **18** | |

**Position tradeoff (chosen: int32 cm absolute).** The task offered two options:

- **int32 cm absolute (chosen).** 8 B for `(x, y)`, range ±21,474,836 m — vastly more than any realistic
  net extent, and needs **no per-frame/chunk origin bookkeeping**. The failure mode of the alternative
  (a ped straying outside a maintained origin's window) simply cannot happen. Landed at **18 B total**
  instead of the task's "~14–16 B" target.
- **int16 cm relative to a per-frame/chunk origin (not chosen).** Would shave the record to ~14 B (4 B for
  `(x, y)` instead of 8), but requires a maintained origin per frame or per spatial chunk, and silently
  clamps/wraps once a ped strays more than ±327.67 m from that origin — a real risk for a population
  spread across a city-scale net without also building spatial chunking. Left as a documented follow-up;
  not needed at the measured scale (see the bandwidth table below — even the un-shaved 18 B record clears
  the budget with wide margin).

`Radius` is carried per-record (not amortized via a one-time lifecycle message the way vehicle
length/width are) because the task spec calls for it in this record; at 2 B/record the cost is
negligible either way. `Handle` carries only the `u32` index — the `u16` generation is **not** on this
hot-path wire record. This is a deliberate size/robustness tradeoff: the generation is authoritative on
the separate lifecycle topic (keyed by the full handle), so a decoder that needs to detect a stale/reused
index resolves it there, exactly the same "physical/authoritative params travel once, elsewhere" pattern
`SUMOSHARP-DEADRECKONING.md` §4.2 already uses for vehicle length/width.

No `Z`: pedestrians are assumed to ride the walkable ground plane; elevation is resolved on the IG from
the static walkable-surface geometry, exactly as `VehicleRecord` already carries no `Z` (resolved from
lane geometry). Not a claim that multi-level peds are unsupported forever — a scope cut for this POC.

### 2. `PathArcRecord` — the low-power ped's one-time payload

New record kind (`FrameCodec.KindPathArc = 4`). Logical type `Sim.Replication.PathArcRecord` (handle,
`Speed`, `StartTime`, `IReadOnlyList<Vec2> Path`) — the wire counterpart of
`Sim.Pedestrians.Lod.PathArcRecord` (POC-3's event model). `Sim.Replication` does not take a
`Sim.Pedestrians` dependency; it reuses `Sim.Core.Orca.Vec2` directly, same as `Sim.Pedestrians` does.

**Wire layout — 14 B + 8 B/waypoint (variable length):**

| Field | Type | Bytes |
|---|---|---|
| handle index | `u32` | 4 |
| speed | `f32` | 4 |
| startTime | `f32` | 4 |
| point count | `u16` | 2 |
| per point: x_cm, y_cm | `i32`, `i32` | 8 × N |

Sent **once** per ped (on spawn, and again on each demotion with a fresh re-routed path) on the low-rate/
durable lifecycle topic — never repeated per step — so it is not counted as per-step bandwidth, only
amortized over the path's lifetime (plus the separate heartbeat). This is what the whole "ambient 90k for
almost free" claim in `PEDESTRIAN-DESIGN.md` §7 rests on.

### 3. Round-trip tests

`tests/Sim.ParityTests/RungB22ReplicationCodecTests.cs` (additive, alongside the existing `VehicleFrame`/
`CrowdFrame` round-trip tests, which are unchanged):

- `PedFreeKinematicFrame_RoundTrips_WithinCentimeter` — encodes two records (including a large-magnitude
  case, x = −20,000.005 m) and asserts `X`/`Y`/`Vx`/`Vy`/`Radius` all round-trip within **1 cm** (the
  quantization tolerance the task asked for), and that the decoded handle's generation is 0 (documenting
  the dropped-generation tradeoff rather than silently passing).
- `PathArcFrame_RoundTrips` — encodes a 4-point path and a degenerate 1-point path, asserts the variable
  frame-size arithmetic (`PathArcFrameSize`/`PathArcRecordSize`) matches the actual bytes written, and
  that handle, speed, startTime (float32 precision, asserted to 3 decimals) and every waypoint (cm
  precision, asserted to 2 decimals) round-trip.

**Result: both pass**, cm-tolerance confirmed by assertion, not eyeballed.

## The bandwidth benchmark — `src/Sim.BenchPedNet`

A new console project (`dotnet run -c Release --project src/Sim.BenchPedNet`), deliberately **not** part
of `dotnet test` (same convention as `Sim.Bench`/`Sim.BenchCity`/`Sim.BenchCrowd`/`Sim.BenchCrowd`'s POC-7a
sibling — wall-clock/modeling numbers must never gate the hermetic offline loop). It builds the target
population **deterministically** (`Sim.Core.VehicleRng`, seeded `SplitMix64` — no `System.Random`,
per `CLAUDE.md`), **encodes it through the real `FrameCodec`** (`WriteVehicleFrame`, `WriteCrowdFrame`,
`WritePedFreeKinematicFrame`, `WritePathArcFrame`), and reports the actual returned byte counts — not a
`recordSize * count` hand calculation. The heartbeat is modeled as a `PathArc` record with **zero**
waypoints (14 B: handle + speed + startTime + a zero point-count) — a real, codec-encoded value, and if
anything a *conservative* (slightly pessimistic) stand-in versus a bespoke smaller liveness message.

Population: **10,000 cars**, **10,000 high-power peds**, **90,000 low-power peds** (100,000 peds total) —
the POC-7 target. Rates: cars and high-power peds at 10 Hz; low-power heartbeat every 3 s; a representative
16-waypoint path amortized over a 60 s lifetime. DR-gated "typical" sent fraction: 60% (a model of
`DrErrorPublishPolicy` going quiet for predictable movers).

### Results (measured, one run of the tool; reproducible — deterministic seeding)

**Scenario (a): typical — LOD split, DR-gated**

| Class | Sent | Bytes/s | Mbit/s |
|---|---:|---:|---:|
| Cars (existing 48 B `VehicleRecord`, 60% sent) | 6,000 / 10,000 | 2,880,160 | 23.04 |
| High-power peds (18 B quantized, 60% sent) | 6,000 / 10,000 | 1,080,160 | 8.64 |
| Low-power peds (`PathArc` amortized + heartbeat) | 90,000 | 633,000 | 5.06 |
| **TOTAL (single multicast stream)** | | **4,593,320** | **36.75** |

→ **Fits 500 Mbit/s with ~92.7% headroom.**

(Low-power detail: path record 142 B for a 16-point leg, amortized over 60 s = 2.37 B/s/ped; heartbeat
record 14 B every 3 s = 4.67 B/s/ped; × 90,000 peds = 633,000 B/s combined.)

**Scenario (b): worst-case spike — ALL 100k peds promoted to `FreeKinematic`, everything at 100%**

| Class | Sent | Bytes/s | Mbit/s |
|---|---:|---:|---:|
| Cars, 100% sent | 10,000 | 4,800,160 | 38.40 |
| ALL peds promoted, quantized `FreeKinematic` (18 B), 100% sent | 100,000 | 18,000,160 | 144.00 |
| **TOTAL (single multicast stream)** | | **22,800,320** | **182.40** |

→ **Fits 500 Mbit/s with ~63.5% headroom**, even in the pathological case of the entire ambient
population waking simultaneously (an incident-scale event) and every record being sent every tick.

**Scenario (c): naive baseline — unquantized `FreeKinematic` for all peds (no quantization, no LOD)**

| Class | Sent | Bytes/s | Mbit/s |
|---|---:|---:|---:|
| Cars, 100% sent (unchanged — `VehicleRecord` was already unquantized) | 10,000 | 4,800,160 | 38.40 |
| ALL peds, unquantized `CrowdRecord` (32 B), 100% sent | 100,000 | 32,000,160 | 256.00 |
| **TOTAL (single multicast stream)** | | **36,800,320** | **294.40** |

→ **Still fits 500 Mbit/s, but only ~41.1% headroom** — this is the design doc's "honest finding"
confirmed empirically: even the pessimistic all-high-power-unquantized case is not a hard wall. (This
number lines up almost exactly with `PEDESTRIAN-DESIGN.md` §7's own estimate of "~256 Mbit/s" for
100k unquantized peds at 10 Hz — the estimate and the measurement agree.)

### Headroom summary

| Scenario | Total Mbit/s | Fits 500 Mbit/s? | Headroom |
|---|---:|:---:|---:|
| (a) Typical (LOD split, DR-gated) | 36.75 | Yes | 92.7% |
| (b) Worst-case spike (all promoted, 100%) | 182.40 | Yes | 63.5% |
| (c) Naive baseline (no quantization/LOD) | 294.40 | Yes | 41.1% |

**What quantization + LOD actually buys, quantified:** scenario (b) vs (c) isolates the ped class alone:
144.00 Mbit/s (quantized, all promoted) vs 256.00 Mbit/s (unquantized) — a **43.75% cut** from
quantization alone, before the LOD split even engages. Scenario (a) vs (b) isolates what the LOD split
buys on top: 36.75 Mbit/s (typical, mostly low-power) vs 182.40 Mbit/s (all promoted) — a further **~5×**
reduction. Consistent with `PEDESTRIAN-DESIGN.md` §7's claim that the LOD split's primary payoff is CPU +
spike headroom, not bare survival — bandwidth was never the hard wall, even without it.

### DDS framing overhead (not modeled precisely, per the task's own instruction)

The numbers above are `FrameCodec` **payload** bytes; DDS per-sample/instance framing is extra. The blob
topic chunks at ~64 KiB/sample (`FrameChunker.MaxRecordsForPayload`); the tool reports the resulting
sample counts per tick:

| Population @ 100% | Record size | Samples/tick (≤64 KiB each) |
|---|---:|---:|
| 10,000 cars | 48 B | 8 |
| 100,000 peds, quantized (18 B) | 18 B | 28 |
| 100,000 peds, unquantized (32 B) | 32 B | 49 |

Single-digit-to-tens of samples/tick is a small, bounded overhead multiplier (headers, sequence numbers,
instance bookkeeping per sample) — even a generous 15–20% allowance on top of scenario (c)'s 294.40 Mbit/s
(≈ 339–353 Mbit/s) still clears the 500 Mbit/s budget. Not modeled precisely, per the task's own framing.

## Gates

```
dotnet build
  → Build succeeded, 0 errors (35 pre-existing warnings, unrelated to this change)

dotnet test
  → Sim.ParityTests:                       Passed! Failed: 0, Passed: 470, Skipped: 3, Total: 473
                                            (468 pre-existing + 2 new: PedFreeKinematicFrame_RoundTrips_
                                             WithinCentimeter, PathArcFrame_RoundTrips; the 3 skips are
                                             pre-existing and unrelated)
    Sim.Pedestrians.Tests:                 Passed! Failed: 0, Passed: 61, Skipped: 0, Total: 61
    Sim.Host.Tests:                        Passed! Failed: 0, Passed: 1, Skipped: 0, Total: 1
    Sim.Pedestrians.Nav.DotRecast.Tests:   Passed! Failed: 0, Passed: 2, Skipped: 0, Total: 2
```

All green, all pre-existing suites unaffected. `CrowdRecord`/`VehicleRecord` layouts are byte-for-byte
unchanged (`RungB22ReplicationCodecTests.VehicleFrame_RoundTrips`/`CrowdFrame_RoundTrips` — both
pre-existing, both still pass unmodified). `src/Sim.Core/Orca`, `Sim.Ingest`, `OrcaCrowd`, and
`MixedTrafficCrowd` were not touched.

## Files touched

- `src/Sim.Replication/Records.cs` — additive: `PedFreeKinematicRecord`, `PathArcRecord`.
- `src/Sim.Replication/FrameCodec.cs` — additive: `KindPedFreeKinematic`, `KindPathArc`,
  `PedFreeKinematicRecordSize`, `PedFreeKinematicFrameSize`, `WritePedFreeKinematicFrame`,
  `ReadPedFreeKinematicFrame`, `PathArcRecordSize`, `PathArcFrameSize`, `WritePathArcFrame`,
  `ReadPathArcFrame`, and the private cm-quantization helpers. Existing `KindVehicle`/`KindCrowd` paths
  and sizes unchanged.
- `tests/Sim.ParityTests/RungB22ReplicationCodecTests.cs` — additive: the two round-trip tests above.
- `src/Sim.BenchPedNet/` (new project) — the bandwidth benchmark tool.
- `Traffic.sln` — `Sim.BenchPedNet` added.
- `docs/PEDESTRIAN-POC7B-FINDINGS.md` — this file.
