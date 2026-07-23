# Dense-flow engine integration — TRACKER

At-a-glance status for `docs/DENSE-ENGINE-INTEGRATION-TASKS.md`. Tick a box only when its success
conditions have been verified first-hand (Opus gate, per CLAUDE.md).

## Stage 1 — Core integration (delivers the #15 fix)
- [ ] **T1** — overlay the 5 engine files wholesale; solution builds clean
- [ ] **T2** — adopt `departPos="base"` test expectation; `Sim.ParityTests` = 654/4
- [ ] **T3** — determinism/bench: no `System.Random`; `deterministic=True`, `par==single=True` (hash: ____)
- [ ] **T4** — #15 gridlock A/B: arrivals ≥ 75 @160 / ≥ ~30 @70, end stoppedFrac ≤ 0.5, jam-and-recover
- [ ] **T5** — full `dotnet test` sweep green + parity-review accept
- [ ] **T6** — commit + push (fetch+rebase first)

## Stage 2 — Harden (OPTIONAL)
- [ ] **T7** — bring branch's dedicated parity tests + `_repro` scenarios (owner-gated)

## Notes / results log
- (record measured A/B numbers, parity result, and bench hash here as tasks complete)
