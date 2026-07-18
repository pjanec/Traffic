# Issue 2 is a teleport MISCLASSIFICATION, not a deadlock — diagnosis + shareable repro

Re-diagnosed the real-box "Issue 2" (SumoSharp 105 jam-teleports vs vanilla 1). The framing changes
completely, and in your favour: **this is not a car-following / demand / gridlock problem. It is a
teleport-classification / yield-wait-handling divergence.**

## The evidence

1. **Congestion is equal — vanilla is if anything *more* congested.** On the real box, sustained halts
   (speed<0.1 for ≥120 s) are **vanilla 256 vs SumoSharp 205**. So SumoSharp vehicles are *not* more
   stuck. Yet vanilla fires 1 jam-teleport and SumoSharp fires 105.

2. **The smoking gun — same scenario, same flags, opposite teleport category** (junction-realistic
   synthetic repro, `--time-to-teleport 120`):

   | | vanilla | SumoSharp |
   |---|---|---|
   | `<teleports jam=…>` | **0** | **75** |
   | `<teleports yield=…>` | **3** | **0** |
   | total | 3 | 75 |

   Vanilla classifies long unsignalized-junction waits as **yield** waits (3, and it *tolerates* the
   rest without teleporting). SumoSharp classifies the very same waits as **jam** and teleports them at
   the 120 s jam threshold. The `--jtype right_before_left` variant reproduces identically (vanilla 0
   jam, SumoSharp 65 jam).

## What that means for the fix

The vehicles SumoSharp jam-teleports are cars **legitimately waiting for right-of-way** at an
unsignalized junction (a foe stream has priority), *not* cars blocked by a jammed leader on their own
lane. Vanilla SUMO distinguishes these two:
- **jam** teleport = blocked by a stopped *leader on your own lane* past `time-to-teleport`.
- **yield** teleport = waiting to enter a junction for a *foe with right-of-way*; SUMO handles this
  separately and far more leniently (it is not the plain `time-to-teleport` jam timer — a car waiting
  for a genuinely busy foe stream is not a jam and mostly should keep waiting).

**Hypothesis for where SumoSharp diverges** (yours to confirm in the engine): the teleport bookkeeping
treats *any* vehicle stopped for > `time-to-teleport` as a **jam** candidate, without the
"am-I-waiting-for-a-junction-foe?" branch — so junction yield-waits get miscounted as jams and
teleported at 120 s. Vanilla's equivalent is in its junction/right-of-way wait accounting (waiting-time
for a foe is tracked distinctly from lane-jam waiting, and the yield case has its own, much weaker,
teleport trigger). Fix = reproduce vanilla's classification: a vehicle whose blocker is a
higher-priority foe at a junction (not a stopped leader on its lane) is a **yield** wait, counted as
yield, and subject to vanilla's yield-teleport rule rather than the 120 s jam cutoff.

This is much smaller than a deadlock rewrite — it's teleport-cause attribution + the yield-wait timer.

## Repro (shareable — pure synthetic, no real geometry)

`synthetic_junction_bundle.tgz` (and committed at SumoData `experiments/subarea/synthetic_junction/`).
An irregular `netgenerate --rand` net that embeds the real-box wedge profile: **83% unsignalized nodes**
(priority / right_before_left), **65% with an approach edge < 30 m** (spillback blocks the upstream
node), **39% with a 2→1 lane drop**, single-lane minor approaches, `--random-priority` major/minor
yields. Deterministic (`build.py`, seed 42). The pre-existing uniform 8×8 grid does NOT show this
(0 vs 0) — it lacks the junction micro-geometry — so **add this as a second golden alongside
`synthetic-parity`; it is the one that gates Issue 2.**

Two commands (from the scenario dir), identical flags, only the binary differs:
```
<bin> -c scenario.sumocfg --fcd-output F.xml --summary-output S.xml \
      --tripinfo-output TI.xml --statistic-output ST.xml --end 1000 --no-step-log true
```
Done when SumoSharp's `<teleports jam=…>` drops to vanilla-ish (0–3) with `yield` accounting matching
vanilla, mean rel-speed converges, and the no-cheating audit still PASSES (it does today: PASS).

## Status

- **Issue 1 (residency): accepted — green** on real box + both synthetics.
- **Issue 2: not green**, but now precisely characterised as a yield-vs-jam teleport-classification bug
  with a shareable golden that reproduces it. Bundle ships both engines' summary + statistic + tripinfo
  (FCDs omitted for size; `build.py` + README regenerate them).
