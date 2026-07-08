# Instructions: capture SUMO 1.20.0 minor-link approach debug trace (scenario 19 "on-ramp merge")

You are running in a **separate clone of the Eclipse SUMO repository** (the `eclipse-sumo/sumo`
`main` checkout). You have **no access** to the `pjanec/Traffic` repo — everything you need is
embedded verbatim in this document.

## Goal

The Traffic project is porting SUMO's minor-link "cautious approach" braking (the logic in
`MSVehicle::planMoveInternal` / `checkLinkLeader` / the `slowedDownForMinor` block) and needs the
**exact per-step internal values** SUMO computes for one vehicle (`rA`) approaching a minor
priority link. Static source reading was not enough — the per-step approach speed emerges from a
runtime interaction that can only be observed by turning on SUMO's built-in `DEBUG_PLAN_MOVE`
prints and running the scenario.

Your job: check out the pinned version, enable the debug prints gated to vehicle `rA`, build the
`sumo` binary in Debug, run the embedded scenario, and **commit the captured log** (plus your
edits and the scenario files) to a new branch so the Traffic session can read the numbers.

**Do not push to `main`. Do not open a PR.** Just create the branch, commit, and push the branch.

---

## Step 1 — Check out the pinned version, create a debug branch

The Traffic project is pinned to **SUMO 1.20.0**. The matching git tag is `v1_20_0`.
**Do not build from `main`** — the goldens and the port must come from the same version.

```bash
git fetch --tags
git checkout v1_20_0
git switch -c debug/c3-minor-link-trace
```

If `git switch -c` complains that you are in detached HEAD, that is expected after checking out a
tag — the `-c` still creates the branch from that commit. Verify:

```bash
git describe --tags   # should print v1_20_0 (or v1_20_0-0-g<sha>)
git branch --show-current   # should print debug/c3-minor-link-trace
```

---

## Step 2 — Enable the debug prints, gated to vehicle `rA`

Edit **`src/microsim/MSVehicle.cpp`**. Two changes:

### 2a. Uncomment `DEBUG_PLAN_MOVE` (near line 90)

Find this line (it is **commented out** by default):

```cpp
//#define DEBUG_PLAN_MOVE
```

Change it to (remove the `//`):

```cpp
#define DEBUG_PLAN_MOVE
```

### 2b. Gate the debug condition to `rA` (near line 106–108)

A few lines below, there is a block of `DEBUG_COND` definitions. By default the **active**
(uncommented) one is:

```cpp
#define DEBUG_COND (isSelected())
```

Change that active line to gate on the ramp vehicle's id:

```cpp
#define DEBUG_COND (getID() == "rA")
```

Leave the other (already-commented) `DEBUG_COND` variants as they are. The net effect: debug
prints fire **only** for vehicle `rA`, keeping the log small and focused.

> If the exact line numbers have drifted, search for the strings `DEBUG_PLAN_MOVE` and
> `#define DEBUG_COND` in `src/microsim/MSVehicle.cpp` — there is exactly one commented
> `//#define DEBUG_PLAN_MOVE` and exactly one **active** `#define DEBUG_COND (...)`; those are the
> two you edit.

---

## Step 3 — Install build dependencies and build the `sumo` binary (Debug)

On a Debian/Ubuntu host:

```bash
sudo apt-get update
sudo apt-get install -y \
  cmake g++ python3 python3-dev \
  libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig
```

Configure and build **only** the command-line `sumo` target in Debug mode. The GUI is not needed;
disabling optional libs keeps the build fast and robust:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCHECK_OPTIONAL_LIBS=OFF
cmake --build build -j"$(nproc)" --target sumo
```

The binary lands at **`build/bin/sumo`**. Confirm it is the Debug build with the prints compiled in:

```bash
./build/bin/sumo --version   # should report Version v1_20_0 / 1.20.0
```

> If `cmake` cannot find FOX and errors out despite `CHECK_OPTIONAL_LIBS=OFF`, add
> `-DFOX_CONFIG=` (empty) to the configure line to force it off. FOX is only needed for `sumo-gui`,
> not the `sumo` CLI target you are building.

---

## Step 4 — Recreate the scenario 19 input files

Create a working directory and write these **three files exactly** (they are copied verbatim from
the Traffic repo's `scenarios/19-onramp-merge/`). Do not reformat or "clean up" the XML — byte
content is what makes the trace reproducible.

```bash
mkdir -p scenario19 && cd scenario19
```

### `net.net.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>

<!-- generated on 2026-07-08 02:14:16 by Eclipse SUMO netconvert Version 1.20.0 -->

<net version="1.20" limitTurnSpeed="5.50" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/net_file.xsd">

    <location netOffset="0.00,30.00" convBoundary="0.00,0.00,1000.00,30.00" origBoundary="0.00,-30.00,1000.00,0.00" projParameter="!"/>

    <edge id=":J_0" function="internal">
        <lane id=":J_0_0" index="0" speed="13.89" length="14.14" shape="488.36,24.84 491.80,26.00 495.08,27.16 498.41,28.05 501.97,28.40"/>
    </edge>
    <edge id=":J_1" function="internal">
        <lane id=":J_1_0" index="0" speed="13.89" length="14.14" shape="487.83,28.40 501.97,28.40"/>
    </edge>

    <edge id="D" from="J" to="C" priority="10">
        <lane id="D_0" index="0" speed="13.89" length="498.03" shape="501.97,28.40 1000.00,28.40"/>
    </edge>
    <edge id="M" from="A" to="J" priority="10">
        <lane id="M_0" index="0" speed="13.89" length="487.83" shape="0.00,28.40 487.83,28.40"/>
    </edge>
    <edge id="R" from="B" to="J" priority="1">
        <lane id="R_0" index="0" speed="13.89" length="91.77" shape="400.46,-1.53 488.36,24.84"/>
    </edge>

    <junction id="A" type="dead_end" x="0.00" y="30.00" incLanes="" intLanes="" shape="0.00,30.00 0.00,26.80"/>
    <junction id="B" type="dead_end" x="400.00" y="0.00" incLanes="" intLanes="" shape="400.00,-0.00 400.92,-3.07"/>
    <junction id="C" type="dead_end" x="1000.00" y="30.00" incLanes="D_0" intLanes="" shape="1000.00,26.80 1000.00,30.00"/>
    <junction id="J" type="priority" x="500.00" y="30.00" incLanes="R_0 M_0" intLanes=":J_0_0 :J_1_0" shape="501.97,30.00 501.97,26.80 488.82,23.30 487.90,26.37 487.83,26.80 487.83,30.00">
        <request index="0" response="10" foes="10" cont="0"/>
        <request index="1" response="00" foes="01" cont="0"/>
    </junction>

    <connection from="M" to="D" fromLane="0" toLane="0" via=":J_1_0" dir="s" state="M"/>
    <connection from="R" to="D" fromLane="0" toLane="0" via=":J_0_0" dir="s" state="m"/>

    <connection from=":J_0" to="D" fromLane="0" toLane="0" dir="s" state="M"/>
    <connection from=":J_1" to="D" fromLane="0" toLane="0" dir="s" state="M"/>

</net>
```

### `rou.rou.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <vType id="car" vClass="passenger" sigma="0"/>
    <route id="main" edges="M D"/>
    <route id="ramp" edges="R D"/>
    <vehicle id="mA" type="car" route="main" depart="0"  departPos="0" departSpeed="13.89" departLane="0"/>
    <vehicle id="rA" type="car" route="ramp" depart="2"  departPos="0" departSpeed="13.89" departLane="0"/>
</routes>
```

### `config.sumocfg`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="net.net.xml"/>
        <route-files value="rou.rou.xml"/>
    </input>
    <time><begin value="0"/><end value="90"/><step-length value="1"/></time>
    <processing>
        <step-method.ballistic value="false"/>
        <time-to-teleport value="-1"/>
        <default.action-step-length value="1"/>
        <default.speeddev value="0"/>
        <collision.action value="none"/>
        <lanechange.duration value="0"/>
    </processing>
    <random_number><seed value="42"/></random_number>
</configuration>
```

---

## Step 5 — Run the scenario and capture the debug log

From inside `scenario19/`, run the Debug binary and tee **both** stdout and stderr to a log. Also
emit the FCD trajectory so the Traffic session can line up the debug values against per-step
positions/speeds:

```bash
../build/bin/sumo -c config.sumocfg \
  --fcd-output fcd.xml --precision 6 \
  2>&1 | tee c3-debug.log
```

`--precision 6` matches how the Traffic goldens are generated. The `2>&1` is important — the
`DEBUG_PLAN_MOVE` prints go to stdout, but keep stderr merged in case any diagnostic lands there.

### What the log must contain (verify before committing)

The log should contain per-timestep blocks for vehicle `rA` once it departs (t≥2). The lines the
Traffic session needs are the minor-link approach prints — grep to confirm they are present:

```bash
grep -nE "rA|slowedDownForMinor|couldBrakeForMinor|link=:J_0_0|visibilityDistance|brakeDist|arrivalSpeed|maxSpeedAtVisibilityDist" c3-debug.log | head -50
```

Specifically, the key values wanted per step (as `rA` closes on the minor link `:J_0_0`) are:

- the `planMoveInternal` / `approaching link=:J_0_0` line with **`seen=` (distance to link)**,
  **`visibilityDistance=`**, and **`brakeDist=`**;
- the **`slowedDownForMinor`** block with **`maxSpeedAtVisibilityDist=`**, **`maxArrivalSpeed=`**,
  and **`arrivalSpeed=`**;
- the resulting **`vSafe` / `vLinkPass` / `vLinkWait`** for that link on each step.

If those strings are **absent**, the debug gate did not take — recheck Step 2 (both the
`#define DEBUG_PLAN_MOVE` uncomment **and** `DEBUG_COND (getID() == "rA")`), rebuild, and rerun.
Do not commit an empty/silent log.

---

## Step 6 — Commit and push the branch

Commit your source edits, the scenario files, and the captured outputs to the debug branch:

```bash
cd ..   # back to repo root
git add src/microsim/MSVehicle.cpp scenario19/
git commit -m "debug: enable DEBUG_PLAN_MOVE for rA, capture scenario19 minor-link trace"
git push -u origin debug/c3-minor-link-trace
```

> `build/` is git-ignored by SUMO's `.gitignore`; do **not** try to commit the build tree or the
> binary. Only the source edit, the `scenario19/` inputs, `c3-debug.log`, and `fcd.xml` need to
> travel.

---

## Deliverable summary

When you are done, report back:

1. the branch name (`debug/c3-minor-link-trace`) and the commit SHA;
2. confirmation that `git describe --tags` showed **v1_20_0**;
3. the first ~5 `slowedDownForMinor` / `approaching link=:J_0_0` lines for `rA` pasted inline
   (a quick sanity preview so the Traffic session knows the numbers are real before pulling);
4. the paths committed: `scenario19/c3-debug.log` and `scenario19/fcd.xml`.

That log is the whole point — it unblocks the C3 on-ramp-merge parity port back in the Traffic repo.
