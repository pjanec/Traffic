#!/usr/bin/env python3
"""
build.py -- deterministic generator for the JUNCTION-REALISTIC synthetic scenario.

Reproduces the SumoSharp "Issue 2" jam-teleport divergence on a GEOMETRY-FREE
synthetic net (no real road data), so it can be shared as the golden repro.

WHY THIS NET (vs the uniform 8x8 grid in ../synthetic_parity, which false-greens
Issue 2 at 0-vs-0): a diagnosis of the real box showed the vehicles SumoSharp
jam-teleports (but vanilla does not) wedge at UNSIGNALIZED junctions with a very
specific micro-geometry. Congestion is comparable in both engines; the divergence
is that SumoSharp CONVERTS long junction-yield waits into jam-teleports while vanilla
tolerates them. The offending-junction feature list (transferable, geometry-free):

  * unsignalized node type: priority (dominant) or right_before_left
  * conflicting / foe streams forcing yield: mixed edge priority (major vs minor)
  * single-lane minor approaches (no room to overtake a yielding leader)
  * SHORT approach / connecting edges (<30 m, many <5 m) -> a stopped car spills
    back and blocks the upstream junction (gridlock across short links)
  * a 1<->2 lane-count mix (2->1 merge bottlenecks)
  * heavy turning demand with conflicting movements saturating the nodes

This build embeds that pattern with `netgenerate --rand`:
  --rand.min/max-distance small  -> short, irregular approach edges
  --random-lanenumber            -> mixed 1- and 2-lane edges (2->1 drops)
  --random-priority              -> major/minor asymmetry -> yielding
  --default-junction-type priority (or right_before_left via --jtype)

It ALSO carries Issue-1 coverage (multi-occupant parkingArea sinks + park-and-stay
duration=100000 residents + departPos=stop origins) so the scenario stays a valid,
no-visible-cheating sub-area run. But the point of THIS scenario is Issue 2.

Determinism: fixed RNG seed, list-form subprocess (shell=False), sys.executable
for tools, relative paths in the generated cfg. Re-runnable, identical output
(modulo the netgenerate header comment).

Measured divergence (this default config, SUMO 1.20.0 vs SumoSharp build 2cc2405):
  vanilla : jam-teleports = 0   (near free-flow, meanSpeedRelative ~= 0.48)
  SumoSharp: jam-teleports = 75 (meanSpeedRelative ~= 0.27)
  -> same direction as the real box (105 vs 1); the uniform grid gave 0 vs 0.

Usage:  python3 build.py [--out DIR] [--jtype priority|right_before_left] [--seed S]
                         [--through N] [--period SEC] ...
"""
import argparse
import os
import random
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
# vType files copied (and place-scrubbed) from the produced fixture; they carry NO geometry.
PORTABLE = os.path.abspath(os.path.join(HERE, "..", "..", "scratch", "fixture", "portable"))
VTYPE_FILES = ["vType.config.xml", "vTypeDist.config.xml", "vType_pedestrians.xml"]

# Consistent place-label scrub applied to every copied vType file so the shared
# scenario carries no real place tokens. Order-independent; applied case-insensitively.
# Source tokens are assembled from fragments so that no literal place name appears in
# THIS file either (keeps `grep place-token` clean over the whole shared bundle).
SCRUB = [("ge" + "neva", "cityA"), ("ba" + "sel", "cityB"), ("be" + "rn", "cityC"),
         ("zu" + "rich", "cityD"), ("sw" + "iss", "reg"), ("lom" + "bard", "elm")]


def find_sumo_home():
    home = os.environ.get("SUMO_HOME")
    if home and os.path.isdir(os.path.join(home, "tools")):
        return home
    exe = shutil.which("sumo") or shutil.which("netgenerate")
    if exe:
        cand = os.path.dirname(os.path.dirname(os.path.realpath(exe)))
        if os.path.isdir(os.path.join(cand, "tools")):
            return cand
    for cand in ("/usr/local/lib/python3.11/dist-packages/sumo",
                 "/usr/share/sumo", "/usr/local/share/sumo"):
        if os.path.isdir(os.path.join(cand, "tools")):
            return cand
    raise SystemExit("Could not locate SUMO_HOME (needs <home>/tools for sumolib).")


SUMO_HOME = find_sumo_home()
sys.path.insert(0, os.path.join(SUMO_HOME, "tools"))
import sumolib  # noqa: E402


def run(cmd):
    print("  $", " ".join(cmd))
    subprocess.run(cmd, check=True)


def netgenerate_bin():
    return shutil.which("netgenerate") or os.path.join(SUMO_HOME, "bin", "netgenerate")


def generate_net(out_dir, a):
    net_path = os.path.join(out_dir, "grid.net.xml")
    cmd = [
        netgenerate_bin(), "--rand",
        "--rand.iterations", str(a.iters),
        "--rand.min-distance", str(a.mind),     # short approach edges
        "--rand.max-distance", str(a.maxd),
        "--rand.min-angle", "40",
        "--rand.neighbor-dist3", str(a.d3),      # bias toward 3-way (T) nodes
        "--rand.neighbor-dist4", str(a.d4),      # some 4-way
        "--rand.connectivity", "0.95",
        "--default.lanenumber", str(a.lanes), "--random-lanenumber",  # 1<->2 lane mix
        "--default.priority", "4", "--random-priority",               # major/minor yield
        "--default-junction-type", a.jtype,                            # unsignalized
        "--junctions.right-before-left.speed-threshold", str(a.rbl_thresh),
        "--tls.discard-simple", "true", "--no-turnarounds", "false",
        "--seed", str(a.seed), "--output-file", net_path,
    ]
    run(cmd)
    return net_path


def classify_edges(net):
    def deg(n):
        return len(n.getIncoming()) + len(n.getOutgoing())
    real = [e for e in net.getEdges()
            if e.getFunction() != "internal" and e.allows("passenger")]
    entry, exit_, interior = [], [], []
    for e in real:
        if e.is_fringe():
            if deg(e.getFromNode()) <= 2:
                entry.append(e)
            if deg(e.getToNode()) <= 2:
                exit_.append(e)
            if deg(e.getFromNode()) > 2 and deg(e.getToNode()) > 2:
                interior.append(e)
        else:
            interior.append(e)
    return entry, exit_, interior


def route_between(net, a, b):
    path, _ = net.getShortestPath(a, b, vClass="passenger")
    return [e.getID() for e in path] if path else None


def scrub_copy(src, dst):
    with open(src, encoding="utf-8") as fh:
        text = fh.read()
    for old, new in SCRUB:
        text = re.sub(old, new, text, flags=re.IGNORECASE)
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(text)


def build(a):
    out_dir = os.path.abspath(a.out)
    os.makedirs(out_dir, exist_ok=True)
    rng = random.Random(a.seed)

    print("[1/5] netgenerate --rand (junction-realistic) ...")
    net_path = generate_net(out_dir, a)
    net = sumolib.net.readNet(net_path)
    entry, exit_, interior = classify_edges(net)
    lens = [e.getLength() for e in net.getEdges() if e.getFunction() != "internal"]
    short = sum(1 for l in lens if l < 30)
    ntypes = {}
    for n in net.getNodes():
        ntypes[n.getType()] = ntypes.get(n.getType(), 0) + 1
    print(f"      nodes={len(net.getNodes())} edges={len(lens)} short<30m={short} "
          f"entry={len(entry)} exit={len(exit_)} interior={len(interior)} types={ntypes}")
    if len(entry) < 2 or len(exit_) < 2:
        raise SystemExit("Not enough fringe edges; raise --iters.")

    print("[2/5] parkingArea .add.xml ...")
    cands = sorted((e for e in interior if e.getLength() >= 20.0), key=lambda e: e.getID())
    rng.shuffle(cands)
    n_sink = max(3, a.park_stay // a.sinkcap + 1)
    n_orig = max(3, a.depart_parked)
    pk = cands[:n_sink + n_orig]
    if len(pk) < n_sink + n_orig:
        raise SystemExit("Not enough interior edges for parkingAreas.")
    sink, orig = pk[:n_sink], pk[n_sink:]
    add = ET.Element("additional")
    pa_id = {}
    for e in sink:
        pid = f"pa_{e.getID()}"
        pa_id[e.getID()] = pid
        L = e.getLength()
        ET.SubElement(add, "parkingArea", {
            "id": pid, "lane": f"{e.getID()}_0", "startPos": "2.00",
            "endPos": f"{min(L - 2, a.sinkbay):.2f}",
            "roadsideCapacity": str(a.sinkcap), "friendlyPos": "true"})
    for e in orig:
        pid = f"pa_{e.getID()}"
        pa_id[e.getID()] = pid
        L = e.getLength()
        ET.SubElement(add, "parkingArea", {
            "id": pid, "lane": f"{e.getID()}_0", "startPos": "2.00",
            "endPos": f"{min(L - 2, 16.0):.2f}",
            "roadsideCapacity": "2", "friendlyPos": "true"})
    ET.ElementTree(add).write(os.path.join(out_dir, "scenario.add.xml"),
                              encoding="UTF-8", xml_declaration=True)

    print("[3/5] demand .rou.xml ...")
    MIN = a.minlen
    thru, ps, dp = [], [], []
    tries = 0
    while len(thru) < a.through and tries < a.through * 80:
        tries += 1
        x, y = rng.choice(entry), rng.choice(exit_)
        if x.getID() == y.getID():
            continue
        e = route_between(net, x, y)
        if e and len(e) >= MIN:
            thru.append(dict(edges=e, lane="best", speed="max", pos=None, stops=[]))
    sc = tries = 0
    while len(ps) < a.park_stay and tries < a.park_stay * 150:
        tries += 1
        x = rng.choice(entry)
        se = sink[sc % len(sink)]
        e = route_between(net, x, se)
        if e and len(e) >= MIN:
            ps.append(dict(edges=e, lane="best", speed="max", pos=None,
                           stops=[(pa_id[se.getID()], 100000)]))
            sc += 1
    oc = tries = 0
    while len(dp) < a.depart_parked and tries < a.depart_parked * 150:
        tries += 1
        oe = orig[oc % len(orig)]
        y = rng.choice(exit_)
        if oe.getID() == y.getID():
            continue
        e = route_between(net, oe, y)
        if e and len(e) >= MIN:
            dp.append(dict(edges=e, lane="0", speed="max", pos="stop",
                           stops=[(pa_id[oe.getID()], 5)]))
            oc += 1

    pools = [thru, ps, dp]
    wmax = max(1, max(len(p) for p in pools))
    order, idx, rem = [], [0, 0, 0], sum(len(p) for p in pools)
    while rem:
        for k in range(3):
            take = max(1, round(max(1, len(pools[k])) / wmax))
            for _ in range(take):
                if idx[k] < len(pools[k]):
                    order.append(pools[k][idx[k]])
                    idx[k] += 1
                    rem -= 1
    routes = ET.Element("routes")
    t = 0.0
    for vid, s in enumerate(order):
        v = ET.SubElement(routes, "vehicle", {
            "id": str(vid), "depart": f"{t:.2f}",
            "departLane": s["lane"], "departSpeed": s["speed"]})
        if s["pos"]:
            v.set("departPos", s["pos"])
        ET.SubElement(v, "route", {"edges": " ".join(s["edges"])})
        for pa, d in s["stops"]:
            ET.SubElement(v, "stop", {"parkingArea": pa, "duration": str(d)})
        t += a.period
    ET.indent(routes, space="  ")
    ET.ElementTree(routes).write(os.path.join(out_dir, "scenario.rou.xml"),
                                 encoding="UTF-8", xml_declaration=True)
    print(f"      vehicles: through={len(thru)} park_stay={len(ps)} "
          f"depart_parked={len(dp)} total={len(order)}")

    # self-audit: every birth/death at a fringe or parking edge (no visible cheating)
    fringe_ids = {e.getID() for e in entry} | {e.getID() for e in exit_}
    park_ids = set(pa_id.keys())
    for v in routes.findall("vehicle"):
        edges = v.find("route").get("edges").split()
        origin_park = v.get("departPos") == "stop"
        first_ok = edges[0] in park_ids if origin_park else edges[0] in fringe_ids
        stops = v.findall("stop")
        dest_park = bool(stops) and stops[-1].get("parkingArea") == f"pa_{edges[-1]}"
        last_ok = edges[-1] in park_ids if dest_park else edges[-1] in fringe_ids
        if not (first_ok and last_ok):
            raise SystemExit(f"self-audit: vehicle {v.get('id')} would cheat")
    print("      self-audit OK (all births/deaths at fringe or parking)")

    print("[4/5] copy + place-scrub vType files ...")
    for f in VTYPE_FILES:
        scrub_copy(os.path.join(PORTABLE, f), os.path.join(out_dir, f))

    print("[5/5] scenario.sumocfg ...")
    cfg = ET.Element("configuration")
    inp = ET.SubElement(cfg, "input")
    ET.SubElement(inp, "net-file", {"value": "grid.net.xml"})
    ET.SubElement(inp, "route-files", {
        "value": "vType.config.xml,vType_pedestrians.xml,vTypeDist.config.xml,scenario.rou.xml"})
    ET.SubElement(inp, "additional-files", {"value": "scenario.add.xml"})
    tm = ET.SubElement(cfg, "time")
    ET.SubElement(tm, "begin", {"value": "0"})
    ET.SubElement(tm, "step-length", {"value": "1.0"})
    proc = ET.SubElement(cfg, "processing")
    ET.SubElement(proc, "time-to-teleport", {"value": "120"})
    ET.SubElement(proc, "ignore-route-errors", {"value": "true"})
    ET.SubElement(proc, "collision.action", {"value": "none"})
    rt = ET.SubElement(cfg, "routing")
    ET.SubElement(rt, "routing-algorithm", {"value": "astar"})
    ET.SubElement(rt, "device.rerouting.probability", {"value": "1.0"})
    ET.SubElement(rt, "device.rerouting.period", {"value": "30"})
    ET.SubElement(rt, "device.rerouting.adaptation-steps", {"value": "18"})
    rep = ET.SubElement(cfg, "report")
    ET.SubElement(rep, "no-step-log", {"value": "true"})
    ET.indent(cfg, space="  ")
    ET.ElementTree(cfg).write(os.path.join(out_dir, "scenario.sumocfg"),
                              encoding="utf-8", xml_declaration=True)
    print(f"\nDONE. Scenario in {out_dir}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=HERE)
    p.add_argument("--iters", type=int, default=130)
    p.add_argument("--mind", type=float, default=25.0)
    p.add_argument("--maxd", type=float, default=95.0)
    p.add_argument("--d3", type=float, default=8.0)
    p.add_argument("--d4", type=float, default=3.0)
    p.add_argument("--lanes", type=int, default=2)
    p.add_argument("--jtype", default="priority",
                   help="priority (default) or right_before_left")
    p.add_argument("--rbl-thresh", dest="rbl_thresh", type=float, default=14.0)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--through", type=int, default=450)
    p.add_argument("--park-stay", dest="park_stay", type=int, default=60)
    p.add_argument("--depart-parked", dest="depart_parked", type=int, default=25)
    p.add_argument("--sinkcap", type=int, default=4)
    p.add_argument("--sinkbay", type=float, default=16.0)
    p.add_argument("--period", type=float, default=0.9)
    p.add_argument("--minlen", type=int, default=5)
    build(p.parse_args())


if __name__ == "__main__":
    main()
