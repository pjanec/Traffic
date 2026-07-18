#!/usr/bin/env python3
"""
audit_nocheat.py — verify the no-visible-cheating rule for a cut+parking sub-area run.

Rule: a vehicle may be BORN (first appear) or DIE (disappear) only at
  (a) a FRINGE edge (a stub cut by the crop), or
  (b) OFF-ROAD inside a parkingArea (internal origin/destination sink).
Any birth/death on a visible internal, non-parking lane is a violation.

Births are authoritative from the route file: SUMO inserts a vehicle on route[0]
(at departPos). A vehicle with departPos="stop" is inserted already parked
off-road in the parkingArea on route[0]. Deaths are taken from tripinfo
(actual arrival edge) and cross-checked against the destination parking stop.

A route-intent audit trusts the parking assignment, so it cannot see a stop that
SUMO rejected at runtime (e.g. a lane that forbids stopping) — that leaves a car
materialising at rest on a visible lane. Pass an FCD file to add the authoritative
runtime birth check: a vehicle's FIRST appearance must be on a fringe edge or a
parking edge; anywhere else is a visible pop, whatever its speed.

Usage: audit_nocheat.py <sub.net.xml> <sub_parking.rou.xml> <sub_parking.add.xml> <out.tripinfo.xml> [out.fcd.xml]
"""
import sys, xml.etree.ElementTree as ET
import sumolib

net_f, rou_f, add_f, trip_f = sys.argv[1:5]
net = sumolib.net.readNet(net_f)
fringe = {e.getID() for e in net.getEdges()
          if e.getFunction() != 'internal' and e.is_fringe()}
park_edges = {pa.get('lane').rsplit('_', 1)[0]
              for pa in ET.parse(add_f).getroot().findall('parkingArea')}

edge = lambda lane: lane.rsplit('_', 1)[0]

# ---- parse route file: intent per vehicle ----
veh = {}
for v in ET.parse(rou_f).getroot().findall('vehicle'):
    vid = v.get('id')
    edges = v.find('route').get('edges').split()
    stops = v.findall('stop')
    origin_park = v.get('departPos') == 'stop'
    dest_park = False
    if stops:
        # a long-duration stop at the last edge is the destination sink
        last_stop = stops[-1]
        if last_stop.get('parkingArea') == f'pa_{edges[-1]}':
            dest_park = True
    veh[vid] = dict(first=edges[0], last=edges[-1],
                    origin_park=origin_park, dest_park=dest_park)

# ---- births ----
birth_viol = []
for vid, d in veh.items():
    if d['origin_park']:
        ok = d['first'] in park_edges          # pulled out of a lot (off-road birth)
    else:
        ok = d['first'] in fringe              # entered at the fringe
    if not ok:
        birth_viol.append((vid, d['first'], d['origin_park']))

# ---- deaths (actual arrival from tripinfo) ----
arrival = {}
for ti in ET.parse(trip_f).getroot().findall('tripinfo'):
    arrival[ti.get('id')] = edge(ti.get('arrivalLane'))

death_viol = []
completed = 0
for vid, alane in arrival.items():
    completed += 1
    d = veh.get(vid)
    if d is None:
        continue
    if d['dest_park']:
        ok = alane in park_edges               # parked off-road in a lot (the sink)
    else:
        ok = alane in fringe                    # left at the fringe
    if not ok:
        death_viol.append((vid, alane, d['dest_park']))

# vehicles that never appear in tripinfo = failed to insert or still parked at end
missing = sorted(set(veh) - set(arrival))

print(f"fringe edges: {len(fringe)}   parking edges: {len(park_edges)}")
print(f"vehicles: {len(veh)}   completed (in tripinfo): {completed}   missing: {len(missing)}")
print(f"BIRTH violations (popped on a visible lane): {len(birth_viol)}")
for x in birth_viol[:10]: print("   ", x)
print(f"DEATH violations (vanished on a visible lane): {len(death_viol)}")
for x in death_viol[:10]: print("   ", x)
if missing:
    print(f"missing-from-tripinfo (inspect): {missing[:10]}{' ...' if len(missing)>10 else ''}")
# ---- optional authoritative FCD birth check ----
fcd_viol = None
if len(sys.argv) > 5:
    fcd_viol = []
    seen = set()
    ok_edges = fringe | park_edges
    for _ev, el in ET.iterparse(sys.argv[5], events=('end',)):
        if el.tag != 'timestep':
            continue
        for v in el.findall('vehicle'):
            vid = v.get('id')
            if vid in seen:
                continue
            seen.add(vid)
            e = edge(v.get('lane'))
            if e not in ok_edges:                # first appearance on a visible lane
                fcd_viol.append((vid, e, float(v.get('speed'))))
        el.clear()
    print(f"FCD birth violations (first appeared on a visible lane): {len(fcd_viol)}")
    for x in fcd_viol[:10]: print("   ", x)

bad = bool(birth_viol or death_viol or fcd_viol)
verdict = "PASS" if not bad else "FAIL"
print(f"\nNO-CHEATING AUDIT: {verdict}")
