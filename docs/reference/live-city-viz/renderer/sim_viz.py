#!/usr/bin/env python3
"""sim_viz.py -- offline HTML traffic-replay exporter.

Ports ONLY the payload builder from the Sim.Viz C# tool (pjanec/SumoSharp,
src/Sim.Viz/{Payload.cs,Program.cs}) to Python. The front-end renderer
(template.html + template.js, copied verbatim into ./templates/ next to this
script) is REUSED AS-IS -- this script never touches rendering logic, it only
produces the REPLAY_DATA JSON the committed template.js already knows how to
draw.

REPLAY_DATA shape (must match template.js's field names EXACTLY -- these are
camelCase because Program.cs serializes with JsonNamingPolicy.CamelCase):

    { "scenes": [ SCENE, ... ] }

    SCENE = {
      "name": str, "desc": str,
      "view": [minX, minY, maxX, maxY],
      "network": NETWORK | null,
      "vdim": [length, width],
      "dt": float,
      "frames": [ FRAME, ... ],
      "labels": [str, ...] | null,      (not produced by this tool)
      "incident": [...] | null,          (not produced by this tool)
      "boundary": [...] | null,          (not produced by this tool)
      "pois": [ {kind, x, y, label?}, ... ]   (OPTIONAL static layer, --pois only;
                key ABSENT when --pois not given -> byte-identical to pre-POI tool)
      "zones": [ {id, type, polygon:[x0,y0,...]}, ... ]     (OPTIONAL, --zones only)
      "buildings": [ {id, type, polygon:[x0,y0,...]}, ... ] (OPTIONAL, --buildings only)
      "parkingLots": [ {id, polygon:[x0,y0,...]}, ... ]     (OPTIONAL, derived from --pois'
                pois/v2 `parking_lot.polygon` records; key ABSENT when none present)
      "parks": [ {id, polygon:[x0,y0,...],                  (OPTIONAL, derived from --pois'
                  meetAreas:[{id,x,y,groupSize?}, ...]}, ... ] pois/v2 `park.polygon`/
                `meet_areas`; key ABSENT when none present)
    }

    Stage 4 (docs/SUBAREA-DEMO-CITY-DESIGN.md sec 5): the four new keys above are all
    ADDITIVE and gated on either a new CLI flag (--zones/--buildings, both default None) or
    on the --pois file actually containing pois/v2 polygon-bearing records
    (parking_lot/park -- absent from v1 pois.json fixtures). With none of that present the
    scene dict is built and serialized in the exact same shape as before this change, so
    the payload -- and therefore the whole replay HTML byte-for-byte -- is unchanged.

    NETWORK = {
      "lanes": [ {"id", "edgeId", "index", "width", "shape": [x0,y0,x1,y1,...], "ped"?: true}, ... ],
      "junctions": [ {"id", "shape": [x0,y0,...]}, ... ],
      "tls": [],       -- stubbed empty (see RESULTS-simviz-port.md)
      "signals": []    -- stubbed empty
    }

    FRAME = { "v": [ [x,y,angleDeg] | null, ... ], "d": [ [x,y,radius,kind] | null, ... ] }
      -- v uses FIXED SLOTS: slot i is always the same vehicle across every
         frame in the scene; a vehicle absent this frame is `null` in its slot.
      -- naviDegree convention (0 = +Y/north, clockwise) -- this is exactly
         SUMO FCD's own `angle` attribute, passed through unchanged.
      -- d (discs) carries PEDESTRIANS parsed from SUMO fcd `<person>` elements.
         Each disc is [x, y, radius, kind] with kind=2 (pedestrian, the template's
         DISC_LABELS convention). Discs use FIXED SLOTS too: slot i is the same
         person across every frame; absent -> `null` (mirrors v). Persons may be
         interleaved in the vehicle FCD or supplied separately via --ped-fcd; the
         two streams are merged by timestep. With NO persons at all, d stays [] and
         output is byte-identical to the vehicle-only tool.

Coordinates are left in raw SUMO world space, rounded to 2 dp. template.js's
worldToScreen() does both the camera fit AND the Y-flip
(screenY = -worldY*scale + offsetY) -- see template.js line ~104 -- so this
script must NOT pre-flip Y.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

# --------------------------------------------------------------------------
# sumolib on the path
# --------------------------------------------------------------------------
def _add_sumo_tools_to_path() -> None:
    candidates = []
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        candidates.append(os.path.join(sumo_home, "tools"))
    candidates.append("/usr/local/lib/python3.11/dist-packages/sumo/tools")
    candidates.append("/usr/share/sumo/tools")
    for c in candidates:
        if c and os.path.isdir(c) and c not in sys.path:
            sys.path.insert(0, c)


_add_sumo_tools_to_path()
import sumolib  # noqa: E402  (path set up just above)

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_HTML_PATH = os.path.join(HERE, "templates", "template.html")
TEMPLATE_JS_PATH = os.path.join(HERE, "templates", "template.js")

DEFAULT_VDIM = (5.0, 2.0)


# --------------------------------------------------------------------------
# Rounding -- matches C#'s PayloadBuilder.R(): Math.Round(v, 2, AwayFromZero).
# --------------------------------------------------------------------------
def R(v: float) -> float:
    if v >= 0:
        return math.floor(v * 100.0 + 0.5) / 100.0
    return -math.floor(-v * 100.0 + 0.5) / 100.0


class BBox:
    """Running [minX, minY, maxX, maxY] accumulator (mirrors Program.cs Track())."""

    def __init__(self) -> None:
        self.min_x = math.inf
        self.min_y = math.inf
        self.max_x = -math.inf
        self.max_y = -math.inf

    def track(self, x: float, y: float) -> None:
        if x < self.min_x:
            self.min_x = x
        if y < self.min_y:
            self.min_y = y
        if x > self.max_x:
            self.max_x = x
        if y > self.max_y:
            self.max_y = y

    def as_view(self) -> list:
        if math.isinf(self.min_x):
            return [0.0, 0.0, 1.0, 1.0]
        return [R(self.min_x), R(self.min_y), R(self.max_x), R(self.max_y)]


# --------------------------------------------------------------------------
# Network payload (LanePayload / JunctionPayload -- TL/signals stubbed empty)
# --------------------------------------------------------------------------
def build_network(net, bbox: BBox) -> dict:
    lanes = []
    for edge in net.getEdges(withInternal=True):
        edge_id = edge.getID()
        for lane in edge.getLanes():
            flat = []
            for x, y in lane.getShape():
                fx, fy = R(x), R(y)
                flat.append(fx)
                flat.append(fy)
                bbox.track(fx, fy)
            entry = {
                "id": lane.getID(),
                "edgeId": edge_id,
                "index": lane.getIndex(),
                "width": lane.getWidth(),  # C# passes lane.Width through unrounded
                "shape": flat,
            }
            # Mark pedestrian-only lanes (sidewalks) so the renderer can draw them
            # as a distinct lighter "footpath" band vs dark car asphalt -- otherwise
            # a sidewalk reads identically to a car lane and peds on it look like
            # they're sharing the carriageway. Additive: absent on car lanes.
            if lane.allows("pedestrian") and not lane.allows("passenger"):
                entry["ped"] = True
            lanes.append(entry)

    junctions = []
    for node in net.getNodes():
        shape = node.getShape()
        if not shape:
            continue
        flat = []
        for x, y in shape:
            fx, fy = R(x), R(y)
            flat.append(fx)
            flat.append(fy)
            bbox.track(fx, fy)
        junctions.append({"id": node.getID(), "shape": flat})

    return {
        "lanes": lanes,
        "junctions": junctions,
        # Traffic-light logics and signal heads are OPTIONAL (template.js tolerates
        # empty arrays: precomputeNetwork() and drawSignals() are both null/empty
        # guarded). Stubbed here -- see RESULTS-simviz-port.md.
        "tls": [],
        "signals": [],
    }


# --------------------------------------------------------------------------
# vType dims (best-effort; only used if --rou is given and resolvable)
# --------------------------------------------------------------------------
def resolve_vdim_from_rou(rou_paths: list, first_vehicle_id: str, vdim_override) -> tuple:
    if vdim_override is not None:
        return vdim_override

    veh_type = None
    type_dims = {}
    for path in rou_paths:
        try:
            for _, el in ET.iterparse(path, events=("end",)):
                if el.tag == "vehicle" and veh_type is None and el.get("id") == first_vehicle_id:
                    veh_type = el.get("type")
                elif el.tag == "vType":
                    length = el.get("length")
                    width = el.get("width")
                    if length is not None and width is not None:
                        type_dims[el.get("id")] = (float(length), float(width))
                el.clear()
        except (ET.ParseError, OSError):
            continue

    if veh_type and veh_type in type_dims:
        return type_dims[veh_type]
    return DEFAULT_VDIM


# --------------------------------------------------------------------------
# FCD parsing -- streaming (ET.iterparse + clear), two passes:
#   pass 1: collect the sorted set of distinct vehicle ids -> fixed slot index
#   pass 2: build the actual frame list using those slots
# Timesteps with ZERO vehicles are dropped (mirrors Program.cs: byTime is only
# ever populated from AllPoints, so an empty <timestep/> never gets an entry).
# --------------------------------------------------------------------------
def collect_vehicle_ids(fcd_path: str, begin: float, end: float) -> list:
    ids = set()
    cur_time = None
    in_range = False
    for event, el in ET.iterparse(fcd_path, events=("start", "end")):
        if event == "start" and el.tag == "timestep":
            cur_time = float(el.get("time"))
            in_range = begin <= cur_time <= end
        elif event == "end" and el.tag == "vehicle":
            if in_range:
                ids.add(el.get("id"))
            el.clear()
        elif event == "end" and el.tag == "timestep":
            el.clear()
    return sorted(ids)  # ordinal string sort == StringComparer.Ordinal for ASCII ids


def build_vehicle_slots(fcd_path: str, begin: float, end: float, slot_by_id: dict, bbox: BBox) -> dict:
    """time -> fixed-slot vehicle list [[x,y,angle]|null, ...].

    Only timesteps carrying >=1 in-range vehicle get an entry (mirrors the
    original build_frames: an all-empty timestep never produced a frame).
    """
    n_slots = len(slot_by_id)
    by_time = {}
    cur_time = None
    in_range = False
    cur_slots = None

    for event, el in ET.iterparse(fcd_path, events=("start", "end")):
        if event == "start" and el.tag == "timestep":
            cur_time = float(el.get("time"))
            in_range = begin <= cur_time <= end
            cur_slots = [None] * n_slots if in_range else None
        elif event == "end" and el.tag == "vehicle":
            if in_range:
                vid = el.get("id")
                slot = slot_by_id.get(vid)
                if slot is not None:
                    x = float(el.get("x"))
                    y = float(el.get("y"))
                    a = float(el.get("angle"))
                    bbox.track(x, y)
                    cur_slots[slot] = [R(x), R(y), R(a)]
            el.clear()
        elif event == "end" and el.tag == "timestep":
            if in_range and any(s is not None for s in (cur_slots or ())):
                by_time[cur_time] = cur_slots
            el.clear()
            cur_slots = None

    return by_time


# --------------------------------------------------------------------------
# Person (pedestrian) parsing -- SUMO fcd-schema <person id x y angle .../>.
# Persons appear inside <timestep> alongside <vehicle> (SUMO emits <person>
# when persons are present) and/or in a SEPARATE person-only FCD (--ped-fcd).
# Both are accepted; the paths list is scanned in order and MERGED by timestep
# into a single set of fixed per-person disc slots. Each disc is
# [x, y, radius, kind=2] -- kind 2 is the template's pedestrian convention.
# --------------------------------------------------------------------------
PED_KIND = 2  # template.js DISC_LABELS: 2 == "pedestrian"


# --------------------------------------------------------------------------
# POI (point-of-interest / "places") static layer -- OPTIONAL (--pois).
# deduce_pois.py emits either a bare list of POI dicts or a wrapper object
# {..., "pois": [ ... ]}; both are accepted. Each POI dict carries at least
# {id, kind, pos:[x,y]}. Coordinates are ALREADY in the net XY frame (same as
# vehicles/discs), so no transform -- just round to 2 dp like everything else.
# Emitted into the scene as a NEW top-level STATIC layer (not per-frame):
#   "pois": [ {"kind", "x", "y", "label"}, ... ]
# When --pois is absent the scene has NO "pois" key at all, so vehicle-only /
# ped+vehicle output stays byte-identical to the pre-POI tool.
# --------------------------------------------------------------------------
def load_pois(path: str, bbox: BBox) -> list:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    if isinstance(raw, dict):
        items = raw.get("pois", [])
    else:
        items = raw
    out = []
    for poi in items:
        pos = poi.get("pos")
        if not pos or len(pos) < 2:
            continue
        x, y = float(pos[0]), float(pos[1])
        bbox.track(x, y)
        entry = {"kind": poi.get("kind", ""), "x": R(x), "y": R(y)}
        label = poi.get("id")
        if label:
            entry["label"] = str(label)
        out.append(entry)
    return out


# --------------------------------------------------------------------------
# Zones / buildings (Stage 4, docs/SUBAREA-DEMO-CITY-DESIGN.md sec 5) -- OPTIONAL static
# polygon layers, --zones <zones.json> / --buildings <buildings.json>. Both files are
# `{ ..metadata.., "zones"|"buildings": [ {id, polygon|footprint:[[x,y],...], type, ...} ] }`
# (compose.py's zones/v1 / buildings/v1). Coordinates are already in the net XY frame, so
# only round + flatten (matches the POI/network convention) and bbox-track them so a
# district polygon poking past the agent/net extent still fits the camera view.
# --------------------------------------------------------------------------
def _flatten_ring(points, bbox: BBox) -> list:
    flat = []
    for pt in points:
        x, y = float(pt[0]), float(pt[1])
        fx, fy = R(x), R(y)
        flat.append(fx)
        flat.append(fy)
        bbox.track(fx, fy)
    return flat


def load_zones(path: str, bbox: BBox) -> list:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    items = raw.get("zones", []) if isinstance(raw, dict) else raw
    out = []
    for z in items:
        poly = z.get("polygon")
        if not poly:
            continue
        out.append({"id": z.get("id", ""), "type": z.get("type", ""), "polygon": _flatten_ring(poly, bbox)})
    return out


def load_buildings(path: str, bbox: BBox) -> list:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    items = raw.get("buildings", []) if isinstance(raw, dict) else raw
    out = []
    for b in items:
        poly = b.get("footprint") or b.get("polygon")
        if not poly:
            continue
        out.append({"id": b.get("id", ""), "type": b.get("type", ""), "polygon": _flatten_ring(poly, bbox)})
    return out


# --------------------------------------------------------------------------
# pois/v2 polygon-bearing records (parking_lot / park) -- parsed out of the SAME --pois
# file already read by load_pois() into two more OPTIONAL static layers. A v1 pois.json
# (no such records) yields two empty lists, so no new scene keys are added for it -- the
# byte-identical-without-new-input guarantee holds for existing v1 fixtures.
# --------------------------------------------------------------------------
def load_poi_polygons(path: str, bbox: BBox) -> tuple:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    items = raw.get("pois", []) if isinstance(raw, dict) else raw
    parking_lots = []
    parks = []
    for poi in items:
        kind = poi.get("kind")
        poly = poi.get("polygon")
        if kind == "parking_lot" and poly:
            parking_lots.append({"id": poi.get("id", ""), "polygon": _flatten_ring(poly, bbox)})
        elif kind == "park" and poly:
            meet_areas = []
            for m in poi.get("meet_areas") or []:
                mp = m.get("pos")
                if not mp or len(mp) < 2:
                    continue
                mx, my = R(float(mp[0])), R(float(mp[1]))
                bbox.track(mx, my)
                entry = {"id": m.get("id", ""), "x": mx, "y": my}
                if "group_size" in m:
                    entry["groupSize"] = m["group_size"]
                meet_areas.append(entry)
            parks.append({"id": poi.get("id", ""), "polygon": _flatten_ring(poly, bbox), "meetAreas": meet_areas})
    return parking_lots, parks


def collect_person_ids(paths: list, begin: float, end: float) -> list:
    ids = set()
    for path in paths:
        cur_time = None
        in_range = False
        for event, el in ET.iterparse(path, events=("start", "end")):
            if event == "start" and el.tag == "timestep":
                cur_time = float(el.get("time"))
                in_range = begin <= cur_time <= end
            elif event == "end" and el.tag == "person":
                if in_range:
                    ids.add(el.get("id"))
                el.clear()
            elif event == "end" and el.tag == "timestep":
                el.clear()
    return sorted(ids)


def build_person_discs(
    paths: list, begin: float, end: float, slot_by_pid: dict, radius: float, bbox: BBox
) -> dict:
    """time -> fixed-slot disc list [[x,y,radius,2]|null, ...], merged across paths."""
    n_slots = len(slot_by_pid)
    by_time = {}
    for path in paths:
        cur_time = None
        in_range = False
        cur_slots = None
        for event, el in ET.iterparse(path, events=("start", "end")):
            if event == "start" and el.tag == "timestep":
                cur_time = float(el.get("time"))
                in_range = begin <= cur_time <= end
                if in_range:
                    # Reuse the slot list already started by an earlier path at this
                    # same timestep so interleaved + separate streams MERGE.
                    cur_slots = by_time.get(cur_time)
                    if cur_slots is None:
                        cur_slots = [None] * n_slots
                        by_time[cur_time] = cur_slots
                else:
                    cur_slots = None
            elif event == "end" and el.tag == "person":
                if in_range:
                    pid = el.get("id")
                    slot = slot_by_pid.get(pid)
                    if slot is not None:
                        x = float(el.get("x"))
                        y = float(el.get("y"))
                        bbox.track(x, y)
                        cur_slots[slot] = [R(x), R(y), R(radius), PED_KIND]
                el.clear()
            elif event == "end" and el.tag == "timestep":
                el.clear()
                cur_slots = None
    return by_time


# --------------------------------------------------------------------------
# Frame assembly -- union the vehicle and person timesteps, sort ascending,
# and emit one FRAME per time carrying both channels on their fixed slots.
# With no persons, d stays [] and times == vehicle times -> byte-identical
# to the original vehicle-only output.
# --------------------------------------------------------------------------
def assemble_frames(v_by_time: dict, d_by_time: dict, n_vehicles: int, n_persons: int):
    all_times = sorted(set(v_by_time) | set(d_by_time))
    frames = []
    for t in all_times:
        v = v_by_time.get(t)
        if v is None:
            v = [None] * n_vehicles
        d = d_by_time.get(t) if n_persons else None
        if d is None:
            d = [None] * n_persons  # == [] when n_persons == 0
        frames.append({"v": v, "d": d})
    dt = all_times[1] - all_times[0] if len(all_times) > 1 else 1.0
    return frames, dt, all_times


# --------------------------------------------------------------------------
# HTML assembly -- exact tokens from template.html:
#   __SCENARIO_NAME__   (title text)
#   /*REPLAY_DATA*/      (the JSON payload, inside `const REPLAY_DATA = ...;`)
#   /*TEMPLATE_JS*/       (the verbatim template.js source)
# --------------------------------------------------------------------------
def write_html(scene: dict, title: str, out_path: str) -> None:
    with open(TEMPLATE_HTML_PATH, "r", encoding="utf-8") as f:
        html = f.read()
    with open(TEMPLATE_JS_PATH, "r", encoding="utf-8") as f:
        js = f.read()

    replay_data = {"scenes": [scene]}
    payload_json = json.dumps(replay_data, separators=(",", ":"), allow_nan=False)

    if "__SCENARIO_NAME__" not in html or "/*REPLAY_DATA*/" not in html or "/*TEMPLATE_JS*/" not in html:
        raise RuntimeError("template.html is missing one of the expected injection tokens")

    html = html.replace("__SCENARIO_NAME__", title)
    html = html.replace("/*REPLAY_DATA*/", payload_json)
    html = html.replace("/*TEMPLATE_JS*/", js)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--net", required=True, help="SUMO *.net.xml")
    p.add_argument("--fcd", required=True, help="SUMO FCD output *.fcd.xml (vehicles; may also carry interleaved <person> elements)")
    p.add_argument("--ped-fcd", default=None, help="optional separate person-only SUMO FCD *.fcd.xml (merged with --fcd by timestep)")
    p.add_argument("--ped-radius", type=float, default=0.25, help="pedestrian disc radius in metres (default 0.25)")
    p.add_argument("--pois", default=None, help="optional deduce_pois.py pois.json -- adds a static POI layer to the scene")
    p.add_argument("--zones", default=None, help="optional compose.py zones.json (zones/v1) -- adds a static district-tint polygon layer")
    p.add_argument("--buildings", default=None, help="optional compose.py buildings.json (buildings/v1) -- adds a static building-footprint polygon layer")
    p.add_argument("--out", required=True, help="output replay .html path")
    p.add_argument("--name", default=None, help="scene name (defaults to FCD basename)")
    p.add_argument("--desc", default=None, help="scene description (defaults to name)")
    p.add_argument("--begin", type=float, default=0.0, help="min FCD time (s) to include")
    p.add_argument("--end", type=float, default=1e9, help="max FCD time (s) to include")
    p.add_argument("--vdim", default=None, help="shared vehicle box dims 'length,width' (default 5,2)")
    p.add_argument("--rou", default=None, help="optional *.rou.xml (comma-separated OK) to resolve vdim from vType")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    if not os.path.exists(args.net):
        print(f"error: net file not found: {args.net}", file=sys.stderr)
        return 2
    if not os.path.exists(args.fcd):
        print(f"error: FCD file not found: {args.fcd}", file=sys.stderr)
        return 2
    if args.ped_fcd is not None and not os.path.exists(args.ped_fcd):
        print(f"error: ped FCD file not found: {args.ped_fcd}", file=sys.stderr)
        return 2
    if args.pois is not None and not os.path.exists(args.pois):
        print(f"error: pois file not found: {args.pois}", file=sys.stderr)
        return 2
    if args.zones is not None and not os.path.exists(args.zones):
        print(f"error: zones file not found: {args.zones}", file=sys.stderr)
        return 2
    if args.buildings is not None and not os.path.exists(args.buildings):
        print(f"error: buildings file not found: {args.buildings}", file=sys.stderr)
        return 2

    scene_name = args.name or os.path.splitext(os.path.basename(args.fcd))[0]
    scene_desc = args.desc or scene_name

    print(f"loading net: {args.net}", file=sys.stderr)
    net = sumolib.net.readNet(args.net, withInternal=True)

    bbox = BBox()
    network = build_network(net, bbox)
    print(f"  lanes={len(network['lanes'])} junctions={len(network['junctions'])}", file=sys.stderr)

    print(f"scanning FCD for vehicle ids: {args.fcd}", file=sys.stderr)
    ordered_ids = collect_vehicle_ids(args.fcd, args.begin, args.end)
    slot_by_id = {vid: i for i, vid in enumerate(ordered_ids)}
    print(f"  distinct vehicles={len(ordered_ids)}", file=sys.stderr)

    vdim_override = None
    if args.vdim:
        length_s, width_s = args.vdim.split(",")
        vdim_override = (float(length_s), float(width_s))

    if ordered_ids:
        rou_paths = [pth.strip() for pth in (args.rou.split(",") if args.rou else []) if pth.strip()]
        vdim = resolve_vdim_from_rou(rou_paths, ordered_ids[0], vdim_override)
    else:
        vdim = vdim_override or (0.0, 0.0)

    # Persons come from the vehicle FCD (interleaved) AND/OR a separate --ped-fcd.
    # A person id present in both streams shares one fixed slot; positions merge
    # by timestep. With no persons anywhere, everything below is a no-op and the
    # disc channel stays [].
    ped_paths = [args.fcd]
    if args.ped_fcd:
        ped_paths.append(args.ped_fcd)
    print(f"scanning for person ids: {ped_paths}", file=sys.stderr)
    ordered_pids = collect_person_ids(ped_paths, args.begin, args.end)
    slot_by_pid = {pid: i for i, pid in enumerate(ordered_pids)}
    print(f"  distinct persons={len(ordered_pids)}", file=sys.stderr)

    print(f"building vehicle slots: {args.fcd}", file=sys.stderr)
    v_by_time = build_vehicle_slots(args.fcd, args.begin, args.end, slot_by_id, bbox)

    d_by_time = {}
    if ordered_pids:
        print("building person discs", file=sys.stderr)
        d_by_time = build_person_discs(
            ped_paths, args.begin, args.end, slot_by_pid, args.ped_radius, bbox
        )

    frames, dt, times = assemble_frames(v_by_time, d_by_time, len(ordered_ids), len(ordered_pids))
    print(f"  frames={len(frames)} dt={dt}", file=sys.stderr)

    # Static POI layer (optional). Loaded (and bbox-tracked) BEFORE as_view() so
    # any POI just outside the agent/net extent still fits in the camera view.
    pois = None
    parking_lots, parks = [], []
    if args.pois:
        print(f"loading POIs: {args.pois}", file=sys.stderr)
        pois = load_pois(args.pois, bbox)
        parking_lots, parks = load_poi_polygons(args.pois, bbox)
        print(f"  pois={len(pois)} parking_lots={len(parking_lots)} parks={len(parks)}", file=sys.stderr)

    # Static district-tint / building-footprint layers (Stage 4). OPTIONAL -- keys omitted
    # entirely when their flag is absent so a no-arg run's payload is unchanged.
    zones = None
    if args.zones:
        print(f"loading zones: {args.zones}", file=sys.stderr)
        zones = load_zones(args.zones, bbox)
        print(f"  zones={len(zones)}", file=sys.stderr)

    buildings = None
    if args.buildings:
        print(f"loading buildings: {args.buildings}", file=sys.stderr)
        buildings = load_buildings(args.buildings, bbox)
        print(f"  buildings={len(buildings)}", file=sys.stderr)

    scene = {
        "name": scene_name,
        "desc": scene_desc,
        "view": bbox.as_view(),
        "network": network,
        "vdim": [vdim[0], vdim[1]],
        "dt": dt,
        "frames": frames,
    }
    # NEW static layer -- key omitted entirely when --pois is absent so the
    # vehicle(+ped)-only payload stays byte-identical to the pre-POI tool.
    if pois is not None:
        scene["pois"] = pois
    # Stage 4 static layers -- each key omitted entirely when its source is absent/empty,
    # so a no-arg (or v1-pois-only) run's payload is byte-identical to before this change.
    if zones is not None:
        scene["zones"] = zones
    if buildings is not None:
        scene["buildings"] = buildings
    if parking_lots:
        scene["parkingLots"] = parking_lots
    if parks:
        scene["parks"] = parks

    write_html(scene, scene_name, args.out)

    size = os.path.getsize(args.out)
    print(
        f"wrote {args.out}  ({size} bytes, {size / 1024.0 / 1024.0:.2f} MiB)  "
        f"lanes={len(network['lanes'])} frames={len(frames)} vehicles={len(ordered_ids)} "
        f"persons={len(ordered_pids)} pois={len(pois) if pois is not None else 0} "
        f"zones={len(zones) if zones is not None else 0} "
        f"buildings={len(buildings) if buildings is not None else 0} "
        f"parkingLots={len(parking_lots)} parks={len(parks)} "
        f"dt={dt} view={scene['view']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
