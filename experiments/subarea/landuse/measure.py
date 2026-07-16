import xml.etree.ElementTree as ET
import sumolib
from landuse_zones import NET, classify

def edge_zone_map(net):
    commercial, residential = classify(net)
    zone = {}
    for e in commercial:
        zone[e.getID()] = "commercial"
    for e in residential:
        zone[e.getID()] = "residential"
    return zone

def measure(rou_file, zone):
    tree = ET.parse(rou_file)
    root = tree.getroot()
    n = 0
    origin_residential = 0
    dest_commercial = 0
    origin_commercial = 0
    dest_residential = 0
    for veh in root.iter("vehicle"):
        route = veh.find("route")
        if route is None:
            continue
        edges = route.get("edges").split()
        if not edges:
            continue
        first = edges[0]
        last = edges[-1]
        n += 1
        if zone.get(first) == "residential":
            origin_residential += 1
        elif zone.get(first) == "commercial":
            origin_commercial += 1
        if zone.get(last) == "commercial":
            dest_commercial += 1
        elif zone.get(last) == "residential":
            dest_residential += 1
    return {
        "n": n,
        "origin_residential_frac": origin_residential / n,
        "origin_commercial_frac": origin_commercial / n,
        "dest_commercial_frac": dest_commercial / n,
        "dest_residential_frac": dest_residential / n,
    }

if __name__ == "__main__":
    net = sumolib.net.readNet(NET)
    zone = edge_zone_map(net)

    lu = measure("lu.rou.xml", zone)
    uni = measure("uni.rou.xml", zone)

    print("=== WEIGHTED (lu) ===")
    for k, v in lu.items():
        print(f"  {k}: {v}")
    print("=== UNIFORM (uni) ===")
    for k, v in uni.items():
        print(f"  {k}: {v}")

    ratio_dest = lu["dest_commercial_frac"] / uni["dest_commercial_frac"]
    ratio_origin_res = lu["origin_residential_frac"] / uni["origin_residential_frac"]
    ratio_origin_com_suppression = uni["origin_commercial_frac"] / lu["origin_commercial_frac"]
    print(f"\nratio (weighted dest-commercial / uniform dest-commercial)          = {ratio_dest:.3f}")
    print(f"ratio (weighted origin-residential / uniform origin-residential)    = {ratio_origin_res:.3f}  (ceiling-limited: uniform baseline already ~97% residential)")
    print(f"ratio (uniform origin-commercial / weighted origin-commercial)      = {ratio_origin_com_suppression:.3f}  (symmetric suppression of commercial-as-origin)")
