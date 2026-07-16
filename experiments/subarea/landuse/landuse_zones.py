import sumolib
import sys

NET = "synth_macro.net.xml"
CORE_MIN = 3600.0
CORE_MAX = 5100.0

def edge_center(e):
    shape = e.getShape()
    # average all shape points (works fine for straight grid edges)
    xs = [p[0] for p in shape]
    ys = [p[1] for p in shape]
    return sum(xs) / len(xs), sum(ys) / len(ys)

def is_commercial(e):
    x, y = edge_center(e)
    return CORE_MIN <= x <= CORE_MAX and CORE_MIN <= y <= CORE_MAX

def classify(net):
    commercial = []
    residential = []
    for e in net.getEdges():
        if e.getFunction() == "internal":
            continue
        if is_commercial(e):
            commercial.append(e)
        else:
            residential.append(e)
    return commercial, residential

if __name__ == "__main__":
    net = sumolib.net.readNet(NET)
    commercial, residential = classify(net)
    print(f"commercial edges: {len(commercial)}")
    print(f"residential edges: {len(residential)}")
    total = len(commercial) + len(residential)
    print(f"commercial edge-count fraction: {len(commercial)/total:.4f}")

    com_len = sum(e.getLength() for e in commercial)
    res_len = sum(e.getLength() for e in residential)
    tot_len = com_len + res_len
    print(f"commercial length fraction: {com_len/tot_len:.4f}")
    print(f"residential length fraction: {res_len/tot_len:.4f}")
