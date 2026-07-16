import sumolib
from landuse_zones import NET, classify

BEGIN = 0
END = 3600

def write_weight_file(path, commercial, residential, com_value, res_value):
    with open(path, "w") as f:
        f.write('<edgedata>\n')
        f.write(f'    <interval begin="{BEGIN}" end="{END}">\n')
        for e in residential:
            f.write(f'        <edge id="{e.getID()}" value="{res_value}"/>\n')
        for e in commercial:
            f.write(f'        <edge id="{e.getID()}" value="{com_value}"/>\n')
        f.write('    </interval>\n')
        f.write('</edgedata>\n')

if __name__ == "__main__":
    net = sumolib.net.readNet(NET)
    commercial, residential = classify(net)

    # src: trips START in residential -> residential gets high weight
    write_weight_file("lu.src.xml", commercial, residential, com_value=0.1, res_value=1.0)
    # dst: trips END in commercial -> commercial gets high weight
    write_weight_file("lu.dst.xml", commercial, residential, com_value=1.0, res_value=0.1)

    print("wrote lu.src.xml and lu.dst.xml")
    print(f"commercial edges={len(commercial)} residential edges={len(residential)}")
