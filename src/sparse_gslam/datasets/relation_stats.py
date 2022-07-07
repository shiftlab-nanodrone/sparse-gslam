import math
from tabulate import tabulate


def gen_stats(dname: str):
    f = open(f"{dname}/{dname}.relations").readlines()
    sx = 0.0
    sy = 0.0
    st = 0.0
    for line in f:
        line = line.split(" ")
        x = float(line[2])
        y = float(line[3])
        theta = float(line[7])

        sx += abs(x)
        sy += abs(y)
        st += abs(theta)
    sx /= len(f)
    sy /= len(f)
    st /= len(f)
    return dname, sx, sy, math.hypot(sx, sy), math.degrees(st)

print(tabulate([
    gen_stats("intel-lab"),
    gen_stats("aces"),
    gen_stats("mit-killian")
], headers=["dataset", "mean x dist (m)", "mean y dist (m)", "mean dist (m)", "mean angle (degrees)"], floatfmt=".10f"))