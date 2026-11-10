import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Screw seat heights in frame L (no calls): bridge board top, A1 flange top at its holes, Orin carrier top at its holes."""
import sys, math, numpy as np
from build123d import import_step, Box, Pos
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from check import xf, H2L
from place import X, T
from orin_housing import BRIDGE_HOLES, LIDAR_HOLES, ORIN_HOLES
def top_at(solids, x, y, zlo, zhi, r=3.0):  # highest material z within r of (x, y) between zlo, zhi
    probe = Pos(x, y, (zlo + zhi) / 2) * Box(2 * r, 2 * r, zhi - zlo); best = None
    for s in solids:
        i = s.intersect(probe); i = [] if i is None else i if isinstance(i, list) else [i]
        for p in i:
            if p.volume > 1e-6: z = p.bounding_box().max.Z; best = z if best is None else max(best, z)
    return best
br = [xf(s, np.load("T_bridge_L.npy")) for s in import_step("a1_bridge.step").solids()]
print("bridge top at holes", [round(top_at(br, x, y, 83.0, 86.0, 1.8), 3) for x, y in BRIDGE_HOLES])
KIT = ("stand", "devboard", "connector-7pin")
a1 = [xf(s, np.load("T_a1_L.npy")) for c in import_step("a1.step").children if not c.label.startswith(KIT) for s in c.solids()]
print("A1 flange top at holes", [round(top_at(a1, x, y, 111.0, 116.0, 2.0), 3) for x, y in LIDAR_HOLES])
o = import_step("orin_assm.step"); Mo = H2L @ X["orin"]
car = [xf(s, Mo) for c in o.children if c.label.startswith("600-") for s in c.solids()]
print("Orin carrier top at holes", [round(top_at(car, x, y, 35.0, 40.0, 2.0) or -1, 3) for x, y in ORIN_HOLES])
def segs(solids, x, y, zlo, zhi):  # material z-intervals of a thin column at (x, y)
    col = Pos(x, y, (zlo + zhi) / 2) * Box(0.3, 0.3, zhi - zlo); out = []
    for s in solids:
        i = s.intersect(col); i = [] if i is None else i if isinstance(i, list) else [i]
        out += [(round(p.bounding_box().min.Z, 2), round(p.bounding_box().max.Z, 2)) for p in i if p.volume > 1e-7]
    return sorted(out)
x, y = LIDAR_HOLES[0]
for dx in (0.0, 1.8, 2.6, 3.5):
    print("A1 column at hole0 +", dx, segs(a1, x + dx, y, 105, 140))
