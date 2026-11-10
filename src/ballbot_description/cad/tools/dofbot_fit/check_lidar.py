import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""A1 (no dev-kit board/stands) on the 20+5 standoffs: interference + clearances vs cover, bridge, standoffs -> T_a1_L.npy."""
import sys, math, numpy as np
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from build123d import import_step
from check import xf, bbox_hit, housing_part, O
from lidar_standoffs import LIDAR_STACK
from orin_housing import Z_TOP, LIDAR_PAD, SCAN, LIDAR_HOLES
KIT = ("stand", "devboard", "connector-7pin")  # dev-kit parts, not used
def Hm(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
Rz = lambda a: np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
SO = LIDAR_PAD + LIDAR_STACK
Ta = Hm(Rz(math.pi), [SCAN[0], SCAN[1], Z_TOP + SO]) @ Hm(np.eye(3), [90.55, 132.92, -744.29])  # flange z744.29
np.save("T_a1_L.npy", Ta)  # before the slow checks
a1 = [(c.label, xf(s, Ta)) for c in import_step("a1.step").children if not c.label.startswith(KIT) for s in c.solids()]
Tb = np.load("T_bridge_L.npy"); br = [(s.label, xf(s, Tb)) for s in import_step("a1_bridge.step").solids()]
so = [(s.label, s) for s in import_step(O + "lidar_standoffs.step").children]
cover = housing_part("orin_housing_cover")
def vol(a, b):
    if not bbox_hit(a, b): return 0.0
    r = a.intersect(b); r = [] if r is None else r if isinstance(r, list) else [r]; return sum(x.volume for x in r)
for p in [(-118.55, -160.92), (-118.55, -104.92), (-48.55, -152.92), (-48.55, -112.92)]:  # A1 holes vs pads
    q = Ta @ np.array([p[0], p[1], 744.29, 1]); print("A1 hole", np.round(q[:3], 2), "nearest pad",
          min(LIDAR_HOLES, key=lambda h: math.dist(h, q[:2])))
print("intersections...", flush=True); print("standoff top z", round(max(s.bounding_box().max.Z for _, s in so), 3), "| A1 mount face z", Z_TOP + SO)
lo = min(a1, key=lambda x: x[1].bounding_box().min.Z); print("A1 lowest:", lo[0], round(lo[1].bounding_box().min.Z - Z_TOP, 2), "mm above cover top")
cz = cover.bounding_box().max.Z; az = min(x.bounding_box().min.Z for _, x in a1)
print(f"cover top (pads) z {cz:.2f} | A1 lowest z {az:.2f} -> cover x A1 impossible, gap {az - cz:.2f} mm")
def pairs(A, B): return [(la, lb, a, b) for la, a in A for lb, b in B if bbox_hit(a, b)]
for what, A, B in (("A1 x standoffs", a1, so), ("A1 x bridge", a1, br), ("cover x standoffs", [("cover", cover)], so)):
    ps = pairs(A, B); v = sum(vol(a, b) for *_, a, b in ps)
    print(f"{what}: {len(ps)} bbox-overlapping pairs {[(la, lb) for la, lb, *_ in ps][:6]}, intersection {v:.4f} mm3", flush=True)
bb = [b.bounding_box() for _, b in br]; A1b = [x.bounding_box() for _, x in a1]
print("bridge bbox z top", round(max(b.max.Z for b in bb), 2), "| A1 x-range", round(min(b.min.X for b in A1b), 1),
      round(max(b.max.X for b in A1b), 1), "| bridge x-range", round(min(b.min.X for b in bb), 1), round(max(b.max.X for b in bb), 1))
