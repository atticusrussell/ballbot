import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import numpy as np, math
from build123d import *
from check import xf, bbox_hit, housing_part
O = "/home/atticus/workspace/ballbot/src/ballbot_description/cad/out/"
cover = housing_part("orin_housing_cover")
SCAN, ZTOP, SO = (-17.0, 0.0), 78.0, 3.0 + 24.6  # pad + kit stack (15 + 1.6 board + 8)
def Hm(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
Rz = lambda a: np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
# A1: flange face z744.29 & scan centre (-90.55,-132.92) -> (SCAN, ZTOP+SO), rotated 180 deg
Ta = Hm(Rz(math.pi), [SCAN[0], SCAN[1], ZTOP + SO]) @ Hm(np.eye(3), [90.55, 132.92, -744.29])
DROP = ()  # full kit stack
a1 = import_step("a1.step"); a1L = [(c.label, xf(s, Ta)) for c in a1.children if not c.label.startswith(DROP) for s in c.solids()]
stands = [s for l, s in a1L if l.startswith("stand")]
print("kit stand bottoms (should be pad top", ZTOP + 3.0, "):", sorted({round(s.bounding_box().min.Z, 2) for s in stands}))
print("A1 (no stands) lowest above cover top:", round(min(s.bounding_box().min.Z for l, s in a1L if not l.startswith('stand')) - ZTOP, 2))
print("A1 L z range:", round(min(s.bounding_box().min.Z for _, s in a1L), 2), round(max(s.bounding_box().max.Z for _, s in a1L), 2))
print("A1 L x range:", round(min(s.bounding_box().min.X for _, s in a1L), 1), round(max(s.bounding_box().max.X for _, s in a1L), 1))
# check flange holes land on LIDAR_HOLES
for p in [(-118.55, -160.92), (-118.55, -104.92), (-48.55, -152.92), (-48.55, -112.92)]:
    q = Ta @ np.array([p[0], p[1], 744.29, 1]); print("  A1 hole ->", np.round(q[:3], 2))
# bridge: holes centre (-15.77, 3.215), board bottom z -0.79; rotate +90 so USB end points -Y
BC, BSO = (42.0, 0.0), 0.5  # v1 0.5 mm rubber spacers on the cover (M2 inserts)
Tb = Hm(Rz(math.pi / 2), [BC[0], BC[1], ZTOP + BSO]) @ Hm(np.eye(3), [15.77, -3.215, 0.79])
br = import_step("a1_bridge.step"); brL = [(s.label, xf(s, Tb)) for s in br.solids()]
bbz = [s.bounding_box() for _, s in brL]
print("bridge L bbox x", round(min(b.min.X for b in bbz), 1), round(max(b.max.X for b in bbz), 1),
      "y", round(min(b.min.Y for b in bbz), 1), round(max(b.max.Y for b in bbz), 1),
      "z", round(min(b.min.Z for b in bbz), 1), round(max(b.max.Z for b in bbz), 1))
for p in [(-25.27, -3.91), (-25.27, 10.34), (-6.27, -3.91), (-6.27, 10.34)]:
    q = Tb @ np.array([p[0], p[1], -0.79, 1]); print("  bridge hole ->", np.round(q[:3], 2))
np.save("T_bridge_L.npy", Tb)  # before the slow checks; T_a1_L.npy is check_lidar.py's
def vol(a, b):
    if not bbox_hit(a, b): return 0.0
    r = a.intersect(b); r = [] if r is None else r if isinstance(r, list) else [r]; return sum(x.volume for x in r)
ca = sum(vol(cover, s) for _, s in a1L); cb = sum(vol(cover, s) for _, s in brL)
ab = sum(vol(s1, s2) for _, s1 in a1L for _, s2 in brL if bbox_hit(s1, s2))
print(f"cover x A1 {ca:.3f} | cover x bridge {cb:.3f} | A1 x bridge {ab:.3f} mm3")
lo = min(s.bounding_box().min.Z for _, s in a1L); print("A1 lowest above cover top:", round(lo - ZTOP, 2), "mm")
gap = min(s1.distance_to(s2) for _, s1 in a1L for _, s2 in brL if s1.bounding_box().min.Z < 100)
print("min A1 <-> bridge distance:", round(gap, 2), "mm")
