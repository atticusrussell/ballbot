import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Study (no calls): PCB rotated 180 deg about Z - where its holes/edge connectors go, cover windows they need, Orin fit."""
import sys, numpy as np
from build123d import import_step, Face, GeomType
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from check import xf, H2L
from place import T, X
from orin_housing import (BOARD_HOLES, ORIN_HOLES, RAISE, ORIG_SHELL, DZ, HALF_L, W_OUT, T as WALL, FAN_C, PLATE_SCREWS)
Rz = np.diag([-1, -1, 1.0])
c = np.mean(ORIN_HOLES, axis=0); print("Orin hole-rect centre", np.round(c, 2), "| board centre (0, 0)")
print("holes rotated about board centre:", [tuple(np.round(-np.array(h), 2)) for h in BOARD_HOLES])
print("holes rotated about Orin rect centre:", [tuple(np.round(2 * c - np.array(h), 2)) for h in BOARD_HOLES])
print("board bbox if rotated about Orin rect centre: x", round(2 * c[0] - 65, 1), round(2 * c[0] + 65, 1), "y", round(2 * c[1] - 50, 1), round(2 * c[1] + 50, 1),
      "| cover inner x ±", HALF_L - WALL, "y ±", W_OUT - WALL)
# cover windows (from the DOFBOT shell walls, in L incl. RAISE)
sh = import_step(str(ORIG_SHELL)); wins = []
for f in sh.faces():
    n = f.normal_at()
    if f.geom_type == GeomType.PLANE and abs(abs(n.Y) - 1) < 1e-3 and abs(f.center().Y) > 53:
        for w in f.inner_wires():
            b = Face(w).bounding_box()
            if Face(w).area > 30: wins.append((round(f.center().Y), round(b.min.X, 1), round(b.max.X, 1), round(b.min.Z + DZ + RAISE, 1), round(b.max.Z + DZ + RAISE, 1)))
print("cover windows (wall y, x0, x1, z0, z1):"); [print("  ", w) for w in sorted(wins)]
# PCB parts near the long edges (connectors), current pose
import sys as _s
if "--orin" in _s.argv: pass
pcb = next(ch for ch in import_step("housing.step").children if ch.label.startswith("YB-EVV01"))
M = T([0, 0, RAISE]) @ H2L @ np.load("D_pcb_snapped.npy")
edge = []
for ch in pcb.children:
    for s in ch.solids():
        b = xf(s, M).bounding_box()
        if (b.max.Y > 44 or b.min.Y < -44 or b.max.X > 58 or b.min.X < -58) and b.max.Z - b.min.Z > 2:
            edge.append((ch.label[:28], round(b.min.X, 1), round(b.max.X, 1), round(b.min.Y, 1), round(b.max.Y, 1), round(b.max.Z, 1)))
print("PCB edge parts now (label, x0, x1, y0, y1, ztop):"); [print("  ", e) for e in sorted(edge, key=lambda e: (e[3], e[1]))]
o = import_step("orin_assm.step"); Mo = T([0, 0, RAISE]) @ H2L @ X["orin"]
ob = [xf(s, Mo).bounding_box() for s in o.solids()]
lo = [min(getattr(b.min, a) for b in ob) for a in "XYZ"]; hi = [max(getattr(b.max, a) for b in ob) for a in "XYZ"]
print("Orin bbox L now: x", round(lo[0], 1), round(hi[0], 1), "y", round(lo[1], 1), round(hi[1], 1), "z", round(lo[2], 1), round(hi[2], 1))
print("  shifted (+10.6, +7.94): x", round(lo[0] + 10.6, 1), round(hi[0] + 10.6, 1), "y", round(lo[1] + 7.94, 1), round(hi[1] + 7.94, 1))
print("  rotated 180 about (0,0): x", round(-hi[0], 1), round(-lo[0], 1), "y", round(-hi[1], 1), round(-lo[1], 1))
print("fan grille centre", FAN_C)
