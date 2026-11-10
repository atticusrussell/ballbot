import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2b (PCB + Orin turned 180 about the board centre): interference of PCB / Orin with the new cover and base (no calls)."""
import sys, numpy as np
from build123d import import_step
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from check import xf, H2L, housing_part, clash
from place import T, X, L2H
from orin_housing import RAISE
G = T([0, 0, 0], np.diag([-1.0, -1.0, 1.0]))
pcb = next(c for c in import_step("housing.step").children if c.label.startswith("YB-EVV01"))
P = [(c.label, xf(s, G @ T([0, 0, RAISE]) @ H2L @ np.load("D_pcb_snapped.npy"))) for c in pcb.children for s in c.solids()]
o = import_step("orin_assm.step"); Mo = np.linalg.inv(L2H) @ X["orin"]  # X['orin'] already has RAISE + rotation
O = [(c.label or "orin", xf(s, Mo)) for c in o.children for s in c.solids()]
cover, base = housing_part("orin_housing_cover"), housing_part("orin_housing_base")
for n, part in (("cover", cover), ("base", base)):
    clash(f"{n} x PCB", part, P); clash(f"{n} x Orin", part, O)
