import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2 board mount: PCB (raised) vs base (bosses) and the 6 board standoffs (stand-ins for doc bn_3318 M2.5x10, 5.5 AF)."""
import sys, math, numpy as np
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from build123d import import_step, Pos, RegularPolygon, extrude
from check import xf, H2L, housing_part, clash
from orin_housing import RAISE, BOARD_HOLES, BOSS_TOP, PCB_Z
from place import T
pcb = next(c for c in import_step("housing.step").children if c.label.startswith("YB-EVV01"))
M = T([0, 0, 0], np.diag([-1.0, -1.0, 1.0])) @ T([0, 0, RAISE]) @ H2L @ np.load("D_pcb_snapped.npy")  # v2b rotation in L
P = [(c.label, xf(s, M)) for c in pcb.children for s in c.solids()]
board = next(s for l, s in P if l == "Board_step"); print("board z", round(board.bounding_box().min.Z, 3), "expect", PCB_Z)
so = [Pos(x, y, BOSS_TOP) * extrude(RegularPolygon(5.5 / math.sqrt(3), 6), amount=10).solids()[0] for x, y in BOARD_HOLES]
clash("base", housing_part("orin_housing_base"), P)
for (x, y), s in zip(BOARD_HOLES, so): clash(f"standoff ({x}, {y})", s, [(l, p) for l, p in P if l != "Board_step"])
