import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""PCB board bbox + holes in frame L (should match orin_housing PCB_HOLES / ORIN_HOLES)."""
import numpy as np
from build123d import import_step
from boardholes_lib import holes
from check import xf, H2L
pcb = next(c for c in import_step("housing.step").children if c.label.startswith("YB-EVV01"))
board = next(c for c in pcb.children if c.label == "Board_step")
bL = xf(board, H2L @ np.load("D_pcb_snapped.npy"))
print("board L bbox", bL.bounding_box())
for d, x, y in holes(bL): print(f"  hole d{d} at ({x:.2f}, {y:.2f})")
