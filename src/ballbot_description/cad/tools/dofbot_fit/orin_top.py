import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Tallest Orin solids in L (fan / heatsink location for the cover grille)."""
import numpy as np
from build123d import import_step
from check import H2L
D = H2L @ np.load("D_orin.npy")
rows = []
for s in import_step("orin_assm.step").solids():
    b = s.bounding_box(); lo = D @ [b.min.X, b.min.Y, b.min.Z, 1]; hi = D @ [b.max.X, b.max.Y, b.max.Z, 1]
    rows.append((hi[2], s.label, lo[:3], hi[:3]))
rows.sort(key=lambda r: -r[0])
for z, l, lo, hi in rows[:12]:
    print(f"{l[:28]:28s} x {lo[0]:7.1f}..{hi[0]:7.1f} y {lo[1]:7.1f}..{hi[1]:7.1f} z {lo[2]:6.1f}..{hi[2]:6.1f}")
