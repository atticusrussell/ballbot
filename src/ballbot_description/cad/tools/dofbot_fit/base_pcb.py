import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Where does the base touch the PCB (bottom-side parts / pins)?"""
import numpy as np
from build123d import import_step
from check import xf, bbox_hit, housing_part, H2L
base = housing_part("orin_housing_base")
pcb = next(c for c in import_step("housing.step").children if c.label.startswith("YB-EVV01"))
Dp = H2L @ np.load("D_pcb_snapped.npy")
for s in pcb.solids():
    sb = s.bounding_box()
    if sb.min.Z > -90 and False: continue
    sL = xf(s, Dp)
    if not bbox_hit(base, sL): continue
    r = base.intersect(sL); r = [] if r is None else r if isinstance(r, list) else [r]
    for x in r:
        if x.volume > 1e-3:
            b = x.bounding_box(); print(round(x.volume, 2), s.label, [round(v, 1) for v in (b.min.X, b.max.X, b.min.Y, b.max.Y, b.min.Z, b.max.Z)])
