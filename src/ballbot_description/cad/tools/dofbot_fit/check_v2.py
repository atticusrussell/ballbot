import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Max pose error of the v2 standoffs + cover in dofbot_pcb_housing (1 call)."""
import json, numpy as np
from bb import api
from place import HOUSING, X, T, asm, state
RY = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
a = api.get(asm(HOUSING) + "?includeNonSolids=true").json(); json.dump(a, open("housing_v2_now.json", "w"))
occ = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in a["rootAssembly"]["occurrences"]}
def mm(M): M = M.copy(); M[:3, 3] *= 1000; return M
v2 = state["v2"]; done = state["mates"]
for n, (i, h, z) in v2["standoffs"].items():
    if f"fasten {n}" in done: print(f"{n:<20} err {np.abs(mm(occ[(i,)]) - X['housing'] @ T([h[0], h[1], z], RY)).max():.4f} mm")
print(f"{'cover':<20} err {np.abs(mm(occ[(v2['ids']['cover'],)]) - X['housing']).max():.4f} mm")
