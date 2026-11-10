import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Pose error of every hw_mates.py part vs its intended pose (1 call; --chassis for still_chassis_assm)."""
import json, sys, numpy as np
sys.argv.append("--plan-only")
from bb import api
from place import HOUSING, CHASSIS, X, asm, state
import mates as m
import hw_mates as h
e = CHASSIS if "--chassis" in sys.argv else HOUSING
occ = {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in api.get(asm(e)).json()["rootAssembly"]["occurrences"]}
pre = (state["repair"]["chassis"],) if e == CHASSIS else ()
worst = 0
for name, (cp, Mc), (pp, Mp) in h.PLAN:
    if name == "fasten pcb on standoff": continue
    want = occ[pp] @ Mp @ np.linalg.inv(Mc); err = np.abs(occ[cp] - want).max(); worst = max(worst, err)
    if err > 1e-3: print(f"{name:<28} err {err:.4f}")
print(f"{len(h.PLAN)} hw mates, max error {worst:.4f} mm")
