import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""robot_assm: move a housing leaf that keeps its own in-context position (SOT-223) onto the housing pose. 2 calls + check."""
import json, sys, numpy as np
from bb import api
from place import asm, state, set_xf
import mates as m
ROBOT = "83ac8f3e1b8a64890594cf88"; pre = ("MwVYb3BSH0u/35hbh", state["repair"]["chassis"]); B = "MTus/tENlOyaO79+2"
occ = lambda r: {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in r["rootAssembly"]["occurrences"]}
H = {tuple(k.split("|")): m.mm(np.array(v)) for k, v in json.load(open("verify_housing.json"))["H"].items()}
R = occ(api.get(asm(ROBOT) + "?includeNonSolids=true").json()); G = R[pre + (B,)] @ np.linalg.inv(H[(B,)])
bad = [k for k, v in H.items() if pre + k in R and np.abs(R[pre + k] - G @ v).max() > 1e-3]
print("off in robot:", bad)
if "--fix" in sys.argv:
    for k in bad: set_xf(ROBOT, [list(pre + k)], G @ H[k] @ np.linalg.inv(R[pre + k]), relative=True)
    R = occ(api.get(asm(ROBOT) + "?includeNonSolids=true").json())
    print("max error now", round(max(np.abs(R[pre + k] - G @ v).max() for k, v in H.items() if pre + k in R), 4), "mm")
